"""BLE relay daemon (Raspberry Pi).

Maintains a persistent BLE connection and forwards UDP color commands
from capture.py to the LED strip.

    python ambient/relay.py
"""

from __future__ import annotations

import asyncio
import logging
import os
import struct
import sys
import time
from pathlib import Path

from bleak import BleakClient

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from protocol import CCHIP_TABLE, Cmd, PROFILES, build_packet

# Configuration

ADDRESS: str = os.environ.get("BLE_ADDRESS", "80:AC:C8:62:62:09")
PROFILE: str = os.environ.get("BLE_PROFILE", "P031")
HOST: str = "0.0.0.0"
PORT: int = int(os.environ.get("RELAY_PORT", "9000"))

# UDP command IDs

log = logging.getLogger("relay")

_MAGIC = 0xDB
_CMD_COLOR = 0x01
_CMD_MODE = 0x02
_CMD_BRIGHTNESS = 0x03
_CMD_ON = 0x04
_CMD_OFF = 0x05
_CMD_HANDSHAKE = 0x06
_CMD_PING = 0xFF

# BLE connection with auto-reconnect


class _BLERelay:
    __slots__ = (
        "address", "profile", "client", "connected",
        "_notify_data", "_notify_event", "_lock",
        "_write_count", "_total_latency",
    )

    _MAX_RECONNECT = 5

    def __init__(self) -> None:
        self.address = ADDRESS
        if PROFILE not in PROFILES:
            raise ValueError(f"Unknown profile {PROFILE!r}. Valid: {', '.join(sorted(PROFILES))}")
        self.profile = PROFILES[PROFILE]
        self.client: BleakClient | None = None
        self.connected = False
        self._notify_data = bytearray()
        self._notify_event = asyncio.Event()
        self._lock = asyncio.Lock()
        self._write_count = 0
        self._total_latency = 0.0

    def _on_notify(self, _sender: int, data: bytearray) -> None:
        self._notify_data = data
        self._notify_event.set()

    async def connect(self) -> None:
        log.info("Connecting to %s...", self.address)
        self.client = BleakClient(self.address)
        await self.client.connect()
        await self.client.start_notify(
            self.profile.notify_uuid, self._on_notify,
        )
        self.connected = True
        log.info("Connected. Handshake...")
        await self.handshake()

    async def handshake(self) -> None:
        """CCHIP challenge-response (required before commands)."""
        if self.client is None:
            raise RuntimeError("Not connected")

        cchip = b"CCHIP"
        uuid = self.profile.write_uuid

        # Phase 1: request challenge
        self._notify_event.clear()
        await self.client.write_gatt_char(
            uuid, build_packet(0x00, cchip), response=True,
        )
        await asyncio.wait_for(self._notify_event.wait(), timeout=3.0)

        rx = self._notify_data
        if len(rx) < 10 or rx[3:8] != cchip:
            raise RuntimeError(f"Handshake phase 1 failed: {rx.hex()}")

        c1, c2 = rx[8], rx[9]
        r1 = CCHIP_TABLE[((c1 >> 4) + (c1 & 0x0F)) % len(CCHIP_TABLE)]
        r2 = CCHIP_TABLE[((c2 >> 4) + (c2 & 0x0F)) % len(CCHIP_TABLE)]

        # Phase 2: send response
        self._notify_event.clear()
        await self.client.write_gatt_char(
            uuid, build_packet(0x01, cchip + bytes([r1, r2])), response=True,
        )
        await asyncio.wait_for(self._notify_event.wait(), timeout=3.0)

        rx2 = self._notify_data
        if len(rx2) < 9 or rx2[8] != 0x00:
            raise RuntimeError(f"Handshake phase 2 failed: {rx2.hex()}")

        log.info("Handshake OK")
        await self.write(build_packet(Cmd.SET_MODE, b"\x01\x00"))
        await asyncio.sleep(0.05)

    async def write(self, data: bytes) -> None:
        """Write to the BLE characteristic with latency tracking."""
        if self.client is None:
            raise RuntimeError("Not connected")
        async with self._lock:
            try:
                t0 = time.perf_counter()
                await self.client.write_gatt_char(
                    self.profile.write_uuid, data, response=True,
                )
                self._write_count += 1
                self._total_latency += time.perf_counter() - t0
            except Exception as exc:
                log.warning("Write failed: %s. Reconnecting...", exc)
                self.connected = False
                await self._reconnect()

    async def _reconnect(self) -> None:
        for attempt in range(self._MAX_RECONNECT):
            try:
                if self.client:
                    try:
                        await self.client.disconnect()
                    except Exception:
                        pass
                await self.connect()
                return
            except Exception as exc:
                log.warning(
                    "Reconnect %d/%d failed: %s",
                    attempt + 1, self._MAX_RECONNECT, exc,
                )
                await asyncio.sleep(1)
        log.error("Failed after %d attempts.", self._MAX_RECONNECT)

    @property
    def avg_latency_ms(self) -> float:
        if self._write_count == 0:
            return 0.0
        return (self._total_latency / self._write_count) * 1000


# UDP server


class _Server:
    __slots__ = ("ble", "_writes", "_start")

    def __init__(self, ble: _BLERelay) -> None:
        self.ble = ble
        self._writes = 0
        self._start = time.perf_counter()

    async def start(self) -> asyncio.DatagramTransport:
        loop = asyncio.get_running_loop()
        transport, _ = await loop.create_datagram_endpoint(
            lambda: _Protocol(self),
            local_addr=(HOST, PORT),
        )
        log.info("UDP listening on %s:%d", HOST, PORT)
        return transport

    async def handle(
        self,
        data: bytes,
        addr: tuple[str, int],
        transport: asyncio.DatagramTransport,
    ) -> None:
        if len(data) < 7 or data[0] != _MAGIC:
            return

        cmd = data[1]
        r, g, b, w = data[2], data[3], data[4], data[5]

        if cmd == _CMD_COLOR:
            await self.ble.write(
                build_packet(Cmd.QC_SET_COLOR, bytes([r, g, b, w])),
            )
            self._writes += 1
        elif cmd == _CMD_MODE:
            await self.ble.write(build_packet(Cmd.SET_MODE, bytes([r, g])))
        elif cmd == _CMD_BRIGHTNESS:
            await self.ble.write(build_packet(Cmd.BRIGHTNESS, bytes([r])))
        elif cmd == _CMD_ON:
            await self.ble.write(build_packet(Cmd.SET_MODE, b"\x01\x00"))
        elif cmd == _CMD_OFF:
            await self.ble.write(build_packet(Cmd.POWER, b"\x00"))
        elif cmd == _CMD_HANDSHAKE:
            await self.ble.handshake()
        elif cmd == _CMD_PING:
            lat = int(self.ble.avg_latency_ms * 10)
            transport.sendto(
                struct.pack("!BHI", _MAGIC, lat, self._writes), addr,
            )

        if self._writes > 0 and self._writes % 100 == 0:
            elapsed = time.perf_counter() - self._start
            log.info(
                "%d writes | %.1f/s | BLE avg %.1fms",
                self._writes, self._writes / elapsed, self.ble.avg_latency_ms,
            )


class _Protocol(asyncio.DatagramProtocol):
    __slots__ = ("server", "transport")

    def __init__(self, server: _Server) -> None:
        self.server = server
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport: asyncio.DatagramTransport) -> None:
        self.transport = transport

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        if self.transport is None:
            return
        asyncio.create_task(self.server.handle(data, addr, self.transport))


# Main loop


async def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    ble = _BLERelay()
    await ble.connect()

    server = _Server(ble)
    transport = await server.start()

    log.info("Relay ready.")
    log.info("  BLE: %s (%s)", ADDRESS, PROFILE)
    log.info("  UDP: %s:%d", HOST, PORT)

    try:
        while True:
            await asyncio.sleep(3600)
    except KeyboardInterrupt:
        log.info("Shutting down...")
    finally:
        transport.close()
        if ble.client:
            await ble.client.disconnect()
        log.info("Done.")


if __name__ == "__main__":
    asyncio.run(main())
