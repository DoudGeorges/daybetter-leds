"""BLE relay daemon (Raspberry Pi).

Maintains a persistent BLE connection and forwards UDP colour commands
from ``capture.py`` to the LED strip.
"""

from __future__ import annotations

import asyncio
import logging
import os
import struct
import sys
import time
from enum import IntEnum
from pathlib import Path
from typing import TYPE_CHECKING

from bleak import BleakClient

if TYPE_CHECKING:
    from bleak import BleakGATTCharacteristic

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from protocol import Cmd, DeviceProfile, PROFILES, build_packet  # noqa: E402

__all__: list[str] = []

ADDRESS: str = os.environ.get("BLE_ADDRESS", "AA:BB:CC:DD:EE:FF")
PROFILE: str = os.environ.get("BLE_PROFILE", "P031")
HOST: str = "0.0.0.0"
PORT: int = int(os.environ.get("RELAY_PORT", "9000"))

_MIN_WRITE_INTERVAL: float = 0.030  # 30 ms -> ~33 Hz BLE cap

log = logging.getLogger("relay")


class UDPCmd(IntEnum):
    """Wire IDs for the PC-to-Pi UDP micro-protocol."""

    COLOR = 0x01
    MODE = 0x02
    BRIGHTNESS = 0x03
    ON = 0x04
    OFF = 0x05
    HANDSHAKE = 0x06
    PING = 0xFF


_MAGIC = 0xDB


class _BLERelay:
    """Persistent BLE connection with auto-reconnect and write coalescing."""

    __slots__ = (
        "_address", "_profile", "_client", "_connected",
        "_notify_data", "_notify_event", "_lock",
        "_write_count", "_total_latency", "_last_write_t",
    )

    _MAX_RECONNECT = 5
    _HANDSHAKE_RETRIES = 3

    def __init__(self, address: str, profile: DeviceProfile) -> None:
        self._address = address
        self._profile = profile
        self._client: BleakClient | None = None
        self._connected = False
        self._notify_data = bytearray()
        self._notify_event = asyncio.Event()
        self._lock = asyncio.Lock()
        self._write_count = 0
        self._total_latency = 0.0
        self._last_write_t = 0.0

    def _on_notify(
        self, _characteristic: BleakGATTCharacteristic, data: bytearray,
    ) -> None:
        self._notify_data = data
        self._notify_event.set()

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def client(self) -> BleakClient | None:
        return self._client

    @property
    def avg_latency_ms(self) -> float:
        if self._write_count == 0:
            return 0.0
        return (self._total_latency / self._write_count) * 1000.0

    async def connect(self) -> None:
        """Connect to the device and perform the CCHIP handshake."""
        log.info("Connecting to %s ...", self._address)
        self._client = BleakClient(self._address)
        await self._client.connect()
        await self._client.start_notify(
            self._profile.notify_uuid, self._on_notify,
        )
        self._connected = True
        log.info("Connected. Starting handshake ...")

        last_exc: Exception | None = None
        for attempt in range(1, self._HANDSHAKE_RETRIES + 1):
            try:
                await self._handshake()
                return
            except Exception as exc:
                last_exc = exc
                log.warning(
                    "Handshake attempt %d/%d failed: %s",
                    attempt, self._HANDSHAKE_RETRIES, exc,
                )
                if attempt < self._HANDSHAKE_RETRIES:
                    await asyncio.sleep(0.5)

        raise RuntimeError(
            f"Handshake failed after {self._HANDSHAKE_RETRIES} attempts",
        ) from last_exc

    async def _handshake(self) -> None:
        """CCHIP challenge-response required before the device accepts commands."""
        if self._client is None:
            raise RuntimeError("Not connected")

        cchip = b"CCHIP"
        uuid = self._profile.write_uuid

        # Phase 1: request challenge
        self._notify_event.clear()
        await self._client.write_gatt_char(
            uuid, build_packet(0x00, cchip), response=True,
        )
        await asyncio.wait_for(self._notify_event.wait(), timeout=3.0)

        rx = self._notify_data
        if len(rx) < 10 or rx[3:8] != cchip:
            raise RuntimeError(f"Phase 1 failed: {rx.hex()}")

        r1 = rx[8] ^ 36
        r2 = rx[9] ^ 76

        # Phase 2: send response
        self._notify_event.clear()
        await self._client.write_gatt_char(
            uuid, build_packet(0x01, cchip + bytes([r1, r2])), response=True,
        )
        await asyncio.wait_for(self._notify_event.wait(), timeout=3.0)

        rx2 = self._notify_data
        if len(rx2) < 9 or rx2[8] != 0x00:
            raise RuntimeError(f"Phase 2 failed: {rx2.hex()}")

        log.info("Handshake OK")
        await self._write_raw(
            build_packet(Cmd.SET_MODE, b"\x01\x00"), response=True,
        )
        await asyncio.sleep(0.05)

    async def _write_raw(self, data: bytes, *, response: bool = False) -> None:
        if self._client is None:
            raise RuntimeError("Not connected")
        t0 = time.perf_counter()
        await self._client.write_gatt_char(
            self._profile.write_uuid, data, response=response,
        )
        self._write_count += 1
        self._total_latency += time.perf_counter() - t0

    async def write_color(self, r: int, g: int, b: int, w: int = 0) -> None:
        """Fire-and-forget colour update with minimum-interval coalescing."""
        async with self._lock:
            elapsed = time.perf_counter() - self._last_write_t
            if elapsed < _MIN_WRITE_INTERVAL:
                await asyncio.sleep(_MIN_WRITE_INTERVAL - elapsed)
            try:
                await self._write_raw(
                    build_packet(Cmd.QC_SET_COLOR, bytes([r, g, b, w])),
                )
                self._last_write_t = time.perf_counter()
            except Exception as exc:
                log.warning("Colour write failed: %s", exc)
                self._connected = False
                await self._reconnect()

    async def write_cmd(self, data: bytes) -> None:
        """Acknowledged write for non-colour commands."""
        async with self._lock:
            try:
                await self._write_raw(data, response=True)
            except Exception as exc:
                log.warning("Command write failed: %s", exc)
                self._connected = False
                await self._reconnect()

    async def _reconnect(self) -> None:
        for attempt in range(self._MAX_RECONNECT):
            delay = min(2.0 ** attempt, 10.0)
            try:
                if self._client is not None:
                    try:
                        await self._client.disconnect()
                    except Exception:
                        pass
                await self.connect()
                return
            except Exception as exc:
                log.warning(
                    "Reconnect %d/%d failed: %s",
                    attempt + 1, self._MAX_RECONNECT, exc,
                )
                await asyncio.sleep(delay)
        log.error("Giving up after %d reconnect attempts.", self._MAX_RECONNECT)


class _Server:
    """UDP front-end that coalesces colour commands for the BLE back-end."""

    __slots__ = ("_ble", "_writes", "_start", "_pending")

    def __init__(self, ble: _BLERelay) -> None:
        self._ble = ble
        self._writes = 0
        self._start = time.perf_counter()
        self._pending: tuple[int, int, int, int] | None = None

    async def start(self) -> asyncio.DatagramTransport:
        loop = asyncio.get_running_loop()
        transport, _ = await loop.create_datagram_endpoint(
            lambda: _Protocol(self),
            local_addr=(HOST, PORT),
        )
        log.info("UDP listening on %s:%d", HOST, PORT)
        asyncio.create_task(self._colour_loop())
        return transport

    async def _colour_loop(self) -> None:
        """Drain the latest pending colour at BLE speed, dropping stale."""
        while True:
            if self._pending is not None:
                r, g, b, w = self._pending
                self._pending = None
                await self._ble.write_color(r, g, b, w)
                self._writes += 1
                if self._writes % 100 == 0:
                    elapsed = time.perf_counter() - self._start
                    log.info(
                        "%d writes | %.1f/s | BLE avg %.1fms",
                        self._writes,
                        self._writes / max(elapsed, 0.001),
                        self._ble.avg_latency_ms,
                    )
            else:
                await asyncio.sleep(0.005)

    async def handle(
        self,
        data: bytes,
        addr: tuple[str, int],
        transport: asyncio.DatagramTransport,
    ) -> None:
        """Dispatch a single inbound UDP datagram."""
        if len(data) < 7 or data[0] != _MAGIC:
            return

        cmd = data[1]
        r = data[2] & 0xFF
        g = data[3] & 0xFF
        b = data[4] & 0xFF
        w = data[5] & 0xFF

        if cmd == UDPCmd.COLOR:
            self._pending = (r, g, b, w)
        elif cmd == UDPCmd.MODE:
            await self._ble.write_cmd(build_packet(Cmd.SET_MODE, bytes([r, g])))
        elif cmd == UDPCmd.BRIGHTNESS:
            await self._ble.write_cmd(build_packet(Cmd.BRIGHTNESS, bytes([r])))
        elif cmd == UDPCmd.ON:
            await self._ble.write_cmd(build_packet(Cmd.SET_MODE, b"\x01\x00"))
        elif cmd == UDPCmd.OFF:
            await self._ble.write_cmd(build_packet(Cmd.POWER, b"\x00"))
        elif cmd == UDPCmd.HANDSHAKE:
            await self._ble.connect()
        elif cmd == UDPCmd.PING:
            lat = int(self._ble.avg_latency_ms * 10)
            transport.sendto(
                struct.pack("!BHI", _MAGIC, lat, self._writes), addr,
            )


class _Protocol(asyncio.DatagramProtocol):
    """asyncio datagram adapter that dispatches to ``_Server``."""

    __slots__ = ("_server", "_transport")

    def __init__(self, server: _Server) -> None:
        self._server = server
        self._transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport: asyncio.DatagramTransport) -> None:  # type: ignore[override]
        self._transport = transport

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        if self._transport is None:
            return
        asyncio.create_task(self._server.handle(data, addr, self._transport))


async def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    if PROFILE not in PROFILES:
        valid = ", ".join(sorted(PROFILES))
        log.error("Unknown profile %r. Valid: %s", PROFILE, valid)
        return

    ble = _BLERelay(ADDRESS, PROFILES[PROFILE])
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
        log.info("Shutting down ...")
    finally:
        transport.close()
        if ble.client is not None:
            await ble.client.disconnect()
        log.info("Done.")


if __name__ == "__main__":
    asyncio.run(main())
