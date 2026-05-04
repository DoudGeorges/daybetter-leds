"""DayBetter BLE protocol.

Packet format, CRC-16, GATT profiles, and handshake constants.
For the full protocol reference, see PROTOCOL.md.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
_ENV = _ROOT / ".env"
if _ENV.exists():
    for _line in _ENV.read_text(encoding="utf-8").splitlines():
        _line = _line.strip()
        if _line and not _line.startswith("#") and "=" in _line:
            _k, _, _v = _line.partition("=")
            os.environ.setdefault(_k.strip(), _v.strip())

_CRC16_TABLE: tuple[int, ...] = (
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
)


def crc16_modbus(data: bytes | bytearray) -> int:
    """CRC-16/MODBUS over *data*."""
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


_HEADER = 0xA0


def build_packet(cmd: int, payload: bytes = b"") -> bytes:
    """Build a BLE write packet: ``[0xA0][cmd][len][payload…][crc16]``."""
    length = len(payload) + 3
    buf = bytearray([_HEADER, cmd & 0xFF, length & 0xFF]) + bytearray(payload)
    crc = crc16_modbus(buf)
    buf.append(crc & 0xFF)
    buf.append((crc >> 8) & 0xFF)
    return bytes(buf)


class Cmd(IntEnum):
    """BLE command IDs used by both QC and HC device families."""

    HC_SET_COLOR = 0x04
    POWER = 0x11
    SET_MODE = 0x12
    BRIGHTNESS = 0x13
    QC_SET_COLOR = 0x15
    SPEED = 0x16


@dataclass(frozen=True, slots=True)
class DeviceProfile:
    """GATT UUID set for a specific device hardware revision."""

    name: str
    service: str
    write: str
    notify: str
    scan: str
    device_class: int  # 1 = QC (Smart Light), 2 = HC (Magic Light)

    @staticmethod
    def _full_uuid(short: str) -> str:
        return f"0000{short.lower()}-0000-1000-8000-00805f9b34fb"

    @property
    def service_uuid(self) -> str:
        return self._full_uuid(self.service)

    @property
    def write_uuid(self) -> str:
        return self._full_uuid(self.write)

    @property
    def notify_uuid(self) -> str:
        return self._full_uuid(self.notify)

    @property
    def is_qc(self) -> bool:
        return self.device_class == 1

    @property
    def is_hc(self) -> bool:
        return self.device_class == 2


PROFILES: dict[str, DeviceProfile] = {
    "P001": DeviceProfile("Smart Light", "FF10", "FF12", "FF11", "1800", 1),
    "P00B": DeviceProfile("Magic Light", "AE30", "AE01", "AE02", "AF30", 2),
    "P010": DeviceProfile("Magic Light", "E010", "A010", "F010", "C010", 2),
    "P016": DeviceProfile("Wi-Fi Light", "E016", "A016", "F016", "C016", 1),
    "P019": DeviceProfile("Magic Light", "E019", "A019", "F019", "C019", 2),
    "P01A": DeviceProfile("Magic Light", "E01A", "A01A", "F01A", "C01A", 2),
    "P01B": DeviceProfile("Smart Light", "E01B", "A01B", "F01B", "C01B", 1),
    "P020": DeviceProfile("Magic Light", "E020", "A020", "F020", "C020", 2),
    "P026": DeviceProfile("Smart Light", "E026", "A026", "F026", "C026", 1),
    "P028": DeviceProfile("Magic Light", "E028", "A028", "F028", "C028", 2),
    "P02A": DeviceProfile("Magic Light", "E02A", "A02A", "F02A", "C02A", 2),
    "P02C": DeviceProfile("Smart Light", "E02C", "A02C", "F02C", "C02C", 1),
    "P031": DeviceProfile("Smart Light", "E031", "A031", "F031", "C031", 1),
    "P036": DeviceProfile("Magic Light", "E036", "A036", "F036", "C036", 2),
    "P044": DeviceProfile("Magic Light", "E044", "A044", "F044", "C044", 2),
}
