# DayBetter BLE Protocol

Reverse-engineered from the DayBetter Android APK (v1.3.3, `com.th.*` packages).

## Device Families

Two device families exist, named after Chinese labels found in the decompiled source.

| Class | Code | Chinese         | English     | Type           |
| ----- | ---- | --------------- | ----------- | -------------- |
| 1     | QC   | 全彩 (QuánCǎi) | Smart Light | 3-channel RGB  |
| 2     | HC   | 幻彩 (HuànCǎi) | Magic Light | Addressable IC |

Command logic:

- QC: `com.th.qc.command.CommandUtil`
- HC: `com.th.hc.command.HCCommandUtil`

Additional device classes appear in `th_dev_cfg.json` (3 = PIR, 39 = Socket, 43 = Sensor) but fall outside the scope of this document.

## Packet Format

Derived from `StringToSixthUtils.pack` and `BlockUtils.crc16Check`.

```
[header] [cmd] [len] [payload...] [crc_lo] [crc_hi]
```

| Field     | Size     | Description                                     |
| --------- | -------- | ----------------------------------------------- |
| `header`  | 1 byte   | `0xA0` (app → device) or `0xA1` (device → app) |
| `cmd`     | 1 byte   | Command ID                                      |
| `len`     | 1 byte   | `len(payload) + 3`                              |
| `payload` | variable | Command-specific data                           |
| `crc`     | 2 bytes  | CRC-16/MODBUS, little-endian                    |

### CRC-16/MODBUS

- Polynomial: `0xA001`
- Initial value: `0xFFFF`
- Scope: all bytes preceding the CRC (header, cmd, len, payload)
- Byte order: little-endian (low byte first)

```python
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc & 0xFFFF
```

### Packet Builder

```python
def build_packet(cmd: int, payload: bytes = b"") -> bytes:
    length = len(payload) + 3
    buf = bytearray([0xA0, cmd & 0xFF, length & 0xFF]) + bytearray(payload)
    crc = crc16_modbus(buf)
    buf.append(crc & 0xFF)
    buf.append((crc >> 8) & 0xFF)
    return bytes(buf)
```

## CCHIP Handshake

Derived from `BlueConnectionUtil`. The device rejects all commands until this challenge-response handshake succeeds.

### Phase 1: Challenge

The host sends the `CCHIP` identifier. The device responds with two challenge bytes at `rx[8]` and `rx[9]`.

```
TX: build_packet(0x00, b"CCHIP")
RX: [0xA1] [0x00] [len] [CCHIP] [c1] [c2] [...] [crc_lo] [crc_hi]
```

### Phase 2: Response

The host computes a response using either the lookup table method or the XOR method, then sends it back.

**Lookup table method** — 62-entry table covering ASCII `0–9`, `A–Z`, `a–z` (`0x30`–`0x7A`):

```python
r1 = CCHIP_TABLE[((c1 >> 4) + (c1 & 0x0F)) % 62]
r2 = CCHIP_TABLE[((c2 >> 4) + (c2 & 0x0F)) % 62]
```

**XOR method** — a simpler derivation found in an alternative APK build:

```python
r1 = c1 ^ 36
r2 = c2 ^ 76
```

The response is sent as:

```
TX: build_packet(0x01, b"CCHIP" + bytes([r1, r2]))
RX: [0xA1] [0x01] [len] [...] [status] [...] [crc_lo] [crc_hi]
```

Success: `rx[8] == 0x00`.

## QC Commands (Smart Light)

Derived from `com.th.qc.command.CommandUtil` (`deviceClass=1`).

| Cmd    | Name            | Payload                        | Notes                                   |
| ------ | --------------- | ------------------------------ | --------------------------------------- |
| `0x10` | Read properties | `[0x01]`                       | Response on notify characteristic       |
| `0x11` | Power           | `[0x01]` on, `[0x00]` off     |                                         |
| `0x12` | Set mode        | `[mode, subMode]`              | `subMode`: `0x00` static, `0xFF` effect |
| `0x13` | Brightness      | `[level]`                      | 0–100                                   |
| `0x14` | Read pixel      |                                | Also used by HC                         |
| `0x15` | Set color       | `[R, G, B, W]`                 | Requires mode `[0x01, 0x00]` first      |
| `0x16` | Effect speed    | `[speed]`                      | 0–100                                   |
| `0x17` | Mic sensitivity | `[level]`                      | 0–100                                   |
| `0x1A` | Mic on/off      | `[0x01]` or `[0x00]`          |                                         |
| `0x20` | Cold/warm white | `[0xFF, 0xFF, cold, warm]`     | Requires mode `[0x00, 0x00]` first      |
| `0x21` | Rhythm control  | `[0x01]` start, `[0x00]` stop | Hardware mic                            |
| `0x22` | RGB order       |                                | Channel reordering                      |
| `0x31` | Sync time       |                                |                                         |
| `0x34` | Write timer     |                                |                                         |
| `0x35` | Read timers     |                                |                                         |
| `0x36` | Countdown       |                                |                                         |
| `0x51` | Phone rhythm    |                                | Phone-mic brightness control            |

### QC Mode IDs

Derived from `qc_mode.json`.

| Category | IDs                                                                                                                  |
| -------- | -------------------------------------------------------------------------------------------------------------------- |
| Static   | 2 red, 3 blue, 4 green, 5 cyan, 6 yellow, 7 purple, 8 white                                                         |
| Beat     | 9 three-color, 10 seven-color                                                                                        |
| Gradual  | 11 three-color, 12 seven-color, 13 red, 14 green, 15 blue, 16 yellow, 17 cyan, 18 purple, 19 white, 20 R/G, 21 R/B, 22 G/B |
| Flash    | 23 seven-color, 24 red, 25 green, 26 blue, 27 yellow, 28 cyan, 29 purple, 30 white                                  |

### QC Mic Modes

Activate by sending `SET_MODE [160, subMode]` followed by `RHYTHM_CONTROL [0x01]`.

| subMode | Style   |
| ------- | ------- |
| 0       | Classic |
| 1       | Soft    |
| 2       | Dynamic |
| 3       | Disco   |

## HC Commands (Magic Light)

Derived from `com.th.hc.command.HCCommandUtil` (`deviceClass=2`).

| Cmd    | Name             | Payload                                | Notes                                  |
| ------ | ---------------- | -------------------------------------- | -------------------------------------- |
| `0x04` | Set color        | `[R, G, B, seg1, ..., segN]`           | Segments = IC indices                  |
| `0x05` | Gradient toggle  | `[0x01]` or `[0x00]`                  |                                        |
| `0x06` | DIY params       | `[mode, brightness, speed, count, 0]`  |                                        |
| `0x07` | DIY colors       | `[mode, R1, G1, B1, R2, G2, B2, ...]` | Up to ~10 colors                       |
| `0x09` | Preset mode      | `[mode_id, speed]`                     |                                        |
| `0x12` | Switch to static | `[0xC8]`                               | Pre-built packet: `A0 12 04 C8 81 77` |
| `0x23` | White channel    | `[level]`                              | RGBW white level                       |

### HC Color Workflow

1. Send static mode switch: `A0 12 04 C8 81 77` (pre-computed CRC).
2. Wait 50 ms.
3. Send `build_packet(0x04, [R, G, B, seg1, seg2, ..., segN])`.

Default segments: 1 through 20 (20 ICs).

## Device Profiles

Derived from `th_dev_cfg.json`.

UUIDs follow the format `0000XXXX-0000-1000-8000-00805f9b34fb`, where `XXXX` is the short ID below.

| Profile | Name        | Service | Write | Notify | Scan | Class |
| ------- | ----------- | ------- | ----- | ------ | ---- | ----- |
| P001    | Smart Light | FF10    | FF12  | FF11   | 1800 | QC    |
| P00B    | Magic Light | AE30    | AE01  | AE02   | AF30 | HC    |
| P010    | Magic Light | E010    | A010  | F010   | C010 | HC    |
| P016    | Wi-Fi Light | E016    | A016  | F016   | C016 | QC    |
| P019    | Magic Light | E019    | A019  | F019   | C019 | HC    |
| P01A    | Magic Light | E01A    | A01A  | F01A   | C01A | HC    |
| P01B    | Smart Light | E01B    | A01B  | F01B   | C01B | QC    |
| P020    | Magic Light | E020    | A020  | F020   | C020 | HC    |
| P026    | Smart Light | E026    | A026  | F026   | C026 | QC    |
| P028    | Magic Light | E028    | A028  | F028   | C028 | HC    |
| P02A    | Magic Light | E02A    | A02A  | F02A   | C02A | HC    |
| P02C    | Smart Light | E02C    | A02C  | F02C   | C02C | QC    |
| P031    | Smart Light | E031    | A031  | F031   | C031 | QC    |
| P036    | Magic Light | E036    | A036  | F036   | C036 | HC    |
| P044    | Magic Light | E044    | A044  | F044   | C044 | HC    |

### Advertisement Names

Devices advertise as `SMART LIGHT`, `MAGIC LIGHT`, `DAYBETTER`, `WI-FI LIGHT`, `BLE BULB`, `CCHIP`, or any name prefixed with `P0`.

## BLE Write Modes

Two write strategies are available:

- `write_gatt_char(uuid, data)` — fire-and-forget, ~80 ms
- `write_gatt_char(uuid, data, response=True)` — acknowledged, ~83 ms, guaranteed delivery
