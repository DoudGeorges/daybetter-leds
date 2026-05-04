# daybetter-leds

Reverse-engineered BLE protocol for DayBetter LED strips. Built from decompiling the Android APK. See [PROTOCOL.md](PROTOCOL.md) for the full protocol breakdown.

## Ambient

Real-time screen ambient lighting using a Raspberry Pi as a BLE relay.

The PC captures the screen, pulls out the dominant color, and sends it over UDP to the Pi. The Pi holds a persistent BLE connection to the strip and forwards color updates at up to 30 writes/sec.

```
PC (capture.py)  --UDP-->  Pi (relay.py)  --BLE-->  LED Strip
```

### Setup

1. `pip install -r requirements.txt`
2. Copy `.env.example` to `.env` and fill in your values

```bash
cp .env.example .env
```

### Usage

```bash
# on the Pi
python ambient/relay.py

# on the PC
python ambient/capture.py
```

### Configuration

All tuning options are at the top of `ambient/capture.py`.

| Setting      | Default | Description                           |
| ------------ | ------- | ------------------------------------- |
| `SMOOTHING`  | `0.30`  | EMA weight (0.2 = gaming, 0.5 = film)|
| `SATURATION` | `1.4`   | Color vibrancy                        |
| `GAMMA`      | `2.0`   | Gamma curve (lower = brighter darks)  |
| `BRIGHTNESS` | `100`   | LED brightness 0-100                  |
| `MIN_GLOW`   | `3`     | Below this, LEDs show warm amber      |
| `DELTA`      | `2.0`   | Min RGB change to trigger an update   |
| `MAX_FPS`    | `30`    | Max UDP packets per second            |

## License

MIT
