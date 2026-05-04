# daybetter-leds

Reverse-engineered BLE protocol for DayBetter LED strips. Built from decompiling the Android APK. See [PROTOCOL.md](PROTOCOL.md) for the full protocol breakdown.

## Ambient

Real-time screen ambient lighting using a Raspberry Pi as a BLE relay.

The PC captures the screen, extracts the dominant color in **OKLAB perceptual color space**, and sends it over UDP to the Pi. The Pi holds a persistent BLE connection to the strip and forwards color updates at up to 40 writes/sec.

OKLAB ensures that color averaging, smoothing, and saturation adjustments produce vibrant, perceptually accurate results without the muddy mid-tones of RGB-space processing.

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

| Setting      | Default | Description                                   |
| ------------ | ------- | --------------------------------------------- |
| `SMOOTHING`  | `0.50`  | EMA retention (0.2 = gaming, 0.5 = film)      |
| `SATURATION` | `1.4`   | OKLAB chroma boost (hue-preserving)            |
| `GAMMA`      | `2.2`   | LED gamma (raise to 2.5 for dimmer mid-tones)  |
| `BRIGHTNESS` | `100`   | LED brightness 0-100                           |
| `MIN_GLOW`   | `6`     | Below this, LEDs blend toward warm amber       |
| `DELTA`      | `0.015` | Min OKLAB dE to trigger an update              |
| `MAX_FPS`    | `20`    | Max UDP packets per second                     |

### Color Pipeline

```
Screen -> downsample -> sRGB -> linear RGB -> OKLAB
-> luminance-weighted average -> temporal blend (6 frames)
-> adaptive EMA (3-tier + deadband) -> chroma boost
-> brightness scale -> linear RGB -> LED gamma LUT -> uint8
```

## License

MIT
