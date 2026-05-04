# daybetter-leds

Reverse-engineered BLE protocol for DayBetter LED strips, built from decompiling the Android APK. See [PROTOCOL.md](PROTOCOL.md) for the full protocol breakdown.

## Ambient

Real-time screen ambient lighting using a Raspberry Pi as a BLE relay.

Three decoupled threads ensure smooth, jitter-free output: **Capture** grabs frames at the display's native refresh rate, **Process** runs the full color science pipeline, and **Transmit** pushes RGBW data over UDP at a strict 30 Hz cadence.

The engine extracts the dominant scene chromaticity via **K-Means clustering (K=3) in Oklab perceptual color space**, uses asymmetric luminance smoothing for cinematic fades, and performs **RGBW white-channel matrixing** calibrated to the strip's white LED color temperature.

```
PC (capture.py)  ──UDP──>  Pi (relay.py)  ──BLE──>  LED Strip
  [3 threads]                [async BLE]
```

### Setup

1. Create and activate a virtual environment:

```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# Linux / macOS
source .venv/bin/activate
```

2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Copy `.env.example` to `.env` and fill in your values:

```bash
cp .env.example .env
```

### Usage

```bash
# Pi
python ambient/relay.py

# PC
python ambient/capture.py
```

### Configuration

All tuning options are at the top of `ambient/capture.py`.

| Setting              | Default | Description                                |
| -------------------- | ------- | ------------------------------------------ |
| `ATTACK_ALPHA`       | `0.60`  | Luminance fade-in speed (fast response)    |
| `DECAY_ALPHA`        | `0.08`  | Luminance fade-out speed (cinematic decay) |
| `CHROMA_ALPHA`       | `0.20`  | Chromaticity glide (hue transition speed)  |
| `SATURATION`         | `1.25`  | Oklab chroma boost (hue-preserving)        |
| `GAMMA`              | `2.2`   | LED gamma curve                            |
| `BRIGHTNESS`         | `100`   | LED brightness (0–100)                     |
| `MIN_GLOW`           | `6`     | Dark-scene warm amber floor                |
| `DELTA`              | `0.008` | Minimum Oklab ΔE to trigger an update      |
| `MAX_FPS`            | `30`    | Transmit thread cadence (Hz)               |
| `WHITE_CCT`          | `3000`  | White LED color temperature (Kelvin)       |
| `SAFE_AREA_INTERVAL` | `30`    | Re-detect letterbox bars every N frames    |
| `NEAR_BLACK_Y`       | `0.05`  | Luminance threshold for pixel discard      |

### Color Pipeline

```
Screen → safe-area crop → 16×9 area downsample
→ sRGB → linearize → Oklab → discard near-black (Y < 0.05)
→ K-Means (K=3, saturation × luminance weights) → dominant cluster
→ asymmetric luminance EMA + symmetric chromaticity EMA
→ RGBW white-channel matrixing → LED gamma LUT
→ temporal dithering → uint8 → UDP
```

## License

MIT
