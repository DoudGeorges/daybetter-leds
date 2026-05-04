"""Screen ambient lighting (PC side).

Captures screen content, extracts a representative colour in OKLAB
perceptual colour space, and streams it to the Raspberry Pi relay via UDP.

Pipeline::

    Screen -> downsample -> sRGB -> linear RGB -> OKLAB
    -> luminance-weighted average -> temporal blend (3 frames)
    -> 4-tier adaptive EMA -> chroma boost -> brightness scale
    -> linear RGB -> LED gamma LUT -> uint8
"""

from __future__ import annotations

import asyncio
import logging
import os
import signal
import socket
import struct
import sys
import time
from collections import deque
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import protocol as _protocol  # noqa: E402, F841  (.env side-effect)

if TYPE_CHECKING:
    from types import FrameType

__all__: list[str] = []

log = logging.getLogger("capture")

RELAY_HOST: str = os.environ.get("RELAY_HOST", "192.168.2.122")
RELAY_PORT: int = int(os.environ.get("RELAY_PORT", "9000"))
MONITOR: int = 0
SMOOTHING: float = 0.30   # EMA retention (lower = snappier, higher = smoother)
SATURATION: float = 1.25  # OKLAB chroma boost (1.0 = raw; kept moderate to avoid gamut clipping)
GAMMA: float = 2.4        # LED gamma (applied once, directly on linear RGB)
BRIGHTNESS: int = 100     # LED brightness 0-100
MIN_GLOW: int = 6         # below this, blend toward warm amber
DELTA: float = 0.008      # min OKLAB dE to trigger a UDP send
MAX_FPS: int = 30         # max UDP packets per second


# OKLAB colour science
# Reference: https://bottosson.github.io/posts/oklab/
# All conversions are vectorised numpy, operating on (..., 3) arrays.
# Matrices from Bjorn Ottosson (updated Jan 2021 precision).

# fmt: off
_M1 = np.array([
    [0.4122214708, 0.5363325363, 0.0514459929],
    [0.2119034982, 0.6806995451, 0.1073969566],
    [0.0883024619, 0.2817188376, 0.6299787005],
], dtype=np.float64)

_M2 = np.array([
    [0.2104542553, 0.7936177850, -0.0040720468],
    [1.9779984951, -2.4285922050,  0.4505937099],
    [0.0259040371, 0.7827717662, -0.8086757660],
], dtype=np.float64)

_M1_INV = np.array([
    [1.0,  0.3963377774,  0.2158037573],
    [1.0, -0.1055613458, -0.0638541728],
    [1.0, -0.0894841775, -1.2914855480],
], dtype=np.float64)

_M2_INV = np.array([
    [ 4.0767416621, -3.3077115913,  0.2309699292],
    [-1.2684380046,  2.6097574011, -0.3413193965],
    [-0.0041960863, -0.7034186147,  1.7076147010],
], dtype=np.float64)
# fmt: on


def srgb_to_linear(srgb: np.ndarray) -> np.ndarray:
    """sRGB [0,1] -> linear RGB [0,1] (IEC 61966-2-1)."""
    return np.where(
        srgb <= 0.04045,
        srgb / 12.92,
        ((srgb + 0.055) / 1.055) ** 2.4,
    )


def linear_rgb_to_oklab(rgb: np.ndarray) -> np.ndarray:
    """Linear RGB (..., 3) -> OKLAB (..., 3)."""
    lms = rgb @ _M1.T
    lms_g = np.sign(lms) * np.abs(lms) ** (1.0 / 3.0)
    return lms_g @ _M2.T


def oklab_to_linear_rgb(lab: np.ndarray) -> np.ndarray:
    """OKLAB (..., 3) -> linear RGB (..., 3)."""
    lms_g = lab @ _M1_INV.T
    return (lms_g ** 3) @ _M2_INV.T


def _create_camera(monitor: int = 0) -> tuple[object | None, str]:
    """Try DXCam (GPU-accelerated), fall back to mss."""
    try:
        import dxcam
        cam = dxcam.create(output_idx=monitor, output_color="BGR")
        cam.start(target_fps=MAX_FPS, video_mode=True)
        return cam, "dxcam"
    except Exception:
        return None, "mss"


class _MSSFallback:
    """Cross-platform GDI fallback when DXCam is unavailable."""

    __slots__ = ("_sct", "_mon")

    def __init__(self, monitor: int = 1) -> None:
        import mss as _mss
        self._sct = _mss.MSS()
        self._mon = self._sct.monitors[monitor]

    def grab(self) -> np.ndarray | None:
        shot = self._sct.grab(self._mon)
        if shot is None:
            return None
        return np.frombuffer(shot.raw, dtype=np.uint8).reshape(
            shot.height, shot.width, 4,
        )[:, :, :3]


# BT.709 luma coefficients for luminance weighting in linear space.
_LUMA_R, _LUMA_G, _LUMA_B = 0.2126, 0.7152, 0.0722


def _build_edge_gradient(h: int, w: int) -> np.ndarray:
    """Cosine-falloff weight map: 3x at edges, 1x at centre."""
    y = np.arange(h, dtype=np.float32)
    x = np.arange(w, dtype=np.float32)
    dy = np.minimum(y, h - 1 - y) / max(h * 0.5, 1)
    dx = np.minimum(x, w - 1 - x) / max(w * 0.5, 1)
    dist = np.minimum(dy[:, None], dx[None, :])
    return 1.0 + 2.0 * (0.5 * (1.0 + np.cos(np.clip(dist, 0, 1) * np.pi)))


class _ColorExtractor:
    """Stateful per-frame colour extraction in OKLAB space.

    Encapsulates the temporal frame buffer, anomaly rejection counter,
    and edge-weight cache so the module carries no mutable global state.
    """

    __slots__ = ("_frame_buf", "_reject_count", "_edge_cache")

    _MAX_CONSECUTIVE_REJECTS = 10

    def __init__(self) -> None:
        self._frame_buf: deque[np.ndarray] = deque(maxlen=3)
        self._reject_count = 0
        self._edge_cache: dict[tuple[int, int], np.ndarray] = {}

    def _get_edge_weights(self, h: int, w: int) -> np.ndarray:
        key = (h, w)
        if key not in self._edge_cache:
            self._edge_cache[key] = _build_edge_gradient(h, w)
        return self._edge_cache[key]

    def extract(self, frame: np.ndarray, ds: int = 16) -> np.ndarray | None:
        """Return the dominant OKLAB (L, a, b) vector, or *None* on reject."""
        small = frame[::ds, ::ds].astype(np.float64) / 255.0
        h, w = small.shape[:2]

        # Reject anomalous frames (pure white/black from capture glitches).
        mean_brightness = small.mean() * 255.0
        if mean_brightness > 250.0 or mean_brightness < 2.0:
            self._reject_count += 1
            if self._reject_count < self._MAX_CONSECUTIVE_REJECTS:
                return None
            # Accept after too many consecutive rejects (actual content).
        else:
            self._reject_count = 0

        rgb_srgb = small[:, :, ::-1].copy()  # BGR -> RGB
        rgb_linear = srgb_to_linear(rgb_srgb)
        oklab = linear_rgb_to_oklab(rgb_linear)

        luma = (
            rgb_linear[:, :, 0] * _LUMA_R
            + rgb_linear[:, :, 1] * _LUMA_G
            + rgb_linear[:, :, 2] * _LUMA_B
        )
        luma_w = luma + 0.02  # small floor so black pixels aren't zero-weight

        spatial = self._get_edge_weights(h, w)
        weight = luma_w * spatial

        self._frame_buf.append(oklab)
        if len(self._frame_buf) > 1:
            oklab = np.mean(np.stack(self._frame_buf), axis=0)

        total = weight.sum()
        if total < 1e-9:
            return np.zeros(3, dtype=np.float64)

        ch_L = (oklab[:, :, 0] * weight).sum() / total
        ch_a = (oklab[:, :, 1] * weight).sum() / total
        ch_b = (oklab[:, :, 2] * weight).sum() / total
        return np.array([ch_L, ch_a, ch_b], dtype=np.float64)


class _Smoother:
    """Four-tier adaptive EMA with hysteresis deadband in OKLAB.

    Uses separate enter/exit thresholds to prevent oscillation at the
    deadband boundary (the primary source of visible flicker).

    Tiers (by OKLAB dE, 0-1 scale where black <-> white ~ 1.0):
      >0.35  scene cut  -> alpha=0.04  (96% new, near-instant snap)
      >0.18  large      -> alpha=0.12  (88% new, fast settle)
      >0.08  medium     -> alpha=0.20  (80% new, smooth transition)
      >0.015 small      -> alpha=0.30  (70% new, gentle drift)
      <0.008 deadband   -> hold        (suppress sub-perceptual noise)
    """

    __slots__ = ("_base_alpha", "_state", "_warm", "_in_deadband")

    _SCENE_CUT = 0.35
    _LARGE = 0.18
    _MEDIUM = 0.08
    _DEADBAND_ENTER = 0.008   # must drop below this to enter deadband
    _DEADBAND_EXIT = 0.015    # must exceed this to leave deadband

    _ALPHA_SNAP = 0.04
    _ALPHA_LARGE = 0.12
    _ALPHA_MEDIUM = 0.20

    def __init__(self, base_alpha: float = SMOOTHING) -> None:
        self._base_alpha = base_alpha
        self._state = np.zeros(3, dtype=np.float64)
        self._warm = False
        self._in_deadband = False

    def step(self, lab: np.ndarray) -> np.ndarray:
        """Return the smoothed OKLAB vector for this frame."""
        if not self._warm:
            self._state[:] = lab
            self._warm = True
            return self._state.copy()

        d_e = float(np.linalg.norm(lab - self._state))

        # Hysteresis: once in deadband, require a larger dE to exit.
        if self._in_deadband:
            if d_e < self._DEADBAND_EXIT:
                return self._state.copy()
            self._in_deadband = False
        elif d_e < self._DEADBAND_ENTER:
            self._in_deadband = True
            return self._state.copy()

        if d_e > self._SCENE_CUT:
            alpha = self._ALPHA_SNAP
        elif d_e > self._LARGE:
            alpha = self._ALPHA_LARGE
        elif d_e > self._MEDIUM:
            alpha = self._ALPHA_MEDIUM
        else:
            alpha = self._base_alpha

        self._state = self._state * alpha + lab * (1.0 - alpha)
        return self._state.copy()


# Warm amber fallback for dark scenes (~2700 K).
_DARK_GLOW = np.array([10.0, 5.0, 2.0], dtype=np.float64)


def _build_led_lut(gamma: float) -> np.ndarray:
    """Build a 1024-entry LUT mapping linear [0,1] -> LED uint8.

    A single gamma curve converts linear light values directly to
    LED PWM duty bytes.  No sRGB round-trip — this is the *only*
    non-linearity in the output path.
    """
    t = np.arange(1024, dtype=np.float64) / 1023.0
    return np.clip(255.0 * t ** (1.0 / gamma) + 0.5, 0, 255).astype(np.uint8)


def _linear_to_led(value: float, lut: np.ndarray) -> int:
    """Map a single linear [0,1] channel through the LED LUT."""
    idx = min(max(int(value * 1023.0 + 0.5), 0), 1023)
    return int(lut[idx])


class _Pipeline:
    """OKLAB -> final ``(R, G, B)`` uint8 for LED output.

    Steps:
      1. Chroma boost (scale *a*, *b* -- hue-preserving)
      2. Brightness scaling (scale *L*)
      3. OKLAB -> linear RGB (no sRGB detour)
      4. Single LED gamma LUT (linear -> PWM byte)
      5. Dark-scene floor: proportional blend toward warm amber
    """

    __slots__ = ("_sat", "_bri", "_lut", "_floor")

    def __init__(
        self,
        saturation: float = SATURATION,
        brightness: int = BRIGHTNESS,
        gamma: float = GAMMA,
        min_glow: int = MIN_GLOW,
    ) -> None:
        self._sat = saturation
        self._bri = brightness / 100.0
        self._lut = _build_led_lut(gamma)
        self._floor = min_glow

    def apply(self, lab: np.ndarray) -> tuple[int, int, int]:
        out_l = float(lab[0]) * self._bri
        out_a = float(lab[1]) * self._sat
        out_b = float(lab[2]) * self._sat

        # Direct linear RGB — no sRGB encoding, no double gamma.
        linear = oklab_to_linear_rgb(
            np.array([out_l, out_a, out_b], dtype=np.float64),
        )
        linear = np.clip(linear, 0.0, 1.0)

        ri = _linear_to_led(float(linear[0]), self._lut)
        gi = _linear_to_led(float(linear[1]), self._lut)
        bi = _linear_to_led(float(linear[2]), self._lut)

        peak = max(ri, gi, bi)
        if self._floor > 0 and peak < self._floor:
            t = peak / self._floor
            ri = int(_DARK_GLOW[0] * (1.0 - t) + ri * t + 0.5)
            gi = int(_DARK_GLOW[1] * (1.0 - t) + gi * t + 0.5)
            bi = int(_DARK_GLOW[2] * (1.0 - t) + bi * t + 0.5)

        return (ri, gi, bi)


_MAGIC = 0xDB
_CMD_COLOR = 0x01


class _UDPSender:
    """Non-blocking UDP socket for streaming colour packets to the relay."""

    __slots__ = ("_addr", "_sock")

    def __init__(self, host: str = RELAY_HOST, port: int = RELAY_PORT) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

    def send(self, r: int, g: int, b: int) -> None:
        try:
            self._sock.sendto(
                struct.pack("BBBBBBB", _MAGIC, _CMD_COLOR, r, g, b, 0, 0),
                self._addr,
            )
        except OSError:
            pass

    def close(self) -> None:
        self._sock.close()


async def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    shutdown = asyncio.Event()

    def on_signal(_signum: int, _frame: FrameType | None) -> None:
        shutdown.set()

    signal.signal(signal.SIGINT, on_signal)

    udp = _UDPSender()
    log.info("Relay -> %s:%d", RELAY_HOST, RELAY_PORT)

    cam, backend = _create_camera(MONITOR)
    mss_fb: _MSSFallback | None = None
    if backend == "mss":
        mss_fb = _MSSFallback(monitor=MONITOR + 1)
        log.info("  Capture: mss")
    else:
        log.info("  Capture: DXCam")

    extractor = _ColorExtractor()
    pipe = _Pipeline()
    smooth = _Smoother()

    interval = 1.0 / MAX_FPS
    delta_sq = DELTA ** 2

    last_lab = np.zeros(3, dtype=np.float64)
    last_t = 0.0
    writes = 0
    frames = 0
    t_start = time.perf_counter()
    disp: deque[float] = deque(maxlen=120)

    log.info(
        "  smooth=%.2f  sat=%.1f  gamma=%.1f  bri=%d%%  glow=%d  fps=%d",
        SMOOTHING, SATURATION, GAMMA, BRIGHTNESS, MIN_GLOW, MAX_FPS,
    )
    log.info("  Pipeline: sRGB -> OKLAB -> smooth -> boost -> LED gamma")
    log.info("  Ctrl+C to stop")

    out = (0, 0, 0)

    try:
        while not shutdown.is_set():
            t0 = time.perf_counter()

            if cam is not None:
                frame = cam.get_latest_frame()
            else:
                assert mss_fb is not None
                frame = mss_fb.grab()

            if frame is None:
                await asyncio.sleep(0.005)
                continue

            raw_lab = extractor.extract(frame)
            if raw_lab is None:
                await asyncio.sleep(0.005)
                continue

            smoothed_lab = smooth.step(raw_lab)
            out = pipe.apply(smoothed_lab)

            now = time.perf_counter()
            diff = smoothed_lab - last_lab
            d_e_sq = float(diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2)

            if d_e_sq > delta_sq and (now - last_t) >= interval:
                udp.send(*out)
                last_lab[:] = smoothed_lab
                last_t = now
                writes += 1

            frames += 1
            await asyncio.sleep(0.003)

            dt = time.perf_counter() - t0
            disp.append(dt)
            if frames % 60 == 0:
                avg_ms = sum(disp) / len(disp) * 1000
                elapsed = now - t_start
                wps = writes / max(elapsed, 0.001)
                print(
                    f"\r  ({out[0]:3d},{out[1]:3d},{out[2]:3d})"
                    f"  {1000 / max(avg_ms, 0.1):4.0f}fps  {avg_ms:4.1f}ms"
                    f"  {wps:4.1f}w/s  w={writes}  {elapsed:.0f}s",
                    end="", flush=True,
                )

    except Exception:
        log.exception("Fatal error in capture loop")
    finally:
        elapsed = time.perf_counter() - t_start
        wps = writes / max(elapsed, 0.001)
        log.info("%d writes / %.0fs = %.1f writes/sec", writes, elapsed, wps)
        udp.close()
        if cam is not None:
            cam.stop()
        log.info("Done.")


if __name__ == "__main__":
    asyncio.run(main())
