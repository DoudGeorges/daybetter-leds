"""Screen ambient lighting – PC side.

Three-thread architecture per ALGORITHM.md §1::

    CaptureThread → [FrameSlot] → ProcessThread → [ColorState] → TransmitThread → UDP

Pipeline::

    Screen → safe-area crop → 16×9 area downsample
    → sRGB linearise → OKLAB → discard near-black (Y<0.05)
    → K-Means (K=3, sat×luma weights) → dominant cluster
    → asymmetric luminance EMA + symmetric chromaticity EMA
    → RGBW white-channel matrixing → LED gamma LUT
    → temporal dithering → uint8 → UDP
"""

from __future__ import annotations

import logging
import math
import os
import signal
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

# Sibling import — the project is not installed as a package, so we prepend
# the repo root to sys.path.  Importing ``protocol`` also triggers its
# module-level ``_load_dotenv()`` which populates ``os.environ`` from ``.env``.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import protocol as _protocol  # noqa: E402

if TYPE_CHECKING:
    from typing import Any

    from types import FrameType

__all__: list[str] = []

# Suppress unused-import lint — ``_protocol`` is imported for its ``.env``
# loading side-effect; the names below are what the rest of this module uses.
_ = _protocol  # noqa: F841

log = logging.getLogger("capture")

RELAY_HOST: str = os.environ.get("RELAY_HOST", "192.168.2.122")
RELAY_PORT: int = int(os.environ.get("RELAY_PORT", "9000"))
MONITOR: int = 0

ATTACK_ALPHA: float = 0.60
DECAY_ALPHA: float = 0.08
CHROMA_ALPHA: float = 0.20

SATURATION: float = 1.25
GAMMA: float = 2.2
BRIGHTNESS: int = 100
MIN_GLOW: int = 6
DELTA: float = 0.008
MAX_FPS: int = 30
WHITE_CCT: int = 3000
SAFE_AREA_INTERVAL: int = 30
NEAR_BLACK_Y: float = 0.05

# OKLAB conversion matrices (Bjorn Ottosson, Jan 2021 precision).
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

_LUMA_R, _LUMA_G, _LUMA_B = 0.2126, 0.7152, 0.0722
_DARK_GLOW = np.array([10.0, 5.0, 2.0, 0.0], dtype=np.float64)
_UDP_MAGIC = 0xDB
_UDP_CMD_COLOR = 0x01


def srgb_to_linear(srgb: np.ndarray) -> np.ndarray:
    """sRGB [0,1] → linear RGB [0,1] (IEC 61966-2-1)."""
    return np.where(
        srgb <= 0.04045,
        srgb / 12.92,
        ((srgb + 0.055) / 1.055) ** 2.4,
    )


def linear_rgb_to_oklab(rgb: np.ndarray) -> np.ndarray:
    """Linear RGB (..., 3) → OKLAB (..., 3)."""
    lms = rgb @ _M1.T
    lms_g = np.sign(lms) * np.abs(lms) ** (1.0 / 3.0)
    return lms_g @ _M2.T


def oklab_to_linear_rgb(lab: np.ndarray) -> np.ndarray:
    """OKLAB (..., 3) → linear RGB (..., 3)."""
    lms_g = lab @ _M1_INV.T
    return (lms_g ** 3) @ _M2_INV.T


def _cct_to_linear_rgb(cct: int) -> np.ndarray:
    """Blackbody colour at *cct* Kelvin as normalised linear RGB."""
    t = cct / 100.0
    r = 255.0 if t <= 66 else max(329.698727446 * (t - 60) ** -0.1332047592, 0.0)
    if t <= 66:
        g = max(99.4708025861 * math.log(t) - 161.1195681661, 0.0)
    else:
        g = max(288.1221695283 * (t - 60) ** -0.0755148492, 0.0)
    b = 255.0 if t >= 66 else (
        0.0 if t <= 19 else max(138.5177312231 * math.log(t - 10) - 305.0447927307, 0.0))
    srgb = np.clip(np.array([r, g, b]) / 255.0, 0.0, 1.0)
    lin = srgb_to_linear(srgb)
    peak = float(lin.max())
    return lin / peak if peak > 1e-9 else lin


_WHITE_LED = _cct_to_linear_rgb(WHITE_CCT)


def _detect_safe_area(
    frame: np.ndarray, threshold: int = 8,
) -> tuple[int, int, int, int]:
    """Return (y0, y1, x0, x1) crop excluding letterbox/pillarbox bars."""
    h, w = frame.shape[:2]
    if h < 2 or w < 2:
        return (0, h, 0, w)

    col = frame[:, w // 2].max(axis=-1)
    row = frame[h // 2, :].max(axis=-1)

    y0 = 0
    while y0 < h and col[y0] < threshold:
        y0 += 1
    y1 = h
    while y1 > y0 + 1 and col[y1 - 1] < threshold:
        y1 -= 1
    x0 = 0
    while x0 < w and row[x0] < threshold:
        x0 += 1
    x1 = w
    while x1 > x0 + 1 and row[x1 - 1] < threshold:
        x1 -= 1

    if (y1 - y0) < 16 or (x1 - x0) < 16:
        return (0, h, 0, w)
    return (y0, y1, x0, x1)


def _area_downsample(
    crop: np.ndarray, tw: int = 16, th: int = 9,
) -> np.ndarray:
    """Downsample *crop* to a tw×th micro-matrix via area averaging."""
    h, w = crop.shape[:2]
    sh, sw = h // th, w // tw
    if sh < 1 or sw < 1:
        # Frame too small for proper area-average; clamp and pad.
        out = np.zeros((th, tw, 3), dtype=np.float64)
        ch, cw = min(h, th), min(w, tw)
        out[:ch, :cw] = crop[:ch, :cw].astype(np.float64) / 255.0
        return out
    trimmed = crop[: th * sh, : tw * sw]
    return trimmed.reshape(th, sh, tw, sw, 3).mean(axis=(1, 3)).astype(np.float64) / 255.0


def _kmeans_dominant(
    pixels: np.ndarray, weights: np.ndarray, k: int = 3, iters: int = 5,
) -> np.ndarray:
    """Weighted K-Means on OKLAB pixels; returns dominant cluster centroid."""
    n = len(pixels)
    if n == 0:
        return np.zeros(3, dtype=np.float64)
    if n <= k:
        total_w = weights.sum()
        if total_w < 1e-9:
            return pixels.mean(axis=0)
        return (pixels * weights[:, None]).sum(axis=0) / total_w

    # Greedy farthest-point initialisation.
    idx = [int(np.argmax(weights))]
    for _ in range(k - 1):
        dists = np.min(
            [np.linalg.norm(pixels - pixels[i], axis=1) for i in idx], axis=0,
        )
        idx.append(int(np.argmax(dists)))
    centroids = pixels[idx].copy()

    labels = np.zeros(n, dtype=np.intp)
    for _ in range(iters):
        dist_matrix = np.stack(
            [np.linalg.norm(pixels - c, axis=1) for c in centroids], axis=1,
        )
        labels = dist_matrix.argmin(axis=1)
        for j in range(k):
            mask = labels == j
            if not mask.any():
                continue
            w = weights[mask]
            total_w = w.sum()
            if total_w > 1e-9:
                centroids[j] = (pixels[mask] * w[:, None]).sum(axis=0) / total_w

    cluster_w = np.array([weights[labels == j].sum() for j in range(k)])
    return centroids[int(cluster_w.argmax())]


def extract_dominant(micro: np.ndarray) -> np.ndarray:
    """16×9 sRGB-float [0,1] micro-matrix → dominant OKLAB (L, a, b)."""
    flat = micro.reshape(-1, 3)
    linear = srgb_to_linear(flat)
    luma = linear[:, 0] * _LUMA_R + linear[:, 1] * _LUMA_G + linear[:, 2] * _LUMA_B

    keep = luma >= NEAR_BLACK_Y
    if keep.sum() < 3:
        return np.array([float(luma.mean()), 0.0, 0.0], dtype=np.float64)

    lin_kept = linear[keep]
    luma_kept = luma[keep]
    oklab = linear_rgb_to_oklab(lin_kept)

    chroma = np.sqrt(oklab[:, 1] ** 2 + oklab[:, 2] ** 2)
    weights = chroma * luma_kept + 1e-6
    return _kmeans_dominant(oklab, weights)


class _LuminanceSmoother:
    """Asymmetric EMA: fast attack (α=0.6), slow cinematic decay (α=0.08)."""

    __slots__ = ("_val", "_warm", "_attack", "_decay")

    def __init__(self, attack: float = ATTACK_ALPHA, decay: float = DECAY_ALPHA) -> None:
        self._val = 0.0
        self._warm = False
        self._attack = attack
        self._decay = decay

    def step(self, target: float) -> float:
        if not self._warm:
            self._val = target
            self._warm = True
            return self._val
        alpha = self._attack if target > self._val else self._decay
        self._val = alpha * target + (1.0 - alpha) * self._val
        return self._val


class _ChromaSmoother:
    """Symmetric EMA on OKLAB (a, b) chromaticity channels."""

    __slots__ = ("_ab", "_warm", "_alpha")

    def __init__(self, alpha: float = CHROMA_ALPHA) -> None:
        self._ab = np.zeros(2, dtype=np.float64)
        self._warm = False
        self._alpha = alpha

    def step(self, target_ab: np.ndarray) -> np.ndarray:
        if not self._warm:
            self._ab[:] = target_ab
            self._warm = True
            return self._ab.copy()
        self._ab = self._alpha * target_ab + (1.0 - self._alpha) * self._ab
        return self._ab.copy()


class _Smoother:
    """Split luminance/chromaticity smoothing with hysteresis deadband."""

    __slots__ = ("_lum", "_chroma", "_state", "_warm", "_in_deadband")

    _DB_ENTER = 0.008
    _DB_EXIT = 0.015

    def __init__(self) -> None:
        self._lum = _LuminanceSmoother()
        self._chroma = _ChromaSmoother()
        self._state = np.zeros(3, dtype=np.float64)
        self._warm = False
        self._in_deadband = False

    def step(self, lab: np.ndarray) -> np.ndarray:
        if not self._warm:
            self._state[:] = lab
            self._lum.step(float(lab[0]))
            self._chroma.step(lab[1:3])
            self._warm = True
            return self._state.copy()

        d_e = float(np.linalg.norm(lab - self._state))

        if self._in_deadband:
            if d_e < self._DB_EXIT:
                return self._state.copy()
            self._in_deadband = False
        elif d_e < self._DB_ENTER:
            self._in_deadband = True
            return self._state.copy()

        smoothed_l = self._lum.step(float(lab[0]))
        smoothed_ab = self._chroma.step(lab[1:3])
        self._state = np.array([smoothed_l, smoothed_ab[0], smoothed_ab[1]], dtype=np.float64)
        return self._state.copy()


def _rgbw_matrix(
    linear: np.ndarray, white: np.ndarray = _WHITE_LED,
) -> tuple[float, float, float, float]:
    """Decompose linear RGB into residual RGB + white channel via CTM."""
    r, g, b = float(linear[0]), float(linear[1]), float(linear[2])
    ratios: list[float] = []
    for channel, white_channel in zip((r, g, b), white):
        if white_channel > 0.001:
            ratios.append(channel / float(white_channel))
    w = max(0.0, min(min(ratios) if ratios else 0.0, 1.0))
    return (
        r - w * float(white[0]),
        g - w * float(white[1]),
        b - w * float(white[2]),
        w,
    )


def _build_led_lut(gamma: float) -> np.ndarray:
    """1024-entry LUT mapping linear [0,1] → LED uint8."""
    t = np.arange(1024, dtype=np.float64) / 1023.0
    return np.clip(255.0 * t ** (1.0 / gamma) + 0.5, 0, 255).astype(np.uint8)


def _lin_to_led(v: float, lut: np.ndarray) -> int:
    """Map a single linear [0,1] value through the LED gamma LUT."""
    return int(lut[min(max(int(v * 1023.0 + 0.5), 0), 1023)])


class _TemporalDither:
    """1-D error diffusion across 4 RGBW channels per frame."""

    __slots__ = ("_err",)

    def __init__(self) -> None:
        self._err = np.zeros(4, dtype=np.float64)

    def quantize(self, values: np.ndarray) -> np.ndarray:
        adjusted = values + self._err
        quantized = np.clip(np.floor(adjusted + 0.5), 0, 255).astype(np.uint8)
        self._err = adjusted - quantized.astype(np.float64)
        return quantized


class _Pipeline:
    """OKLAB → final (R, G, B, W) uint8 for LED output."""

    __slots__ = ("_sat", "_bri", "_lut", "_floor", "_dither")

    def __init__(self) -> None:
        self._sat = SATURATION
        self._bri = BRIGHTNESS / 100.0
        self._lut = _build_led_lut(GAMMA)
        self._floor = MIN_GLOW
        self._dither = _TemporalDither()

    def apply(self, lab: np.ndarray) -> tuple[int, int, int, int]:
        scaled = np.array(
            [float(lab[0]) * self._bri, float(lab[1]) * self._sat, float(lab[2]) * self._sat],
            dtype=np.float64,
        )
        linear = np.clip(oklab_to_linear_rgb(scaled), 0.0, 1.0)

        r, g, b, w = _rgbw_matrix(linear)
        rf = _lin_to_led(max(r, 0.0), self._lut)
        gf = _lin_to_led(max(g, 0.0), self._lut)
        bf = _lin_to_led(max(b, 0.0), self._lut)
        wf = _lin_to_led(max(w, 0.0), self._lut)

        peak = max(rf, gf, bf)
        vals = np.array([rf, gf, bf, wf], dtype=np.float64)
        if self._floor > 0 and peak < self._floor:
            t = peak / self._floor
            vals = _DARK_GLOW * (1.0 - t) + vals * t

        q = self._dither.quantize(vals)
        return int(q[0]), int(q[1]), int(q[2]), int(q[3])


def _create_camera(monitor: int = 0) -> tuple[Any, str]:
    """Try DXCam (GPU-accelerated), fall back to mss."""
    try:
        import dxcam

        cam = dxcam.create(output_idx=monitor, output_color="BGR")
        cam.start(target_fps=60, video_mode=True)
        return cam, "dxcam"
    except Exception as exc:
        log.debug("DXCam unavailable, falling back to mss: %s", exc)
        return None, "mss"


class _MSSFallback:
    """Cross-platform GDI fallback when DXCam is unavailable."""

    __slots__ = ("_sct", "_mon")

    def __init__(self, monitor: int = 1) -> None:
        import mss as _mss

        self._sct = _mss.MSS()
        # MSS uses 1-based monitor indices (0 = virtual all-monitors).
        self._mon = self._sct.monitors[monitor]

    def grab(self) -> np.ndarray | None:
        shot = self._sct.grab(self._mon)
        if shot is None:
            return None
        return np.frombuffer(shot.raw, dtype=np.uint8).reshape(
            shot.height, shot.width, 4,
        )[:, :, :3]


class _FrameSlot:
    """Thread-safe single-frame buffer: capture overwrites, process reads latest."""

    __slots__ = ("_frame", "_lock", "_new")

    def __init__(self) -> None:
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._new = threading.Event()

    def put(self, frame: np.ndarray) -> None:
        with self._lock:
            self._frame = frame
            self._new.set()

    def get(self, timeout: float = 0.5) -> np.ndarray | None:
        if not self._new.wait(timeout):
            return None
        with self._lock:
            frame = self._frame
            self._new.clear()
        return frame


class _ColorState:
    """Thread-safe latest RGBW + brightness output."""

    __slots__ = ("_r", "_g", "_b", "_w", "_bri", "_lab", "_seq", "_lock")

    def __init__(self) -> None:
        self._r = self._g = self._b = self._w = 0
        self._bri = BRIGHTNESS
        self._lab = np.zeros(3, dtype=np.float64)
        self._seq = 0
        self._lock = threading.Lock()

    def update(self, r: int, g: int, b: int, w: int, lab: np.ndarray) -> None:
        with self._lock:
            self._r, self._g, self._b, self._w = r, g, b, w
            self._lab = lab.copy()
            self._seq += 1

    def read(self) -> tuple[int, int, int, int, int, np.ndarray, int]:
        with self._lock:
            return (
                self._r, self._g, self._b, self._w,
                self._bri, self._lab.copy(), self._seq,
            )


class _CaptureThread(threading.Thread):
    """Grabs frames from the display at native refresh rate."""

    def __init__(self, slot: _FrameSlot, stop: threading.Event) -> None:
        super().__init__(daemon=True, name="capture")
        self._slot = slot
        self._stop = stop
        self._cam: object | None = None
        self._mss: _MSSFallback | None = None
        self.backend = "?"

    def run(self) -> None:
        self._cam, self.backend = _create_camera(MONITOR)
        if self.backend == "mss":
            self._mss = _MSSFallback(monitor=MONITOR + 1)
        log.info("  Capture: %s", self.backend)

        while not self._stop.is_set():
            try:
                if self._cam is not None:
                    frame = self._cam.get_latest_frame()  # type: ignore[attr-defined]
                else:
                    assert self._mss is not None
                    frame = self._mss.grab()
            except Exception as exc:
                log.warning("Frame grab failed: %s", exc)
                time.sleep(0.05)
                continue
            if frame is None:
                time.sleep(0.005)
                continue
            self._slot.put(frame)
            time.sleep(0.002)

        if self._cam is not None:
            self._cam.stop()  # type: ignore[attr-defined]


class _ProcessThread(threading.Thread):
    """Runs the full colour science pipeline on each new frame."""

    def __init__(
        self, slot: _FrameSlot, state: _ColorState, stop: threading.Event,
    ) -> None:
        super().__init__(daemon=True, name="process")
        self._slot = slot
        self._state = state
        self._stop = stop

    def run(self) -> None:
        smoother = _Smoother()
        pipe = _Pipeline()
        safe_area: tuple[int, int, int, int] | None = None
        frame_n = 0

        while not self._stop.is_set():
            frame = self._slot.get(timeout=0.5)
            if frame is None:
                continue

            if frame_n % SAFE_AREA_INTERVAL == 0:
                safe_area = _detect_safe_area(frame)
            frame_n += 1

            if safe_area is not None:
                y0, y1, x0, x1 = safe_area
                crop = frame[y0:y1, x0:x1]
            else:
                crop = frame

            micro_bgr = _area_downsample(crop)
            micro_rgb = micro_bgr[:, :, ::-1].copy()

            lab = extract_dominant(micro_rgb)
            smoothed = smoother.step(lab)

            r, g, b, w = pipe.apply(smoothed)
            self._state.update(r, g, b, w, smoothed)


class _TransmitThread(threading.Thread):
    """Sends UDP at a strict cadence, independent of capture/process rate."""

    def __init__(self, state: _ColorState, stop: threading.Event) -> None:
        super().__init__(daemon=True, name="transmit")
        self._state = state
        self._stop = stop

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        addr = (RELAY_HOST, RELAY_PORT)
        interval = 1.0 / MAX_FPS
        delta_sq = DELTA ** 2
        last_lab = np.zeros(3, dtype=np.float64)
        last_seq = -1
        writes = 0
        t_start = time.perf_counter()

        log.info("  Transmit: %d Hz → %s:%d", MAX_FPS, RELAY_HOST, RELAY_PORT)

        try:
            while not self._stop.is_set():
                time.sleep(interval)
                r, g, b, w, bri, lab, seq = self._state.read()
                if seq == last_seq:
                    continue

                diff = lab - last_lab
                d_e_sq = float(diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2)
                if d_e_sq < delta_sq:
                    continue

                try:
                    sock.sendto(
                        struct.pack("BBBBBBB", _UDP_MAGIC, _UDP_CMD_COLOR, r, g, b, w, bri),
                        addr,
                    )
                except OSError:
                    pass

                last_lab[:] = lab
                last_seq = seq
                writes += 1

                if writes % 60 == 0:
                    elapsed = time.perf_counter() - t_start
                    wps = writes / max(elapsed, 0.001)
                    log.info(
                        "(%3d,%3d,%3d,%3d)  %.1fw/s  w=%d  %.0fs",
                        r, g, b, w, wps, writes, elapsed,
                    )
        finally:
            sock.close()
            elapsed = time.perf_counter() - t_start
            log.info(
                "%d writes / %.0fs = %.1f w/s",
                writes, elapsed, writes / max(elapsed, 0.001),
            )


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    stop = threading.Event()

    def on_signal(_signum: int, _frame: FrameType | None) -> None:
        stop.set()

    signal.signal(signal.SIGINT, on_signal)

    log.info("Ambient LED Engine")
    log.info(
        "  attack=%.2f  decay=%.2f  chroma=%.2f  sat=%.2f  gamma=%.1f",
        ATTACK_ALPHA, DECAY_ALPHA, CHROMA_ALPHA, SATURATION, GAMMA,
    )
    log.info(
        "  bri=%d%%  glow=%d  fps=%d  white=%dK",
        BRIGHTNESS, MIN_GLOW, MAX_FPS, WHITE_CCT,
    )

    slot = _FrameSlot()
    state = _ColorState()

    threads = [
        _CaptureThread(slot, stop),
        _ProcessThread(slot, state, stop),
        _TransmitThread(state, stop),
    ]
    for t in threads:
        t.start()

    log.info("  Ctrl+C to stop")

    try:
        while not stop.is_set():
            stop.wait(timeout=1.0)
    except KeyboardInterrupt:
        stop.set()

    log.info("Shutting down ...")
    for t in threads:
        t.join(timeout=3)
    log.info("Done.")


if __name__ == "__main__":
    main()
