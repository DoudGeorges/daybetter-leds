"""Screen ambient lighting (PC).

Captures screen content, extracts a representative color, and sends
it to the Raspberry Pi relay over UDP.

    python ambient/capture.py
"""

from __future__ import annotations

import asyncio
import os
import signal
import socket
import struct
import sys
import time
import traceback
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import protocol  # noqa: E402 (triggers .env loading)

# Configuration

RELAY_HOST: str = os.environ.get("RELAY_HOST", "192.168.2.122")
RELAY_PORT: int = int(os.environ.get("RELAY_PORT", "9000"))
MONITOR: int = 0        # DXCam monitor index (0 = primary)
SMOOTHING: float = 0.30 # EMA weight on previous frame (0.2 = gaming, 0.5 = cinema)
SATURATION: float = 1.4 # Color vibrancy (1.0 = raw, 1.8 = oversaturated)
GAMMA: float = 2.0      # Gamma curve for LEDs (lower = brighter shadows)
BRIGHTNESS: int = 100   # LED brightness 0-100
MIN_GLOW: int = 3       # Below this, use warm amber instead of off
DELTA: float = 2.0      # RGB change needed to send an update
MAX_FPS: int = 30       # UDP packets per second

# Screen capture


def _create_camera(monitor: int = 0) -> tuple[object, str]:
    try:
        import dxcam
        cam = dxcam.create(output_idx=monitor, output_color="BGR")
        cam.start(target_fps=30, video_mode=True)
        return cam, "dxcam"
    except Exception:
        return None, "mss"


class _MSSFallback:
    __slots__ = ("sct", "mon")

    def __init__(self, monitor: int = 1) -> None:
        import mss as _mss
        self.sct = _mss.MSS()
        self.mon = self.sct.monitors[monitor]

    def grab(self) -> np.ndarray:
        shot = self.sct.grab(self.mon)
        return np.frombuffer(shot.raw, dtype=np.uint8).reshape(
            shot.height, shot.width, 4,
        )[:, :, :3]


# Color extraction

_LUMA_R, _LUMA_G, _LUMA_B = 0.2126, 0.7152, 0.0722  # BT.709
_edge_cache: dict[tuple[int, int], np.ndarray] = {}


def _build_edge_gradient(h: int, w: int) -> np.ndarray:
    """Cosine-falloff edge weight map: 3x at edges, 1x at center."""
    key = (h, w)
    if key in _edge_cache:
        return _edge_cache[key]
    y = np.arange(h, dtype=np.float32)
    x = np.arange(w, dtype=np.float32)
    dy = np.minimum(y, h - 1 - y) / (h * 0.5)
    dx = np.minimum(x, w - 1 - x) / (w * 0.5)
    dist = np.minimum(dy[:, None], dx[None, :])
    wm = 1.0 + 2.0 * (0.5 * (1.0 + np.cos(np.clip(dist, 0, 1) * np.pi)))
    _edge_cache[key] = wm
    return wm


_frame_buf: deque[np.ndarray] = deque(maxlen=3)


def _extract_color(frame: np.ndarray, ds: int = 16) -> np.ndarray:
    """Downsample, temporal average, edge + luminance weighting."""
    small = frame[::ds, ::ds].astype(np.float32)
    h, w = small.shape[:2]
    _frame_buf.append(small)
    if len(_frame_buf) > 1:
        small = np.mean(np.stack(_frame_buf), axis=0)
    b_ch, g_ch, r_ch = small[:, :, 0], small[:, :, 1], small[:, :, 2]
    luma = _LUMA_R * r_ch + _LUMA_G * g_ch + _LUMA_B * b_ch
    luma_w = (luma / 255.0) ** 0.5 + 0.08
    spatial = _build_edge_gradient(h, w)
    weight = luma_w * spatial
    total = weight.sum()
    if total < 1e-6:
        return np.zeros(3, dtype=np.float32)
    return np.array([
        (r_ch * weight).sum() / total,
        (g_ch * weight).sum() / total,
        (b_ch * weight).sum() / total,
    ], dtype=np.float32)


# Color pipeline

_DARK_GLOW = (8, 4, 1)  # warm amber ~2700K


def _build_gamma_lut(gamma: float) -> np.ndarray:
    """Hybrid gamma LUT with toe lift (0-20) for shadow preservation."""
    lut = np.zeros(256, dtype=np.uint8)
    toe = 20
    toe_g = gamma * 0.6
    for i in range(256):
        x = i / 255.0
        lut[i] = int(255.0 * x ** (toe_g if i <= toe else gamma) + 0.5)
    return lut


class _Pipeline:
    __slots__ = ("sat", "bri", "lut", "floor")

    def __init__(self) -> None:
        self.sat = SATURATION
        self.bri = BRIGHTNESS / 100.0
        self.lut = _build_gamma_lut(GAMMA)
        self.floor = MIN_GLOW

    def apply(self, rgb: np.ndarray) -> tuple[int, int, int]:
        r, g, b = float(rgb[0]), float(rgb[1]), float(rgb[2])
        if self.sat != 1.0:
            m = (r + g + b) / 3.0
            r = m + (r - m) * self.sat
            g = m + (g - m) * self.sat
            b = m + (b - m) * self.sat
        r *= self.bri
        g *= self.bri
        b *= self.bri
        ri = int(self.lut[max(0, min(255, int(r + 0.5)))])
        gi = int(self.lut[max(0, min(255, int(g + 0.5)))])
        bi = int(self.lut[max(0, min(255, int(b + 0.5)))])
        if self.floor > 0 and max(ri, gi, bi) < self.floor:
            ri, gi, bi = _DARK_GLOW
        return ri, gi, bi


# Adaptive EMA smoother


class _Smoother:
    """Snap on scene cuts, smooth on gradual shifts."""

    __slots__ = ("base", "snap", "state", "warm")

    def __init__(self) -> None:
        self.base = SMOOTHING
        self.snap = 50.0
        self.state = np.zeros(3, dtype=np.float64)
        self.warm = False

    def step(self, rgb: np.ndarray) -> np.ndarray:
        if not self.warm:
            self.state[:] = rgb
            self.warm = True
            return self.state.copy()
        d = float(np.linalg.norm(rgb - self.state))
        if d > self.snap:
            a = 0.03
        elif d > self.snap * 0.35:
            a = self.base * 0.25
        else:
            a = self.base
        self.state = self.state * a + rgb * (1.0 - a)
        return self.state.copy()


# UDP sender

_MAGIC = 0xDB
_CMD_COLOR = 0x01


class _UDPSender:
    __slots__ = ("addr", "sock")

    def __init__(self) -> None:
        self.addr = (RELAY_HOST, RELAY_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, r: int, g: int, b: int) -> None:
        try:
            self.sock.sendto(
                struct.pack("BBBBBBB", _MAGIC, _CMD_COLOR, r, g, b, 0, 0),
                self.addr,
            )
        except BlockingIOError:
            pass

    def close(self) -> None:
        self.sock.close()


# Main loop

_shutdown = asyncio.Event()


def _on_signal(*_: object) -> None:
    _shutdown.set()


async def main() -> None:
    signal.signal(signal.SIGINT, _on_signal)

    udp = _UDPSender()
    print(f"Relay -> {RELAY_HOST}:{RELAY_PORT}")

    cam, backend = _create_camera(MONITOR)
    mss_fb: _MSSFallback | None = None
    if backend == "mss":
        mss_fb = _MSSFallback(monitor=MONITOR + 1)
        print("  Capture: mss")
    else:
        print("  Capture: DXCam")

    pipe = _Pipeline()
    smooth = _Smoother()
    interval = 1.0 / MAX_FPS
    delta_sq = DELTA ** 2

    last_sent = (0, 0, 0)
    last_t = 0.0
    writes = 0
    frames = 0
    t_start = time.perf_counter()
    disp: deque[float] = deque(maxlen=120)

    print(f"  smooth={SMOOTHING}  sat={SATURATION}  gamma={GAMMA}  "
          f"bri={BRIGHTNESS}%  glow={MIN_GLOW}  fps={MAX_FPS}")
    print("  Ctrl+C to stop\n")

    try:
        while not _shutdown.is_set():
            t0 = time.perf_counter()

            if cam is not None:
                frame = cam.get_latest_frame()
                if frame is None:
                    await asyncio.sleep(0.005)
                    continue
            else:
                frame = mss_fb.grab()

            # Reject DXCam recovery glitches (near-white frames)
            if frame[::32, ::32, :3].mean() > 245:
                await asyncio.sleep(0.005)
                continue

            raw = _extract_color(frame)
            smoothed = smooth.step(raw)
            out = pipe.apply(smoothed)

            now = time.perf_counter()
            dr = out[0] - last_sent[0]
            dg = out[1] - last_sent[1]
            db = out[2] - last_sent[2]

            if (dr * dr + dg * dg + db * db) > delta_sq and (now - last_t) >= interval:
                udp.send(out[0], out[1], out[2])
                last_sent = out
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
                    f"  {1000 / avg_ms:4.0f}fps  {avg_ms:4.1f}ms"
                    f"  {wps:4.1f}w/s  w={writes}  {elapsed:.0f}s",
                    end="", flush=True,
                )

    except Exception as e:
        print(f"\nError: {e}")
        traceback.print_exc()
    finally:
        elapsed = time.perf_counter() - t_start
        wps = writes / max(elapsed, 0.001)
        print(f"\n\n  {writes} writes / {elapsed:.0f}s = {wps:.1f} writes/sec")
        udp.send(10, 5, 15)
        await asyncio.sleep(0.3)
        udp.close()
        if cam is not None:
            cam.stop()
        print("  Done.")


if __name__ == "__main__":
    asyncio.run(main())
