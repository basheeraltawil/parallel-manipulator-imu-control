"""Frame composition and GIF/MP4 export for the scenario videos."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
MEDIA = ROOT / "docs" / "media"


def fig_to_array(fig) -> np.ndarray:
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()


def hstack(*imgs, height=None) -> np.ndarray:
    height = height or max(i.shape[0] for i in imgs)
    out = []
    for im in imgs:
        if im.shape[0] != height:
            w = int(round(im.shape[1] * height / im.shape[0]))
            im = np.asarray(Image.fromarray(im).resize((w, height), Image.LANCZOS))
        out.append(im)
    return np.hstack(out)


def save_gif(frames, path, fps=12, max_colors=200, scale=1.0):
    """Write an optimised GIF (shared palette keeps the file small for the README)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pil = [Image.fromarray(f) for f in frames]
    if scale != 1.0:
        size = (int(pil[0].width * scale), int(pil[0].height * scale))
        pil = [p.resize(size, Image.LANCZOS) for p in pil]
    # one global palette from a sample of frames -> no flicker, smaller file
    idx = np.unique(np.linspace(0, len(pil) - 1, 8).astype(int))
    sample = Image.fromarray(np.vstack([np.asarray(pil[i]) for i in idx]))
    pal = sample.quantize(colors=max_colors, method=Image.Quantize.MEDIANCUT)
    q = [p.quantize(palette=pal, dither=Image.Dither.NONE) for p in pil]
    q[0].save(path, save_all=True, append_images=q[1:], duration=int(1000 / fps), loop=0, optimize=True)
    if shutil.which("gifsicle"):
        subprocess.run(["gifsicle", "-O3", "--batch", str(path)], check=False)
    print(f"  GIF  {path.relative_to(ROOT)}  ({path.stat().st_size / 1e6:.2f} MB, {len(frames)} frames)")


def save_mp4(frames, path, fps=24):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not shutil.which("ffmpeg"):
        print("  (ffmpeg not found, skipping MP4)")
        return
    h, w = frames[0].shape[:2]
    h2, w2 = h - h % 2, w - w % 2
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                          "-s", f"{w2}x{h2}", "-r", str(fps), "-i", "-", "-c:v", "libx264",
                          "-pix_fmt", "yuv420p", "-crf", "26", "-movflags", "+faststart", str(path)],
                         stdin=subprocess.PIPE)
    for f in frames:
        p.stdin.write(np.ascontiguousarray(f[:h2, :w2]).tobytes())
    p.stdin.close()
    p.wait()
    print(f"  MP4  {path.relative_to(ROOT)}  ({path.stat().st_size / 1e6:.2f} MB)")
