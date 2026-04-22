"""Image loader with support for PNG (uint8 grayscale) and TIF (Bayer BGGR16).

PNG images are loaded directly as uint8 grayscale.
TIF images are uint16 Bayer BGGR16 and require demosaicing before use —
the pipeline mirrors vo_utils.h::convert_image():
  1. cv2.COLOR_BayerBG2BGR  — demosaic to BGR uint16
  2. divide by 256           — scale top 8 bits to uint8
  3. cv2.COLOR_BGR2GRAY      — convert to grayscale
"""

from __future__ import annotations

from pathlib import Path
from typing import Generator, Sequence, Tuple

import cv2
import numpy as np


def load_image_grayscale_u8(path: str) -> np.ndarray:
    """Load a single image as a uint8 (H, W) grayscale array.

    Supports:
      .png          — uint8 grayscale, loaded directly.
      .tif / .tiff  — uint16 Bayer BGGR16, demosaiced then scaled to 8-bit.

    Raises:
      IOError: if the file cannot be read or has an unexpected dtype/shape.
    """
    ext = Path(path).suffix.lower()

    if ext == ".png":
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise IOError(f"Failed to read PNG: {path}")
        return img

    if ext in (".tif", ".tiff"):
        img_raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if img_raw is None:
            raise IOError(f"Failed to read TIF: {path}")
        if img_raw.dtype != np.uint16 or img_raw.ndim != 2:
            raise IOError(
                f"Expected uint16 single-channel TIF (Bayer BGGR16), "
                f"got dtype={img_raw.dtype} ndim={img_raw.ndim}: {path}"
            )
        # Bayer BGGR16 → BGR uint16 → uint8 (top 8 bits) → grayscale
        bgr_u16 = cv2.cvtColor(img_raw, cv2.COLOR_BayerBG2BGR)
        bgr_u8 = (bgr_u16 / 256).astype(np.uint8)
        return cv2.cvtColor(bgr_u8, cv2.COLOR_BGR2GRAY)

    raise IOError(f"Unsupported image extension '{ext}': {path}")


def iter_images(
    image_dir: str,
    extensions: Sequence[str] = (".png", ".tif", ".tiff"),
    max_images: int | None = None,
) -> Generator[Tuple[int, str, np.ndarray], None, None]:
    """Yield (idx, path, img) for every image in sorted alphanumeric order.

    Args:
        image_dir:  Directory to scan.
        extensions: File extensions to include (case-insensitive).
        max_images: If set, stop after this many images.

    Raises:
        ValueError: if no matching images are found.
    """
    exts = {e.lower() if e.startswith(".") else f".{e.lower()}" for e in extensions}
    paths = sorted(
        p for p in Path(image_dir).iterdir() if p.suffix.lower() in exts
    )
    if max_images is not None:
        paths = paths[:max_images]
    if not paths:
        raise ValueError(
            f"No matching images found in {image_dir!r} "
            f"(extensions: {sorted(exts)})"
        )
    for idx, path_obj in enumerate(paths):
        yield idx, str(path_obj), load_image_grayscale_u8(str(path_obj))


def iter_image_pairs(
    image_dir: str,
    extensions: Sequence[str] = (".png", ".tif", ".tiff"),
) -> Generator[Tuple[int, str, np.ndarray, str, np.ndarray], None, None]:
    """Yield consecutive image pairs from a directory.

    Yields:
        (pair_index, path_i, img_i, path_j, img_j)

    Images are sorted alphanumerically (timestamp-ordered filenames like
    PR_YYYYMMDD_HHMMSS_mmm_AC16.png sort correctly). Only one image is held
    beyond the rolling two-image window to minimise RAM usage.

    Args:
        image_dir:  Directory to scan.
        extensions: File extensions to include (case-insensitive, with or
                    without leading dot).

    Raises:
        ValueError: if fewer than 2 matching images are found.
    """
    exts = {e.lower() if e.startswith(".") else f".{e.lower()}" for e in extensions}
    paths = sorted(
        p for p in Path(image_dir).iterdir() if p.suffix.lower() in exts
    )

    if len(paths) < 2:
        raise ValueError(
            f"Need at least 2 images in {image_dir!r}, found {len(paths)} "
            f"(extensions: {sorted(exts)})"
        )

    prev_path = str(paths[0])
    prev_img = load_image_grayscale_u8(prev_path)

    for idx, next_path_obj in enumerate(paths[1:]):
        next_path = str(next_path_obj)
        next_img = load_image_grayscale_u8(next_path)
        yield idx, prev_path, prev_img, next_path, next_img
        prev_path, prev_img = next_path, next_img
