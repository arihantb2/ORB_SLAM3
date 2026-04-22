"""Analysis engine: temporal and stereo visual consistency computation."""

from __future__ import annotations

import warnings
from pathlib import Path
from typing import Sequence

import cv2
import pandas as pd

from visual_consistency import metrics as M
from visual_consistency.loader import iter_image_pairs, load_image_grayscale_u8


def run_analysis(
    image_dir: str,
    extensions: Sequence[str] = (".png", ".tif", ".tiff"),
    progress: bool = True,
    max_images: int | None = None,
) -> pd.DataFrame:
    """Compute all six visual consistency metrics for every consecutive pair.

    Iterates the image sequence as a generator so only two frames are held in
    memory at a time.  Malformed pairs are skipped with a warning rather than
    aborting the entire run.

    Args:
        image_dir:  Directory of image files (sorted alphanumerically).
        extensions: File extensions to include (case-insensitive).
        progress:   Show a tqdm progress bar; silently skipped if tqdm is not
                    installed.
        max_images: If set, process at most this many images (i.e. at most
                    max_images-1 pairs).  Useful for quick previews on large
                    directories.

    Returns:
        DataFrame with columns:
            frame_i, frame_j,
            bhattacharyya, zncc, ssim, eog, phase_psr, spectral_centroid

    Raises:
        ValueError: if the directory has fewer than 2 matching images, or if
                    every pair fails to process.
    """
    try:
        from tqdm import tqdm as _tqdm
        use_tqdm = progress
    except ImportError:
        use_tqdm = False

    pairs_iter = iter_image_pairs(image_dir, extensions)

    if use_tqdm:
        exts = {e.lower() if e.startswith(".") else f".{e.lower()}" for e in extensions}
        n_files = sum(1 for p in Path(image_dir).iterdir() if p.suffix.lower() in exts)
        if max_images is not None:
            n_files = min(n_files, max_images)
        pairs_iter = _tqdm(pairs_iter, total=max(0, n_files - 1), unit="pair")

    max_pairs = (max_images - 1) if max_images is not None else None

    rows = []
    for idx, path_i, img_i, path_j, img_j in pairs_iter:
        if max_pairs is not None and idx >= max_pairs:
            break
        try:
            row = {
                "frame_i":           Path(path_i).name,
                "frame_j":           Path(path_j).name,
                "bhattacharyya":     M.bhattacharyya_distance(img_i, img_j),
                "zncc":              M.zncc(img_i, img_j),
                "ssim":              M.ssim(img_i, img_j),
                "eog":               M.energy_of_gradient(img_i),
                "phase_psr":         M.phase_correlation_psr(img_i, img_j),
                "spectral_centroid": M.spectral_centroid(img_i),
            }
        except Exception as exc:  # noqa: BLE001
            warnings.warn(
                f"Skipping pair {idx} "
                f"({Path(path_i).name} → {Path(path_j).name}): {exc}",
                stacklevel=2,
            )
            continue
        rows.append(row)

    if not rows:
        raise ValueError(
            f"No valid image pairs could be processed in {image_dir!r}"
        )

    return pd.DataFrame(rows)


def run_stereo_analysis(
    left_dir: str,
    right_dir: str,
    extensions: Sequence[str] = (".png", ".tif", ".tiff"),
    progress: bool = True,
    max_images: int | None = None,
) -> pd.DataFrame:
    """Compute all six visual consistency metrics for each left/right stereo pair.

    Pairs images by sorted index: left[i] ↔ right[i].  Only pair-wise
    metrics are computed (Bhattacharyya, ZNCC, SSIM, Phase PSR); single-frame
    metrics (EoG, Spectral Centroid) are omitted.  If the directories have
    different image counts the shorter list determines how many pairs are
    processed.  If left and right images have different spatial resolutions
    the right image is resized to match the left before metric computation.

    Args:
        left_dir:   Directory of left-camera images.
        right_dir:  Directory of right-camera images.
        extensions: File extensions to include (case-insensitive).
        progress:   Show a tqdm progress bar.
        max_images: Cap on number of stereo pairs to process.

    Returns:
        DataFrame with columns:
            frame_left, frame_right, bhattacharyya, zncc, ssim, phase_psr

    Raises:
        ValueError: if either directory has no matching images, or if every
                    pair fails to process.
    """
    try:
        from tqdm import tqdm as _tqdm
        use_tqdm = progress
    except ImportError:
        use_tqdm = False

    exts = {e.lower() if e.startswith(".") else f".{e.lower()}" for e in extensions}
    left_paths = sorted(p for p in Path(left_dir).iterdir() if p.suffix.lower() in exts)
    right_paths = sorted(p for p in Path(right_dir).iterdir() if p.suffix.lower() in exts)

    if not left_paths:
        raise ValueError(
            f"No matching images in left_dir {left_dir!r} (extensions: {sorted(exts)})"
        )
    if not right_paths:
        raise ValueError(
            f"No matching images in right_dir {right_dir!r} (extensions: {sorted(exts)})"
        )

    n_pairs = min(len(left_paths), len(right_paths))
    if len(left_paths) != len(right_paths):
        warnings.warn(
            f"Left dir has {len(left_paths)} images, right has {len(right_paths)}. "
            f"Using first {n_pairs} pairs.",
            stacklevel=2,
        )
    if max_images is not None:
        n_pairs = min(n_pairs, max_images)

    pairs = list(zip(left_paths[:n_pairs], right_paths[:n_pairs]))
    if use_tqdm:
        pairs = _tqdm(pairs, total=n_pairs, unit="pair", desc="stereo pairs")

    rows = []
    for idx, (left_path, right_path) in enumerate(pairs):
        try:
            img_l = load_image_grayscale_u8(str(left_path))
            img_r = load_image_grayscale_u8(str(right_path))
            if img_l.shape != img_r.shape:
                warnings.warn(
                    f"Shape mismatch at pair {idx}: left {img_l.shape} vs "
                    f"right {img_r.shape}. Resizing right to match left.",
                    stacklevel=2,
                )
                img_r = cv2.resize(img_r, (img_l.shape[1], img_l.shape[0]))
            row = {
                "frame_left":    left_path.name,
                "frame_right":   right_path.name,
                "bhattacharyya": M.bhattacharyya_distance(img_l, img_r),
                "zncc":          M.zncc(img_l, img_r),
                "ssim":          M.ssim(img_l, img_r),
                "phase_psr":     M.phase_correlation_psr(img_l, img_r),
            }
        except Exception as exc:  # noqa: BLE001
            warnings.warn(
                f"Skipping stereo pair {idx} "
                f"({left_path.name} ↔ {right_path.name}): {exc}",
                stacklevel=2,
            )
            continue
        rows.append(row)

    if not rows:
        raise ValueError(
            f"No valid stereo pairs could be processed from "
            f"{left_dir!r} and {right_dir!r}"
        )

    return pd.DataFrame(rows)
