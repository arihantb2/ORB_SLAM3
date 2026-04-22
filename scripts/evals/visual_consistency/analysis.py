"""Analysis engine: iterate over an image sequence and compute all six metrics."""

from __future__ import annotations

import warnings
from pathlib import Path
from typing import Sequence

import pandas as pd

from visual_consistency import metrics as M
from visual_consistency.loader import iter_image_pairs


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
