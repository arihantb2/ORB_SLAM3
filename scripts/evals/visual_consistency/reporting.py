"""Dashboard plotting and JSON summary for the visual consistency analysis.

plot_dashboard() creates a metric trend figure and returns it for the caller
to save and/or display.  compute_summary() produces the statistics dict.
write_summary_json() delegates to trajectory_evals.io.write_json so there is
no duplicate JSON serialisation code.

Both plot_dashboard() and compute_summary() accept an optional metric_defs
parameter (list of (col, title, subtitle, ylim) tuples) so callers that only
compute a subset of metrics (e.g. stereo analysis skips single-frame metrics)
can reuse the same functions without modification.

ylim is a (ymin, ymax) pair where ymax=None means auto-scale the upper bound.
Bounded metrics (Bhattacharyya, ZNCC, SSIM) get fixed axes so that plots from
different datasets are directly comparable.  Unbounded metrics (EoG, Phase PSR,
Spectral Centroid) have their lower bound fixed at 0 and the upper auto-scaled.
"""

from __future__ import annotations

import math
import os
from datetime import datetime, timezone
from typing import List, Optional, Sequence, Tuple

import cv2
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import pandas as pd

from trajectory_evals.io import write_json

# (column_name, subplot_title, y-axis description, (ymin, ymax|None))
# Bounded metrics get fixed axes for cross-dataset comparability.
# Unbounded metrics fix only the lower bound (0); upper auto-scales to data.
_METRIC_DEFS = [
    ("bhattacharyya",     "Bhattacharyya Distance",  "intensity distribution distance", (0.0,  1.0)),
    ("zncc",              "ZNCC",                     "texture similarity  [−1, 1]",     (-1.0, 1.0)),
    ("ssim",              "SSIM",                     "structural similarity  [−1, 1]",  (-1.0, 1.0)),
    ("eog",               "Energy of Gradient",       "sharpness per pixel",             (0.0,  None)),
    ("phase_psr",         "Phase Correlation PSR",    "geometric alignment confidence",  (0.0,  None)),
    ("spectral_centroid", "Spectral Centroid  (px)",  "mean radial frequency",           (0.0,  None)),
]

# Pair-only metrics: EoG and Spectral Centroid omitted (single-frame).
STEREO_METRIC_DEFS = [
    ("bhattacharyya", "Bhattacharyya Distance", "intensity distribution distance", (0.0,  1.0)),
    ("zncc",          "ZNCC",                   "texture similarity  [−1, 1]",     (-1.0, 1.0)),
    ("ssim",          "SSIM",                   "structural similarity  [−1, 1]",  (-1.0, 1.0)),
    ("phase_psr",     "Phase Correlation PSR",  "geometric alignment confidence",  (0.0,  None)),
]

_OUTLIER_SIGMA = 2.0  # threshold for red-triangle outlier markers

_MetricDefs = List[Tuple[str, str, str, Tuple]]


def plot_dashboard(
    df: pd.DataFrame,
    metric_defs: Optional[_MetricDefs] = None,
) -> plt.Figure:
    """Create a metric trend dashboard figure with one subplot per metric.

    Grid is auto-sized: 2 columns, ceil(n/2) rows.  For the default 6 metrics
    this gives the familiar 3×2 layout; for 4 metrics a 2×2 grid is used.

    Each subplot shows:
      - A continuous line for the time series.
      - Scatter points coloured blue (inliers) or red triangles (outliers).
      - A dashed line at the mean ± 2σ reference band.

    The caller is responsible for saving and/or displaying the figure.
    Call plot_style.apply_paper_style() before this function.

    Args:
        df:          DataFrame with metric columns.
        metric_defs: List of (col, title, subtitle, ylim) tuples.  Defaults to
                     _METRIC_DEFS (all six temporal metrics).

    Returns:
        The matplotlib Figure object.
    """
    if metric_defs is None:
        metric_defs = _METRIC_DEFS

    n = len(metric_defs)
    ncols = 2
    nrows = math.ceil(n / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, nrows * 10 / 3))
    axes_flat = axes.flatten() if n > 1 else [axes]
    x = np.arange(len(df))

    # Hide any spare axes when n is odd
    for spare in axes_flat[n:]:
        spare.set_visible(False)

    for ax, (col, title, subtitle, ylim) in zip(axes_flat, metric_defs):
        vals = df[col].to_numpy(dtype=float)
        mu = float(np.mean(vals))
        sigma = float(np.std(vals))
        outlier_mask = np.abs(vals - mu) > _OUTLIER_SIGMA * sigma

        ax.plot(x, vals, linewidth=0.9, color="steelblue", zorder=2)
        ax.scatter(
            x[~outlier_mask], vals[~outlier_mask],
            s=4, color="steelblue", zorder=3,
        )
        if outlier_mask.any():
            ax.scatter(
                x[outlier_mask], vals[outlier_mask],
                s=30, marker="^", color="tab:red", zorder=4,
                label=f"outlier (>{_OUTLIER_SIGMA:.0f}σ): {outlier_mask.sum()}",
            )
            ax.legend(fontsize=7, loc="upper right")

        # Mean line + ±2σ reference band
        ax.axhline(mu, color="dimgray", linestyle="--", linewidth=0.8, alpha=0.7)
        ax.axhspan(
            mu - _OUTLIER_SIGMA * sigma,
            mu + _OUTLIER_SIGMA * sigma,
            alpha=0.07, color="steelblue",
        )

        ax.xaxis.set_major_locator(mticker.MaxNLocator(integer=True))
        ax.set_title(f"{title}\n{subtitle}", fontsize=10)
        ax.set_xlabel("Pair index")
        ax.set_ylabel(col)
        ax.grid(axis="y", zorder=0)
        ax.xaxis.grid(False)

        # Apply fixed y bounds: both fixed for bounded metrics, floor-only for unbounded.
        ylo, yhi = ylim
        ax.set_ylim(bottom=ylo, top=yhi)

    fig.suptitle("AUV Visual Consistency Dashboard", fontsize=13, y=1.01)
    fig.tight_layout()
    return fig


def compute_summary(
    df: pd.DataFrame,
    image_dir: str,
    metric_defs: Optional[_MetricDefs] = None,
) -> dict:
    """Compute per-metric statistics and a consistency score for the run.

    Consistency score = 1 / (1 + CV) where CV = std / |mean|.
    Range [0, 1]; 1 = perfectly consistent (zero variance), lower = noisier.

    Args:
        df:          Output DataFrame from run_analysis() or run_stereo_analysis().
        image_dir:   Source directory (stored in metadata).
        metric_defs: List of (col, title, subtitle, ylim) tuples.  Defaults to
                     _METRIC_DEFS (all six temporal metrics).

    Returns:
        Dict suitable for write_summary_json().
    """
    if metric_defs is None:
        metric_defs = _METRIC_DEFS
    metrics_out: dict = {}
    for col, *_ in metric_defs:
        vals = df[col].dropna().to_numpy(float)
        mu = float(np.mean(vals))
        var = float(np.var(vals))
        std = float(np.std(vals))
        cv = std / (abs(mu) + 1e-10)
        metrics_out[col] = {
            "mean":              round(mu, 6),
            "variance":          round(var, 6),
            "std":               round(std, 6),
            "consistency_score": round(1.0 / (1.0 + cv), 4),
        }

    return {
        "n_pairs": int(len(df)),
        "metrics": metrics_out,
        "metadata": {
            "image_dir":    str(Path(image_dir).resolve()),
            "generated_at": datetime.now(timezone.utc).isoformat(),
        },
    }


def write_summary_json(summary: dict, path: str) -> None:
    """Write the summary dict to JSON, creating parent dirs as needed.

    Delegates to trajectory_evals.io.write_json to avoid duplication.
    """
    write_json(path, summary)


# ---------------------------------------------------------------------------
# Strobe Stability Heatmap
# ---------------------------------------------------------------------------


def _hist_percentiles(hist_mat: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute P25 / P50 / P75 for each frame from a normalised histogram matrix.

    Args:
        hist_mat: (N, 256) float array where each row is a normalised histogram.

    Returns:
        Three (N,) arrays: p25, p50, p75 in intensity units [0, 255].
    """
    cdf = np.cumsum(hist_mat, axis=1)          # (N, 256)
    p25 = np.argmax(cdf >= 0.25, axis=1).astype(float)
    p50 = np.argmax(cdf >= 0.50, axis=1).astype(float)
    p75 = np.argmax(cdf >= 0.75, axis=1).astype(float)
    return p25, p50, p75


def plot_strobe_heatmap(
    image_dir: str,
    extensions: Sequence[str] = (".png", ".tif", ".tiff"),
    max_images: int | None = None,
    rolling_window: int | None = None,
    progress: bool = True,
) -> plt.Figure:
    """Create the Strobe Stability Heatmap figure (two stacked subplots).

    Top subplot — Intensity Distribution Heatmap:
      Base:     2D heatmap (X: frame index, Y: intensity 0-255).
                Each column is the normalised 256-bin grayscale histogram of
                one frame.  Bright regions indicate where intensity mass lives.
      Overlays: Rolling median (P50, solid white) and IQR band (P25–P75,
                dashed white lines + semi-transparent fill).  Smooth,
                horizontal lines = photometric consistency; sharp vertical
                deviations = strobe jitter.

    Bottom subplot — Δ Histogram (Delta Analysis):
      Each column is hist[i+1] − hist[i] for each intensity bin.
      Diverging colormap centred at zero highlights which bins gain or lose
      mass between frames — a direct read of strobe-induced intensity shifts.

    The caller is responsible for saving and/or displaying the figure.
    Call plot_style.apply_paper_style() before this function.

    Args:
        image_dir:      Directory of image files.
        extensions:     File extensions to include.
        max_images:     Cap on number of frames loaded.
        rolling_window: Smoothing window for percentile overlays (frames).
                        Defaults to max(5, min(N // 30, 50)).
        progress:       Show tqdm progress bar while loading images.

    Returns:
        The matplotlib Figure object.
    """
    from visual_consistency.loader import iter_images

    try:
        from tqdm import tqdm as _tqdm
        _wrap = _tqdm if progress else (lambda x, **kw: x)
    except ImportError:
        _wrap = lambda x, **kw: x  # noqa: E731

    # ── Build per-frame histogram matrix ─────────────────────────────────────
    exts = {e.lower() if e.startswith(".") else f".{e.lower()}" for e in extensions}
    n_total = sum(1 for p in __import__("pathlib").Path(image_dir).iterdir()
                  if p.suffix.lower() in exts)
    if max_images is not None:
        n_total = min(n_total, max_images)

    hist_rows = []
    for _idx, _path, img in _wrap(
        iter_images(image_dir, extensions, max_images),
        total=n_total, unit="frame", desc="building histograms",
    ):
        h = cv2.calcHist([img], [0], None, [256], [0, 256]).flatten().astype(float)
        h /= h.sum() if h.sum() > 0 else 1.0
        hist_rows.append(h)

    hist_mat = np.array(hist_rows)   # (N, 256)
    N = len(hist_mat)

    # ── Rolling percentile overlays ──────────────────────────────────────────
    win = rolling_window or max(5, min(N // 30, 50))
    p25_raw, p50_raw, p75_raw = _hist_percentiles(hist_mat)
    roll_kw = dict(window=win, center=True, min_periods=1)
    p25 = pd.Series(p25_raw).rolling(**roll_kw).median().to_numpy()
    p50 = pd.Series(p50_raw).rolling(**roll_kw).median().to_numpy()
    p75 = pd.Series(p75_raw).rolling(**roll_kw).median().to_numpy()

    x_frames = np.arange(N)

    # ── Delta histogram ───────────────────────────────────────────────────────
    delta_mat = np.diff(hist_mat, axis=0)   # (N-1, 256)

    # ── Plot ─────────────────────────────────────────────────────────────────
    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1, figsize=(14, 9),
        gridspec_kw={"height_ratios": [2, 1]},
    )

    # — Primary heatmap ——————————————————————————————————————————————————————
    im_top = ax_top.imshow(
        hist_mat.T,           # (256, N): intensity on Y, frame on X
        aspect="auto",
        origin="lower",
        cmap="plasma",
        interpolation="nearest",
    )
    cb_top = fig.colorbar(im_top, ax=ax_top, pad=0.01, fraction=0.03)
    cb_top.set_label("Normalised bin density", fontsize=9)

    ax_top.plot(x_frames, p50, color="white", linewidth=1.6,
                label=f"Rolling median  (w={win})", zorder=3)
    ax_top.plot(x_frames, p25, color="white", linewidth=0.9,
                linestyle="--", alpha=0.75, label="P₂₅ / P₇₅  (IQR bounds)", zorder=3)
    ax_top.plot(x_frames, p75, color="white", linewidth=0.9,
                linestyle="--", alpha=0.75, zorder=3)
    ax_top.fill_between(x_frames, p25, p75,
                        color="white", alpha=0.12, zorder=2)

    ax_top.xaxis.set_major_locator(mticker.MaxNLocator(integer=True))
    ax_top.yaxis.set_major_locator(mticker.MultipleLocator(32))
    ax_top.set_xlim(-0.5, N - 0.5)
    ax_top.set_ylim(-0.5, 255.5)
    ax_top.set_xlabel("Frame index")
    ax_top.set_ylabel("Intensity  (0 – 255)")
    ax_top.set_title(
        "Intensity Distribution over Time\n"
        "Rolling P₂₅ / P₅₀ / P₇₅ overlaid  ·  "
        "vertical deviations = strobe jitter",
        fontsize=10,
    )
    ax_top.legend(fontsize=8, loc="upper right",
                  framealpha=0.6, labelcolor="white",
                  facecolor="#333333")

    # — Delta heatmap ————————————————————————————————————————————————————————
    vmax = float(np.abs(delta_mat).max()) or 1e-6
    im_bot = ax_bot.imshow(
        delta_mat.T,          # (256, N-1)
        aspect="auto",
        origin="lower",
        cmap="RdBu_r",
        vmin=-vmax,
        vmax=vmax,
        interpolation="nearest",
    )
    cb_bot = fig.colorbar(im_bot, ax=ax_bot, pad=0.01, fraction=0.03)
    cb_bot.set_label("Δ bin density", fontsize=9)

    ax_bot.xaxis.set_major_locator(mticker.MaxNLocator(integer=True))
    ax_bot.yaxis.set_major_locator(mticker.MultipleLocator(32))
    ax_bot.set_xlim(-0.5, N - 1.5)
    ax_bot.set_ylim(-0.5, 255.5)
    ax_bot.set_xlabel("Frame index")
    ax_bot.set_ylabel("Intensity  (0 – 255)")
    ax_bot.set_title(
        "Δ Histogram — Frame-to-Frame Rate of Change\n"
        "Red = bin gained mass  ·  Blue = bin lost mass",
        fontsize=10,
    )

    fig.suptitle("Strobe Stability Heatmap", fontsize=13, y=1.01)
    fig.tight_layout()
    return fig
