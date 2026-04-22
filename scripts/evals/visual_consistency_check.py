#!/usr/bin/env python3
"""AUV Temporal Visual Consistency Analysis.

Quantifies photometric, structural, and frequency-domain stability across a
sorted sequence of AUV image frames captured under strobe illumination.
Designed as a pre-flight health check for Visual Odometry pipelines.

Outputs (written to --output-dir if given)
------------------------------------------
  consistency_metrics.csv        per-pair raw metrics
  consistency_dashboard.png      3×2 metric trend dashboard
  strobe_stability_heatmap.png   intensity distribution heatmap + delta analysis
  analysis_summary.json          per-metric statistics and consistency scores
  visual_consistency_console.txt console log

Usage
-----
    vis-check images/                           # print summary only
    vis-check images/ -o results/ --show
    vis-check images/ --ext png -o results/     # restrict to PNG (one camera)
    vis-check i20251003_212212/ --ext tif -o results/
    vis-check images/ --max-images 100 -o results/  # quick preview on first 100
"""

from __future__ import annotations

import argparse
import os
import sys
from contextlib import contextmanager

import matplotlib.pyplot as plt

import plot_style
from visual_consistency.analysis import run_analysis
from visual_consistency.reporting import (
    compute_summary,
    plot_dashboard,
    plot_strobe_heatmap,
    write_summary_json,
)

_LOG_FILE = "visual_consistency_console.txt"


# ---------------------------------------------------------------------------
# Console tee (mirrors trajectory_compare.py pattern)
# ---------------------------------------------------------------------------


class _TeeStream:
    def __init__(self, *streams):
        self._streams = [s for s in streams if s is not None]

    def write(self, data):
        for s in self._streams:
            s.write(data)
        return len(data)

    def flush(self):
        for s in self._streams:
            s.flush()


@contextmanager
def _tee_stdout(path: str):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        old = sys.stdout
        sys.stdout = _TeeStream(old, f)
        try:
            yield
        finally:
            sys.stdout = old


@contextmanager
def _null_ctx():
    yield


# ---------------------------------------------------------------------------
# CLI argument parsing
# ---------------------------------------------------------------------------


def _parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Analyse visual consistency of a sequential AUV image sequence. "
            "Computes six metrics (Bhattacharyya, ZNCC, SSIM, EoG, Phase PSR, "
            "Spectral Centroid) for every consecutive frame pair."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "image_dir",
        help=(
            "Directory containing image files (PNG or TIF), processed in "
            "alphanumeric order. For mixed AC/FC camera directories pass "
            "--ext png or --ext tif to restrict to one channel."
        ),
    )
    parser.add_argument(
        "--ext", nargs="+", default=["png", "tif", "tiff"],
        metavar="EXT",
        help=(
            "File extensions to include (without leading dot). "
            "Default: png tif tiff."
        ),
    )
    parser.add_argument(
        "-o", "--output-dir", default=None,
        help="Directory to save CSV, PNG dashboard, JSON summary, and console log.",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Display the dashboard interactively after saving.",
    )
    parser.add_argument(
        "--no-progress", action="store_true",
        help="Suppress the tqdm progress bar (useful in CI/non-TTY environments).",
    )
    parser.add_argument(
        "--max-images", type=int, default=None, metavar="N",
        help=(
            "Process at most N images (N-1 pairs). "
            "Useful for a quick preview on large directories."
        ),
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Core logic (runs inside tee context so output is captured to log)
# ---------------------------------------------------------------------------


def _run(args) -> None:
    extensions = tuple(
        e.lower() if e.startswith(".") else f".{e.lower()}"
        for e in args.ext
    )
    print(f"Image dir : {args.image_dir}")
    print(f"Extensions: {', '.join(extensions)}")

    df = run_analysis(
        args.image_dir,
        extensions=extensions,
        progress=not args.no_progress,
        max_images=args.max_images,
    )
    print(f"\nProcessed {len(df)} consecutive frame pairs.")

    summary = compute_summary(df, args.image_dir)

    # Per-metric console table
    col_w = max(len(k) for k in summary["metrics"]) + 2
    header = (
        f"\n{'Metric':<{col_w}}  {'Mean':>12}  {'Std':>12}  {'Consistency':>12}"
    )
    print(header)
    print("-" * len(header.lstrip("\n")))
    for metric, stats in summary["metrics"].items():
        print(
            f"{metric:<{col_w}}  {stats['mean']:>12.4f}  "
            f"{stats['std']:>12.4f}  {stats['consistency_score']:>12.4f}"
        )
    print()

    fig_dash = plot_dashboard(df)

    print("Building strobe stability heatmap …")
    fig_heat = plot_strobe_heatmap(
        args.image_dir,
        extensions=extensions,
        max_images=args.max_images,
        progress=not args.no_progress,
    )

    if args.output_dir:
        saved = []

        csv_path = os.path.join(args.output_dir, "consistency_metrics.csv")
        df.to_csv(csv_path, index=False)
        saved.append(csv_path)

        dash_path = os.path.join(args.output_dir, "consistency_dashboard.png")
        fig_dash.savefig(dash_path, bbox_inches="tight")
        saved.append(dash_path)

        heat_path = os.path.join(args.output_dir, "strobe_stability_heatmap.png")
        fig_heat.savefig(heat_path, bbox_inches="tight")
        saved.append(heat_path)

        json_path = os.path.join(args.output_dir, "analysis_summary.json")
        write_summary_json(summary, json_path)
        saved.append(json_path)

        print("Saved:")
        for p in saved:
            print(f"  {p}")

    if args.show:
        plt.show()

    plt.close(fig_dash)
    plt.close(fig_heat)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    args = _parse_args()

    if not os.path.isdir(args.image_dir):
        print(
            f"ERROR: image_dir is not a directory: {args.image_dir}",
            file=sys.stderr,
        )
        sys.exit(1)

    # Apply paper style once before any figure is created
    plot_style.apply_paper_style()

    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)

    log_path = (
        os.path.join(args.output_dir, _LOG_FILE) if args.output_dir else None
    )
    tee_ctx = _tee_stdout(log_path) if log_path else None

    with (tee_ctx if tee_ctx else _null_ctx()):
        _run(args)


if __name__ == "__main__":
    main()
