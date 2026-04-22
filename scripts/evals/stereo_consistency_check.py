#!/usr/bin/env python3
"""AUV Stereo Visual Consistency Analysis.

Quantifies photometric, structural, and frequency-domain similarity between
left and right camera frames at each time step.  Images are paired by sorted
index (left[i] ↔ right[i]).  No extrinsic calibration is required.

Useful for diagnosing stereo camera rig issues: exposure mismatch, one-sided
lens fouling, or synchronisation gaps — before running a stereo VO pipeline.

Outputs (written to --output-dir if given)
------------------------------------------
  stereo_consistency_metrics.csv   per-pair raw metrics
  stereo_consistency_dashboard.png 3×2 metric trend dashboard
  stereo_analysis_summary.json     per-metric statistics and consistency scores
  stereo_consistency_console.txt   console log

Usage
-----
    stereo-check images-left/ images-right/
    stereo-check images-left/ images-right/ -o results/ --show
    stereo-check images-left/ images-right/ --ext png -o results/
    stereo-check images-left/ images-right/ --max-images 200 -o results/
"""

from __future__ import annotations

import argparse
import os
import sys
from contextlib import contextmanager

import matplotlib.pyplot as plt

import plot_style
from visual_consistency.analysis import run_stereo_analysis
from visual_consistency.reporting import (
    STEREO_METRIC_DEFS,
    compute_summary,
    plot_dashboard,
    write_summary_json,
)

_LOG_FILE = "stereo_consistency_console.txt"


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


def _parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Analyse visual consistency between left and right stereo camera "
            "frames.  Computes six metrics (Bhattacharyya, ZNCC, SSIM, EoG, "
            "Phase PSR, Spectral Centroid) for every left/right pair matched "
            "by sorted index.  No extrinsic calibration required."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("left_dir", help="Directory of left-camera images.")
    parser.add_argument("right_dir", help="Directory of right-camera images.")
    parser.add_argument(
        "--ext", nargs="+", default=["png", "tif", "tiff"],
        metavar="EXT",
        help="File extensions to include (without leading dot). Default: png tif tiff.",
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
        help="Suppress the tqdm progress bar.",
    )
    parser.add_argument(
        "--max-images", type=int, default=None, metavar="N",
        help="Process at most N stereo pairs. Useful for quick previews.",
    )
    return parser.parse_args()


def _run(args) -> None:
    extensions = tuple(
        e.lower() if e.startswith(".") else f".{e.lower()}"
        for e in args.ext
    )
    print(f"Left dir  : {args.left_dir}")
    print(f"Right dir : {args.right_dir}")
    print(f"Extensions: {', '.join(extensions)}")

    df = run_stereo_analysis(
        args.left_dir,
        args.right_dir,
        extensions=extensions,
        progress=not args.no_progress,
        max_images=args.max_images,
    )
    print(f"\nProcessed {len(df)} stereo pairs.")

    summary = compute_summary(df, args.left_dir, metric_defs=STEREO_METRIC_DEFS)

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

    fig_dash = plot_dashboard(df, metric_defs=STEREO_METRIC_DEFS)

    if args.output_dir:
        saved = []

        csv_path = os.path.join(args.output_dir, "stereo_consistency_metrics.csv")
        df.to_csv(csv_path, index=False)
        saved.append(csv_path)

        dash_path = os.path.join(args.output_dir, "stereo_consistency_dashboard.png")
        fig_dash.savefig(dash_path, bbox_inches="tight")
        saved.append(dash_path)

        json_path = os.path.join(args.output_dir, "stereo_analysis_summary.json")
        write_summary_json(summary, json_path)
        saved.append(json_path)

        print("Saved:")
        for p in saved:
            print(f"  {p}")

    if args.show:
        plt.show()

    plt.close(fig_dash)


def main():
    args = _parse_args()

    for label, d in (("left_dir", args.left_dir), ("right_dir", args.right_dir)):
        if not os.path.isdir(d):
            print(f"ERROR: {label} is not a directory: {d}", file=sys.stderr)
            sys.exit(1)

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
