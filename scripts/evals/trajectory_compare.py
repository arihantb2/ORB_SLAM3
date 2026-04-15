#!/usr/bin/env python3
"""Compare APE/RPE error statistics across multiple trajectory evaluation runs.

Each input is a trajectory_aligned.csv produced by trajectory_eval.py (or a
directory that contains one).  The script produces side-by-side violin plots,
a CDF overlay, and a summary statistics table.

Outputs written to --output-dir
---------------------------------
  ape_comparison.png   violin/box comparison of APE translation per run
  ape_cdf.png          CDF overlay of APE translation for all runs
  rpe_comparison.png   violin/box comparison of RPE translation (if available)
  summary.csv          per-run statistics table (RMSE, mean, median, max)

Usage
-----
    python trajectory_compare.py run1/trajectory_aligned.csv run2/trajectory_aligned.csv
    python trajectory_compare.py runs/*/trajectory_aligned.csv \\
        --labels "Baseline" "Ours" -o comparison/ --show
    python trajectory_compare.py run_a/ run_b/ run_c/ -o comparison/
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from contextlib import contextmanager
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

_APE_COL = "ape_trans_m"
_RPE_COL = "rpe_trans_m"
_LOG_FILE = "trajectory_compare_console.txt"


# ---------------------------------------------------------------------------
# Console tee
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
# I/O helpers
# ---------------------------------------------------------------------------


def _resolve_path(p: str) -> Path:
    """Accept a CSV file path or a run directory.

    When given a directory, searches for trajectory_aligned.csv in:
      1. <dir>/trajectory_errors_no_scale/  (preferred)
      2. <dir>/trajectory_errors/
    """
    path = Path(p)
    if path.is_dir():
        for subdir in ("trajectory_errors_no_scale", "trajectory_errors"):
            candidate = path / subdir / "trajectory_aligned.csv"
            if candidate.exists():
                return candidate
        raise FileNotFoundError(
            f"No trajectory_aligned.csv found under trajectory_errors_no_scale/ "
            f"or trajectory_errors/ in: {p}"
        )
    if not path.exists():
        raise FileNotFoundError(f"File not found: {p}")
    return path


_DATETIME_RE = re.compile(r"^(\d{8}_\d{6})")


def _default_label(path: Path) -> str:
    """Extract the leading YYYYMMDD_HHMMSS datetime from the run's parent directory name.

    Expected path: <parent>/<run_dir>/trajectory_errors[_no_scale]/trajectory_aligned.csv
    The datetime prefix lives in <parent>.
    """
    if path.name == "trajectory_aligned.csv":
        # parent        = trajectory_errors[_no_scale]
        # parent.parent = run_dir
        # parent.parent.parent = the directory whose name carries the datetime
        name = path.parent.parent.name
    else:
        name = path.stem
    m = _DATETIME_RE.match(name)
    return m.group(1) if m else name


def _load_run(path: Path, label: str) -> dict:
    df = pd.read_csv(path)
    if _APE_COL not in df.columns:
        raise ValueError(
            f"{path}: column '{_APE_COL}' not found. Is this a trajectory_aligned.csv?"
        )
    ape = df[_APE_COL].dropna().to_numpy(dtype=float)
    rpe = (
        df[_RPE_COL].dropna().to_numpy(dtype=float)
        if _RPE_COL in df.columns
        else None
    )
    if rpe is not None and len(rpe) == 0:
        rpe = None
    return {"label": label, "ape": ape, "rpe": rpe, "path": path}


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------


def _stats(values: np.ndarray) -> dict:
    return {
        "rmse":   float(np.sqrt(np.mean(values**2))),
        "mean":   float(np.mean(values)),
        "median": float(np.median(values)),
        "max":    float(np.max(values)),
        "std":    float(np.std(values)),
    }


def _print_and_build_summary(runs: list[dict]) -> pd.DataFrame:
    rows = []
    col_w = max(len(r["label"]) for r in runs) + 2
    header = (
        f"{'Run':<{col_w}}  {'N':>6}  "
        f"{'APE RMSE':>10}  {'APE Mean':>10}  {'APE Med':>9}  {'APE Max':>9}  "
        f"{'RPE RMSE':>10}  {'RPE Mean':>10}  {'RPE Med':>9}"
    )
    print("\nSummary statistics (all values in metres):")
    print(header)
    print("-" * len(header))
    for r in runs:
        a = _stats(r["ape"])
        rpe_vals = r["rpe"]
        b = _stats(rpe_vals) if rpe_vals is not None else None
        print(
            f"{r['label']:<{col_w}}  {len(r['ape']):>6}  "
            f"{a['rmse']:>10.4f}  {a['mean']:>10.4f}  {a['median']:>9.4f}  {a['max']:>9.4f}  "
            + (
                f"{b['rmse']:>10.4f}  {b['mean']:>10.4f}  {b['median']:>9.4f}"
                if b else
                f"{'—':>10}  {'—':>10}  {'—':>9}"
            )
        )
        rows.append({
            "run":           r["label"],
            "n_poses":       len(r["ape"]),
            "ape_rmse_m":    round(a["rmse"],   4),
            "ape_mean_m":    round(a["mean"],   4),
            "ape_median_m":  round(a["median"], 4),
            "ape_max_m":     round(a["max"],    4),
            "ape_std_m":     round(a["std"],    4),
            "rpe_rmse_m":    round(b["rmse"],   4) if b else None,
            "rpe_mean_m":    round(b["mean"],   4) if b else None,
            "rpe_median_m":  round(b["median"], 4) if b else None,
        })
    print()
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------


_META_STAT_DEFS = [
    ("rmse",   "RMSE",   "///"),   # forward diagonal
    ("mean",   "Mean",   "..."),   # dots
    ("median", "Median", "xxx"),   # cross
    ("std",    "Std dev","---"),   # horizontal lines
]


def _bar_fig(
    runs: list[dict],
    col_key: str,
    ylabel: str,
    title: str,
) -> plt.Figure | None:
    """Bar chart — one RMSE bar per run, plus a meta stats cluster at the far right."""
    data = [
        (r["label"], r[col_key])
        for r in runs
        if r[col_key] is not None and len(r[col_key]) > 0
    ]
    if not data:
        return None

    labels, values = zip(*data)
    labels, values = list(labels), list(values)

    n_runs = len(labels)
    run_colors = [plt.get_cmap("tab10")(i % 10) for i in range(n_runs)]

    # Meta stats computed over all individual values pooled across runs
    pooled = np.concatenate(values)
    meta = _stats(pooled)

    # Layout: run bars at 0..n_runs-1, meta cluster starting at n_runs+1 (gap of 1)
    bar_w = 0.6
    meta_bar_w = 0.18
    n_meta = len(_META_STAT_DEFS)
    meta_center = n_runs + 1.0
    meta_offsets = np.linspace(-(n_meta - 1) / 2, (n_meta - 1) / 2, n_meta) * meta_bar_w

    fig_w = max(7, 1.4 * n_runs + 3)
    fig, ax = plt.subplots(figsize=(fig_w, 5))

    # Per-run RMSE bars
    x = np.arange(n_runs, dtype=float)
    rmses = [_stats(v)["rmse"] for v in values]
    bars = ax.bar(x, rmses, width=bar_w, color=run_colors, zorder=3)
    for bar, val in zip(bars, rmses):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.0002,
                f"{val:.4f}", ha="center", va="bottom", fontsize=7)

    # Meta cluster
    meta_tick_positions = []
    meta_tick_labels = []
    for offset, (stat_key, stat_label, hatch) in zip(meta_offsets, _META_STAT_DEFS):
        pos = meta_center + offset
        val = meta[stat_key]
        ax.bar(pos, val, width=meta_bar_w, color="lightgray", hatch=hatch,
               label=stat_label, zorder=3, edgecolor="dimgray", linewidth=0.8)
        ax.text(pos, val + 0.0002, f"{val:.4f}",
                ha="center", va="bottom", fontsize=7, color="dimgray", rotation=90)
        meta_tick_positions.append(pos)
        meta_tick_labels.append(stat_label)

    # Separator line between run bars and meta cluster
    ax.axvline(n_runs + 0.4, color="gray", linestyle="--", linewidth=0.8, zorder=2)

    # X-axis: run labels + individual meta stat labels
    all_tick_pos = list(x) + meta_tick_positions
    all_tick_lbl = list(labels) + meta_tick_labels
    ax.set_xticks(all_tick_pos)
    ax.set_xticklabels(all_tick_lbl, rotation=45, ha="center")

    # Mark the meta cluster region with a bracket label
    ax.annotate(
        "All runs (pooled)",
        xy=(meta_center, 0), xycoords=("data", "axes fraction"),
        xytext=(0, -42), textcoords="offset points",
        ha="center", fontsize=7, color="dimgray",
        annotation_clip=False,
    )

    ax.set_xlabel("")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.set_ylim(bottom=0)
    ax.legend(title="Meta stats", loc="upper right", fontsize=8)
    ax.grid(axis="y", zorder=0)
    ax.xaxis.grid(False)
    fig.tight_layout()
    return fig


def _cdf_fig(runs: list[dict]) -> plt.Figure:
    """CDF overlay of APE translation for all runs."""
    colors = [plt.get_cmap("tab10")(i % 10) for i in range(len(runs))]

    fig, ax = plt.subplots(figsize=(8, 5))
    for r, color in zip(runs, colors):
        ape = r["ape"]
        sorted_ape = np.sort(ape)
        cdf = np.arange(1, len(sorted_ape) + 1) / len(sorted_ape)
        rmse = float(np.sqrt(np.mean(ape**2)))
        ax.plot(
            sorted_ape, cdf,
            color=color, linewidth=1.8,
            label=f"{r['label']}  (RMSE = {rmse:.4f} m)",
        )

    ax.set_xlabel("APE translation  (m)")
    ax.set_ylabel("Cumulative fraction of poses")
    ax.set_title("APE Translation — CDF Comparison")
    ax.set_xlim(left=0)
    ax.set_ylim(0, 1)
    ax.legend()
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Core logic (called inside tee context)
# ---------------------------------------------------------------------------


def _run(args, runs: list[dict], labels: list[str], paths: list[Path]) -> None:
    # ── inputs ───────────────────────────────────────────────────────────────
    print("Inputs:")
    for r in runs:
        n_rpe = len(r["rpe"]) if r["rpe"] is not None else 0
        print(
            f"  {r['label']:<30}  {len(r['ape']):>6} APE poses"
            + (f",  {n_rpe} RPE samples" if n_rpe else "")
            + f"  ←  {r['path']}"
        )

    # ── summary statistics ────────────────────────────────────────────────────
    df_summary = _print_and_build_summary(runs)

    # ── plots ─────────────────────────────────────────────────────────────────
    fig_ape = _bar_fig(
        runs, "ape", "APE translation RMSE  (m)", "APE Translation — Run Comparison"
    )
    fig_cdf = _cdf_fig(runs)
    has_rpe = any(r["rpe"] is not None for r in runs)
    fig_rpe = (
        _bar_fig(runs, "rpe", "RPE translation RMSE  (m)", "RPE Translation — Run Comparison")
        if has_rpe else None
    )

    # ── save ──────────────────────────────────────────────────────────────────
    if args.output_dir:
        saved = []
        if fig_ape:
            p = os.path.join(args.output_dir, "ape_comparison.png")
            fig_ape.savefig(p)
            saved.append(p)
        if fig_cdf:
            p = os.path.join(args.output_dir, "ape_cdf.png")
            fig_cdf.savefig(p)
            saved.append(p)
        if fig_rpe:
            p = os.path.join(args.output_dir, "rpe_comparison.png")
            fig_rpe.savefig(p)
            saved.append(p)
        summary_path = os.path.join(args.output_dir, "summary.csv")
        df_summary.to_csv(summary_path, index=False)
        saved.append(summary_path)
        print("Saved:")
        for p in saved:
            print(f"  {p}")

    if args.show:
        plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def _parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Compare APE/RPE error statistics across multiple trajectory_eval.py runs. "
            "Inputs are trajectory_aligned.csv files (or their parent directories)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "inputs", nargs="+",
        help="Paths to trajectory_aligned.csv files or directories containing them.",
    )
    parser.add_argument(
        "--labels", nargs="+", default=None,
        help=(
            "Human-readable label for each run (one per input). "
            "Defaults to the parent directory name of each CSV."
        ),
    )
    parser.add_argument(
        "-o", "--output-dir", default=None,
        help="Directory to save plots and summary.csv.",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Display plot windows interactively after saving.",
    )
    return parser.parse_args()


def main():
    args = _parse_args()

    # Resolve paths, skipping any that don't have the required file
    if args.labels is not None and len(args.labels) != len(args.inputs):
        print(
            f"ERROR: --labels has {len(args.labels)} entries but {len(args.inputs)} inputs were given.",
            file=sys.stderr,
        )
        sys.exit(1)
    explicit_labels = args.labels or [None] * len(args.inputs)

    paths, labels = [], []
    for inp, lbl in zip(args.inputs, explicit_labels):
        try:
            p = _resolve_path(inp)
        except FileNotFoundError as e:
            print(f"WARNING: skipping {inp} — {e}", file=sys.stderr)
            continue
        paths.append(p)
        labels.append(lbl if lbl is not None else _default_label(p))

    # Load (before tee so load errors go to stderr cleanly)
    runs = []
    for path, label in zip(paths, labels):
        try:
            run = _load_run(path, label)
        except (ValueError, Exception) as e:
            print(f"WARNING: skipping {path} — {e}", file=sys.stderr)
            continue
        runs.append(run)

    if len(runs) < 2:
        print(
            f"ERROR: at least 2 valid runs are required, but only {len(runs)} could be loaded.",
            file=sys.stderr,
        )
        sys.exit(1)

    # Apply style before any figure is created
    plt.style.use(["seaborn-v0_8-paper", "seaborn-v0_8-whitegrid"])
    plt.rcParams.update({
        "savefig.dpi": 300,
        "figure.dpi": 100,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "legend.framealpha": 0.85,
        "lines.linewidth": 1.5,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })

    # Set up output dir and tee before any printing
    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)
    log_path = os.path.join(args.output_dir, _LOG_FILE) if args.output_dir else None
    tee_ctx = _tee_stdout(log_path) if log_path else None

    with (tee_ctx if tee_ctx else _null_ctx()):
        _run(args, runs, labels, paths)


if __name__ == "__main__":
    main()
