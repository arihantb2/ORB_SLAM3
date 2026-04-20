#!/usr/bin/env python3
"""
ORB-SLAM3 Frame Inspector — CSV to JSON converter for the dashboard.

Convert a frame-stats CSV to a data JSON file that the dashboard can load:

    python3 build_frame_inspector_data.py <csv_path> [-o <out.json>]
                          [--trajectory <trajectory_aligned.csv>]

Default output: <csv_stem>_inspector_data.json next to the CSV.
When --trajectory is given, the aligned trajectory (pose + error columns)
is merged in and the 3D Trajectory tab is enabled in the dashboard.

Open the sidecar frame_inspector.html (same directory as this script) in a browser
and use the file picker to load the generated JSON.

Expected frame-stats CSV columns (from ORB-SLAM3 frame stats logger)
---------------------------------------------------------------------
    timestamp, tracking_state, is_keyframe, keypoints_detected,
    motion_model_primary, motion_model_success, motion_model_matches,
    motion_model_inliers, motion_model_retry, ref_kf_primary,
    ref_kf_fallback, ref_kf_success, ref_kf_matches, ref_kf_inliers,
    local_map_success, local_map_inliers, tracked_map_points,
    new_map_point_candidates, tracking_time_ms, image_name

Expected aligned trajectory CSV columns (from trajectory_eval.py)
-------------------------------------------------------------------------
    timestamp, tx, ty, tz, qx, qy, qz, qw,
    ape_trans_m, ape_rot_deg, ape_vec_x, ape_vec_y, ape_vec_z,
    segment_idx, segment_scale, rpe_trans_m, rpe_rot_deg, rpe_scale

Missing columns are silently skipped.
"""

import argparse
import math
import sys
from pathlib import Path

try:
    import pandas as pd
except ImportError:
    sys.exit("pandas is required: pip install pandas")

from trajectory_evals.io import write_json, load_csv as load_trajectory_csv

# ── Column config ─────────────────────────────────────────────────────────────

# (column, display_label, hex_color, match_value)
BOOL_FLAG_DEFS = [
    ("tracking_state",       "LOST frames",       "#ff4444", 0),
    ("is_keyframe",          "Keyframe",           "#f0c040", 1),
    ("motion_model_success", "MM success",         "#44aaff", 1),
    ("motion_model_retry",   "MM retry",           "#ff8800", 1),
    ("ref_kf_fallback",      "Ref-KF fallback",    "#cc44ff", 1),
    ("ref_kf_primary",       "Ref-KF primary",     "#ff44cc", 1),
    ("ref_kf_success",       "Ref-KF success",     "#44ffcc", 1),
    ("local_map_success",    "Local map success",  "#88ff44", 1),
]

# (column, short_label, line_color)
Y_AXIS_DEFS = [
    ("tracked_map_points",   "tracked_map_pts",   "#4a9eff"),
    ("keypoints_detected",   "keypoints_det",     "#f0c040"),
    ("motion_model_inliers", "mm_inliers",        "#44aaff"),
    ("ref_kf_inliers",       "refkf_inliers",     "#cc44ff"),
    ("local_map_inliers",    "localmap_inliers",  "#88ff44"),
    ("tracking_time_ms",     "tracking_time_ms",  "#ff8800"),
]

# (column, display_label) — trajectory error / pose columns from aligned trajectory CSV
TRAJ_COL_DEFS = [
    ("ape_trans_m",   "APE translation (m)"),
    ("ape_rot_deg",   "APE rotation (deg)"),
    ("segment_scale", "Segment scale"),
    ("rpe_trans_m",   "RPE translation (m)"),
    ("rpe_rot_deg",   "RPE rotation (deg)"),
    ("rpe_scale",     "RPE scale"),
    ("segment_idx",   "Segment index"),
]

# (column, short_label, line_color) — trajectory error signals for the time-series tab
TRAJ_SIG_DEFS = [
    ("ape_trans_m",   "ape_trans_m",  "#f38ba8"),
    ("ape_rot_deg",   "ape_rot_deg",  "#fab387"),
    ("rpe_trans_m",   "rpe_trans_m",  "#a6e3a1"),
    ("rpe_rot_deg",   "rpe_rot_deg",  "#94e2d5"),
    ("segment_scale", "seg_scale",    "#89dceb"),
    ("rpe_scale",     "rpe_scale",    "#b4befe"),
]

# Signals active on first load
DEFAULT_ACTIVE_SIGNALS = {"tracked_map_points", "tracking_time_ms"}

# ── Loaders ───────────────────────────────────────────────────────────────────

def load_frame_stats_csv(path):
    df = pd.read_csv(path)
    if "timestamp" not in df.columns:
        sys.exit(f"ERROR: 'timestamp' column not found in {path}")
    df["t"] = (df["timestamp"] - df["timestamp"].iloc[0]).round(3)
    return df

# ── Data payload builder ───────────────────────────────────────────────────────

def build_data_payload(df_stats, df_traj=None):
    """
    Build the JSON-serialisable data payload from a frame-stats DataFrame
    and an optional aligned trajectory DataFrame.

    When df_traj is provided, trajectory columns (pose + error stats) are
    merged into the unified 'raw' dict by timestamp. Rows without a matching
    trajectory timestamp get null for trajectory columns.
    """
    t0 = df_stats["timestamp"].iloc[0]

    # Merge trajectory into frame stats if provided
    if df_traj is not None:
        # Normalise trajectory timestamps to the same elapsed-seconds reference
        df_traj = df_traj.copy().sort_values("timestamp")
        df_stats = df_stats.copy().sort_values("timestamp")

        # Columns to bring in from the trajectory CSV
        traj_cols = ["tx", "ty", "tz", "qx", "qy", "qz", "qw"]
        for col, _ in TRAJ_COL_DEFS:
            if col in df_traj.columns:
                traj_cols.append(col)

        df_merged = pd.merge_asof(
            df_stats,
            df_traj[["timestamp"] + traj_cols],
            on="timestamp",
            direction="nearest",
            tolerance=0.01,   # 10 ms
        )
    else:
        df_merged = df_stats.copy().sort_values("timestamp")

    df = df_merged
    n          = len(df)
    duration_s = float(df["t"].iloc[-1]) if n > 1 else 0.0
    n_lost     = int((df["tracking_state"] == 0).sum()) if "tracking_state" in df.columns else 0
    n_kf       = int((df["is_keyframe"]    == 1).sum()) if "is_keyframe"    in df.columns else 0

    # Build unified raw dict — all columns needed for either tab
    used_cols = {"t", "tracking_state", "is_keyframe", "image_name"}
    for col, _, _, _ in BOOL_FLAG_DEFS:
        used_cols.add(col)
    for col, _, _ in Y_AXIS_DEFS:
        used_cols.add(col)
    if df_traj is not None:
        used_cols.update(["tx", "ty", "tz"])
        for col, _ in TRAJ_COL_DEFS:
            used_cols.add(col)

    present = [c for c in used_cols if c in df.columns]
    # Convert NaN/Inf → None so json.dump emits null (not the bare NaN token,
    # which is valid Python JSON but rejected by JavaScript's JSON.parse).
    _raw = df[present].to_dict(orient="list")
    raw  = {k: [None if isinstance(v, float) and not math.isfinite(v) else v
                for v in vs]
            for k, vs in _raw.items()}

    flags = [
        {"col": col, "label": label, "color": color, "val": val}
        for col, label, color, val in BOOL_FLAG_DEFS
        if col in df.columns
    ]

    sigs = [
        {"col": col, "label": label, "color": color,
         "default": col in DEFAULT_ACTIVE_SIGNALS, "group": "slam"}
        for col, label, color in Y_AXIS_DEFS
        if col in df.columns
    ]
    if not sigs:
        sys.exit("ERROR: no recognised Y-axis columns found in CSV")
    if not any(s["default"] for s in sigs):
        sigs[0]["default"] = True
    # Append trajectory error signals when trajectory data is present
    if df_traj is not None:
        for col, label, color in TRAJ_SIG_DEFS:
            if col in df.columns:
                sigs.append({"col": col, "label": label, "color": color,
                             "default": False, "group": "traj"})

    meta = {
        "run_name":    df_stats.attrs.get("run_name", "unknown"),
        "source_path": df_stats.attrs.get("source_path"),
        "n_frames":    n,
        "duration_s":  round(duration_s, 1),
        "n_lost":      n_lost,
        "pct_lost":    round(100.0 * n_lost / n, 1) if n else 0.0,
        "n_kf":        n_kf,
        "pct_kf":      round(100.0 * n_kf / n, 1) if n else 0.0,
    }

    # Color picker options for the 3D tab (only when trajectory is present)
    color_defs = []
    if df_traj is not None:
        for col, label in TRAJ_COL_DEFS:
            if col in df.columns:
                color_defs.append({"col": col, "label": label})
        for col, label, _ in Y_AXIS_DEFS:
            if col in df.columns:
                color_defs.append({"col": col, "label": label})

    return {"meta": meta, "raw": raw, "flags": flags, "sigs": sigs,
            "color_defs": color_defs}

# ── HTML shell (sidecar file) ──────────────────────────────────────────────────

_INSPECTOR_HTML = Path(__file__).resolve().parent / "frame_inspector.html"


def build_html_shell():
    """Return the static dashboard HTML string (no data embedded)."""
    return _INSPECTOR_HTML.read_text(encoding="utf-8")




# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ORB-SLAM3 Frame Inspector — convert frame-stats CSV to JSON for the dashboard.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  # Legacy usage: pass CSV + optional trajectory explicitly
  python3 build_inspector.py run_20251004/trajectory_frame_stats.csv
  python3 build_inspector.py run_20251004/trajectory_frame_stats.csv -o ~/reports/run_20251004_data.json
  python3 build_inspector.py run_20251004/trajectory_frame_stats.csv --trajectory run_20251004/trajectory_errors/trajectory_aligned.csv

  # New usage: point at a parent directory and let the script find the standard files
  python3 build_inspector.py --dir /path/to/run_20251004

  Then open frame_inspector.html (next to this script) in a browser and load the JSON.
""",
    )
    parser.add_argument(
        "csv", nargs="?", default=None,
        help="Frame stats CSV to convert to a data JSON file.",
    )
    parser.add_argument(
        "--dir", default=None,
        help=(
            "Parent directory containing 'trajectory_frame_stats.csv' and, optionally, "
            "'trajectory_errors/trajectory_aligned.csv'. When provided, the script "
            "searches for these standard filenames relative to this directory."
        ),
    )
    parser.add_argument(
        "--frame-stats", default=None,
        help=(
            "Explicit frame-stats CSV path. Overrides the CSV positional argument and "
            "--dir discovery when provided."
        ),
    )
    parser.add_argument(
        "--trajectory", "-t", default=None,
        help=(
            "Aligned trajectory CSV (output of trajectory_eval.py). "
            "Merged by timestamp into the payload and enables the 3D Trajectory tab."
        ),
    )
    parser.add_argument(
        "--output", "-o", default=None,
        help="Output path (default: <csv_stem>_inspector_data.json).",
    )
    args = parser.parse_args()

    # Resolve frame-stats CSV path according to precedence:
    # 1) --frame-stats
    # 2) positional csv
    # 3) --dir /path/to/run  → /path/to/run/trajectory_frame_stats.csv
    csv_path: Path | None = None
    if args.frame_stats:
        csv_path = Path(args.frame_stats).resolve()
    elif args.csv:
        csv_path = Path(args.csv).resolve()
    elif args.dir:
        base_dir = Path(args.dir).resolve()
        if not base_dir.exists():
            sys.exit(f"ERROR: directory not found: {base_dir}")
        candidate = base_dir / "trajectory_frame_stats.csv"
        if not candidate.exists():
            sys.exit(
                f"ERROR: frame-stats CSV not found under {base_dir} "
                f"(expected {candidate.name}). Use --frame-stats to point at a custom file."
            )
        csv_path = candidate

    if csv_path is None:
        parser.error("Provide either a CSV path, --frame-stats, or --dir.")

    if not csv_path.exists():
        sys.exit(f"ERROR: file not found: {csv_path}")

    out_path = (
        Path(args.output) if args.output
        else csv_path.with_name(csv_path.stem + "_inspector_data.json")
    )

    print(f"Loading frame stats  {csv_path} ...")
    df_stats = load_frame_stats_csv(csv_path)
    # Use parent directory name as run_name, and record the full source path for
    # the dashboard's "recent files" dropdown and metadata display.
    df_stats.attrs["run_name"] = csv_path.parent.name or csv_path.stem
    df_stats.attrs["source_path"] = str(csv_path)
    print(f"  {len(df_stats)} frames, {df_stats['t'].iloc[-1]:.1f} s")

    df_traj = None
    traj_path: Path | None = None
    if args.trajectory:
        traj_path = Path(args.trajectory).resolve()
    elif args.dir:
        base_dir = Path(args.dir).resolve()
        candidate = base_dir / "trajectory_errors" / "trajectory_aligned.csv"
        if candidate.exists():
            traj_path = candidate

    if traj_path is not None:
        if not traj_path.exists():
            sys.exit(f"ERROR: trajectory file not found: {traj_path}")
        print(f"Loading trajectory   {traj_path} ...")
        df_traj = load_trajectory_csv(str(traj_path), "Aligned trajectory")
        print(f"  {len(df_traj)} poses")

    payload = build_data_payload(df_stats, df_traj)
    write_json(str(out_path), payload)
    print(f"Data JSON  {out_path}  ({out_path.stat().st_size // 1024} KB)")
    if df_traj is not None:
        print("  3D Trajectory tab enabled (trajectory data merged).")
    print("Open frame_inspector.html in a browser and select this file via the file picker.")


if __name__ == "__main__":
    main()
