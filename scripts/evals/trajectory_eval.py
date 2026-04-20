#!/usr/bin/env python3

import argparse
import os

import sys
from contextlib import contextmanager

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

from trajectory_evals.alignment import compute_errors
from trajectory_evals.io import load_csv, load_xml, write_trajectory_csv
from trajectory_evals.plotting import plot_errors

def _apply_static_transform(row, T_src_dst):
    """
    Applies a static transform to a dataframe row containing
    [tx, ty, tz, qx, qy, qz, qw], interpreted as T_world_src.

    T_src_dst is a 4x4 mapping p_dst -> p_src.
    """
    pos = row[["tx", "ty", "tz"]].values.astype(float)
    quat = row[["qx", "qy", "qz", "qw"]].values.astype(float)

    T_world_src = np.eye(4)
    T_world_src[:3, :3] = R.from_quat(quat).as_matrix()
    T_world_src[:3, 3] = pos

    T_world_dst = T_world_src @ T_src_dst

    new_pos = T_world_dst[:3, 3]
    new_quat = R.from_matrix(T_world_dst[:3, :3]).as_quat()
    return np.concatenate([new_pos, new_quat])

def invert_pose(row):
    """
    Inverts the pose of a dataframe row containing [tx, ty, tz, qx, qy, qz, qw].
    """
    pos = row[["tx", "ty", "tz"]].values.astype(float)
    quat = row[["qx", "qy", "qz", "qw"]].values.astype(float)
    inv_rot = R.from_quat(quat).inv()
    inv_pos = -inv_rot.apply(pos)
    inv_quat = inv_rot.as_quat()
    return np.concatenate([inv_pos, inv_quat])

def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot APE/RPE errors between estimated and reference trajectories."
    )
    parser.add_argument(
        "--dir",
        default="",
        help=(
            "Directory with trajectories, will automatically use <dir>/trajectory_frames.csv for test "
            "and <dir>/trajectory_nav.csv for ref if --ref not provided."
        )
    )
    parser.add_argument(
        "--test",
        default=None,
        help="Estimated trajectory CSV path.",
    )
    parser.add_argument(
        "--frame-stats",
        default=None,
        help=(
            "Frame-stats CSV path (must contain 'timestamp' and 'tracking_state'). "
            "Used to compute uptime/downtime. If omitted and --traj-dir is provided, "
            "defaults to <traj-dir>/trajectory_frame_stats.csv when it exists."
        ),
    )
    parser.add_argument(
        "--invert-test",
        action="store_true",
        help="Invert the test pose matrix, use this if you have written cTw instead of wTc in the trajectory CSV."
    )
    parser.add_argument(
        "--ref",
        default=None,
        help="Reference trajectory CSV path.",
    )
    parser.add_argument(
        "--ref-group-id",
        type=int,
        default=0,
        help="XML reference cameras group id to load (used only when --ref ends with .xml).",
    )
    parser.add_argument(
        "--delta-t",
        type=float,
        default=5.0,
        help="RPE delta time in seconds.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.25,
        help="RPE time tolerance in seconds.",
    )
    parser.add_argument(
        "--no-scale",
        action="store_true",
        help="Disable scale estimation in Umeyama alignment.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory to save plot image (optional).",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the plot window (opt-in).",
    )
    parser.add_argument(
        "--segment-gap-seconds",
        type=float,
        default=0.6,
        help="Gap threshold in seconds for segment splits.",
    )
    parser.add_argument(
        "--min-segment-samples",
        type=int,
        default=1,
        help=(
            "Minimum number of samples required to keep a segment. "
            "Shorter segments are dropped."
        ),
    )
    parser.add_argument(
        "--alignment-on",
        choices=["trajectory_start", "central"],
        default="trajectory_start",
        help="Alignment mode: start-of-segment or central (Umeyama).",
    )
    parser.add_argument(
        "--platform-config",
        required=True,
        help="Path to static_tf platform YAML (e.g. src/static_tf/config/seeker-cheryl.yaml).",
    )
    parser.add_argument(
        "--test-frame",
        required=True,
        help="Frame id that the test trajectory is expressed in (static_tf frame name).",
    )
    parser.add_argument(
        "--ref-frame",
        required=True,
        help="Frame id that the reference trajectory is expressed in (static_tf frame name).",
    )
    return parser.parse_args()

def _compute_tracking_uptime_stats(
    df_stats: pd.DataFrame,
    start_ts: float,
    end_ts: float,
    intervals=None,
    forced_down_intervals=None,
):
    if df_stats is None or df_stats.empty:
        return None
    if "timestamp" not in df_stats.columns or "tracking_state" not in df_stats.columns:
        return None

    lo = min(float(start_ts), float(end_ts))
    hi = max(float(start_ts), float(end_ts))
    if intervals:
        mask = np.zeros(len(df_stats), dtype=bool)
        for seg_start, seg_end in intervals:
            seg_lo = min(float(seg_start), float(seg_end))
            seg_hi = max(float(seg_start), float(seg_end))
            mask |= (df_stats["timestamp"] >= seg_lo) & (df_stats["timestamp"] <= seg_hi)
        dfw = df_stats[mask].copy()
    else:
        dfw = df_stats[(df_stats["timestamp"] >= lo) & (df_stats["timestamp"] <= hi)].copy()
    if dfw.empty:
        return None

    ts = dfw["tracking_state"].to_numpy()
    ok = (ts == 1)
    lost = (ts == 0)
    forced_down_n = 0
    if forced_down_intervals:
        forced_down_mask = np.zeros(len(dfw), dtype=bool)
        for seg_start, seg_end in forced_down_intervals:
            seg_lo = min(float(seg_start), float(seg_end))
            seg_hi = max(float(seg_start), float(seg_end))
            forced_down_mask |= (dfw["timestamp"] >= seg_lo) & (dfw["timestamp"] <= seg_hi)
        forced_down_n = int(forced_down_mask.sum())
        # Trimmed segments count as downtime regardless of recorded tracking_state.
        ok = ok & (~forced_down_mask)
        lost = lost | forced_down_mask
    total = int(ts.shape[0])
    ok_n = int(ok.sum())
    lost_n = int(lost.sum())

    # Count loss events as contiguous LOST segments
    loss_events = 0
    in_loss = False
    for v in ts:
        is_lost = (v == 0)
        if is_lost and not in_loss:
            loss_events += 1
            in_loss = True
        elif not is_lost:
            in_loss = False

    uptime_pct = 100.0 * ok_n / total if total else 0.0
    downtime_pct = 100.0 * lost_n / total if total else 0.0

    return {
        "total_frames": total,
        "ok_frames": ok_n,
        "lost_frames": lost_n,
        "loss_events": int(loss_events),
        "uptime_pct": float(uptime_pct),
        "downtime_pct": float(downtime_pct),
        "t_start": float(dfw["timestamp"].iloc[0]),
        "t_end": float(dfw["timestamp"].iloc[-1]),
        "forced_down_frames": forced_down_n,
    }

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
def _tee_stdout_to_file(path: str):
    """
    Context manager that duplicates stdout to a file while still printing to console.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        old = sys.stdout
        sys.stdout = _TeeStream(old, f)
        try:
            yield
        finally:
            sys.stdout = old


def main():
    args = parse_args()

    if args.dir:
        test_traj_path = os.path.join(args.dir, "trajectory_frames.csv")
        ref_traj_path = os.path.join(args.dir, "trajectory_nav.csv")
        if args.ref:
            ref_traj_path = args.ref
    else:
        test_traj_path = args.test
        ref_traj_path = args.ref

    # Verify test trajectory path exists
    if not os.path.exists(test_traj_path):
        print(f'Test trajectory {test_traj_path} not found')
        return

    print(f'Test trajectory {test_traj_path} found')

    if not os.path.exists(ref_traj_path):
        print(f'Ref trajectory {ref_traj_path} not found')
        return

    print(f'Ref trajectory {ref_traj_path} found')

    output_dir = args.output_dir or (
        os.path.join(args.dir, "trajectory_errors") if args.dir else None
    )
    if output_dir and args.no_scale:
        output_dir = output_dir + "_no_scale"

    print(f'Output directory {output_dir}')

    tee_ctx = None
    if output_dir:
        log_path = os.path.join(output_dir, "trajectory_errors_console.txt")
        tee_ctx = _tee_stdout_to_file(log_path)

    # From here on, duplicate console output into output_dir (when set).
    if tee_ctx:
        with tee_ctx:
            _run_main_with_optional_logging(args, test_traj_path, ref_traj_path, output_dir)
    else:
        _run_main_with_optional_logging(args, test_traj_path, ref_traj_path, output_dir)


def _run_main_with_optional_logging(args, test_traj_path, ref_traj_path, output_dir):

    df_est = load_csv(test_traj_path, "Test trajectory")

    # Remove rows with nan
    df_est = df_est.dropna()

    # Invert the test pose matrix
    if args.invert_test:
        df_est[["tx", "ty", "tz", "qx", "qy", "qz", "qw"]] = df_est.apply(
            lambda row: invert_pose(row), axis=1, result_type='expand'
        )

    try:
        from static_tf.loader import load_tree
    except Exception as e:
        print(f"Failed to import static_tf. Is it installed in your venv? Error: {e}")
        return

    tree = load_tree(args.platform_config)

    if args.test_frame != args.ref_frame:
        # tree.lookup(target, source) returns T_target_source mapping p_source -> p_target
        # We need T_test_ref mapping p_ref -> p_test to compute:
        #   T_world_ref = T_world_test @ T_test_ref
        T_test_ref = tree.lookup(args.test_frame, args.ref_frame)
        df_est[["tx", "ty", "tz", "qx", "qy", "qz", "qw"]] = df_est.apply(
            lambda row: _apply_static_transform(row, T_test_ref),
            axis=1,
            result_type="expand",
        )

    # Verify reference trajectory path exists
    if not os.path.exists(ref_traj_path):
        print(f'Ref trajectory {ref_traj_path} not found')
        return

    ref_lower = ref_traj_path.lower()
    if ref_lower.endswith(".csv"):
        df_ref = load_csv(ref_traj_path, "Ref trajectory")
    elif ref_lower.endswith(".xml"):
        df_ref = load_xml(
            ref_traj_path, "Ref trajectory", group_id=args.ref_group_id
        )
    else:
        print(f'Ref trajectory {ref_traj_path} file extension not supported, try .csv or .xml')
        return

    df_ref = df_ref.dropna()

    results = compute_errors(
        df_est,
        df_ref,
        delta_t=args.delta_t,
        tolerance=args.tolerance,
        with_scale=not args.no_scale,
        segment_gap_seconds=args.segment_gap_seconds,
        min_segment_samples=args.min_segment_samples,
    )

    segments = results["segments"]
    segment_scales = results["segment_scales"]
    t_est_valid = results["t_est_valid"]
    ref_dist = results["ref_dist"]
    ape_vec = results["ape_vec"]
    t_start = float(t_est_valid[0]) if len(t_est_valid) else None
    t_end = float(t_est_valid[-1]) if len(t_est_valid) else None

    # ── uptime / downtime ───────────────────────────────────────────────────────
    frame_stats_path = args.frame_stats
    if not frame_stats_path and args.dir:
        cand = os.path.join(args.dir, "trajectory_frame_stats.csv")
        if os.path.exists(cand):
            frame_stats_path = cand

    if frame_stats_path and t_start is not None and t_end is not None:
        try:
            df_stats = pd.read_csv(frame_stats_path)
            segment_intervals = [
                (float(t_est_valid[start_idx]), float(t_est_valid[end_idx - 1]))
                for start_idx, end_idx in segments
                if end_idx > start_idx
            ]
            trimmed_intervals = results.get("trimmed_segment_intervals", [])
            uptime_intervals = segment_intervals + trimmed_intervals
            stats = _compute_tracking_uptime_stats(
                df_stats,
                t_start,
                t_end,
                intervals=uptime_intervals,
                forced_down_intervals=trimmed_intervals,
            )
            if stats is None:
                print(
                    f"Tracking uptime: unavailable (missing columns or no rows in window) "
                    f"from frame-stats {frame_stats_path}"
                )
            else:
                print("Tracking uptime (from frame-stats within kept trajectory segments):")
                print(
                    f"  Window: {stats['t_start']:.3f} → {stats['t_end']:.3f}  "
                    f"({stats['total_frames']} frames)"
                )
                print(
                    f"  Up-time:   {stats['uptime_pct']:.1f}%  "
                    f"({stats['ok_frames']} OK frames)"
                )
                print(
                    f"  Down-time: {stats['downtime_pct']:.1f}%  "
                    f"({stats['lost_frames']} LOST frames)"
                )
                print(
                    f"  Losses: {stats['loss_events']} events, {stats['lost_frames']} total LOST frames"
                )
                if stats.get("forced_down_frames", 0) > 0:
                    print(
                        "  Trimmed-segment downtime: "
                        f"{int(stats['forced_down_frames'])} frame-stats rows "
                        "counted as LOST"
                    )
        except Exception as e:
            print(f"Tracking uptime: failed to compute from {frame_stats_path}: {e}")
    else:
        if t_start is None or t_end is None:
            print("Tracking uptime: unavailable (no valid estimated trajectory timestamps).")
        else:
            print("Tracking uptime: unavailable (no frame-stats CSV provided/found).")

    trimmed_seg_n = int(results.get("segments_trimmed", 0))
    if trimmed_seg_n > 0:
        print(
            "Segment trimming: "
            f"kept {int(results.get('segments_kept', len(segments)))}/"
            f"{int(results.get('segments_total', len(segments)))} segments, "
            f"removed {trimmed_seg_n} short segments "
            f"({int(results.get('samples_trimmed', 0))} samples)"
        )

    print("Per-segment metrics:")
    header = (
        f"{'Seg':>3}  {'Samples':>7}  {'T_start':>13}  {'T_end':>13}  "
        f"{'T_duration':>10}  {'Dist_m':>9}  {'Scale':>8}  {'RMSE_m':>8}  "
        f"{'Mean_m':>8}  {'Max_m':>8}"
    )
    print(header)
    print("-" * len(header))
    for idx, (start_idx, end_idx) in enumerate(segments):
        seg_slice = slice(start_idx, end_idx)
        seg_times = t_est_valid[seg_slice]
        seg_ape = np.linalg.norm(ape_vec[seg_slice], axis=1)
        seg_rmse = np.sqrt(np.mean(seg_ape**2)) if seg_ape.size else 0.0
        seg_mean = float(np.mean(seg_ape)) if seg_ape.size else 0.0
        seg_max = float(np.max(seg_ape)) if seg_ape.size else 0.0
        seg_duration = float(seg_times[-1] - seg_times[0]) if seg_times.size > 1 else 0.0
        seg_dist = float(ref_dist[end_idx - 1] - ref_dist[start_idx])
        seg_scale = segment_scales[idx] if idx < len(segment_scales) else float("nan")
        print(
            f"{idx + 1:>3}  {end_idx - start_idx:>7}  "
            f"{seg_times[0]:>13.3f}  {seg_times[-1]:>13.3f}  "
            f"{seg_duration:>10.3f}  {seg_dist:>9.2f}  {seg_scale:>8.4f}  {seg_rmse:>8.3f}  "
            f"{seg_mean:>8.3f}  {seg_max:>8.3f}"
        )

    if output_dir:
        out_path = os.path.join(output_dir, "trajectory_aligned.csv")

        # --- build per-row error columns ---
        N = len(t_est_valid)

        # APE columns (defined at every estimated timestamp)
        extra = {
            "ape_trans_m": results["ape_trans"],
            "ape_rot_deg": results["ape_rot_deg"],
            "ape_vec_x":   results["ape_vec"][:, 0],
            "ape_vec_y":   results["ape_vec"][:, 1],
            "ape_vec_z":   results["ape_vec"][:, 2],
        }

        # Segment index and Umeyama scale factor per pose (alignment-strategy-aware)
        seg_idx_col   = np.full(N, -1, dtype=int)
        seg_scale_col = np.full(N, np.nan)
        for seg_i, (seg_s, seg_e) in enumerate(segments):
            seg_idx_col[seg_s:seg_e]   = seg_i
            seg_scale_col[seg_s:seg_e] = (
                segment_scales[seg_i] if seg_i < len(segment_scales) else np.nan
            )
        extra["segment_idx"]   = seg_idx_col
        extra["segment_scale"] = seg_scale_col

        # RPE columns (defined only where a full delta_t window existed; NaN elsewhere)
        rpe_trans_col = np.full(N, np.nan)
        rpe_rot_col   = np.full(N, np.nan)
        rpe_scale_col = np.full(N, np.nan)
        rpe_times_arr = results["rpe_times"]
        if len(rpe_times_arr) > 0:
            rpe_idx = np.searchsorted(t_est_valid, rpe_times_arr)
            valid_mask = rpe_idx < N
            rpe_trans_col[rpe_idx[valid_mask]] = results["rpe_trans"][valid_mask]
            rpe_rot_col[rpe_idx[valid_mask]]   = results["rpe_rot_deg"][valid_mask]
            if len(results["rpe_scale"]) == len(rpe_times_arr):
                rpe_scale_col[rpe_idx[valid_mask]] = results["rpe_scale"][valid_mask]
        extra["rpe_trans_m"] = rpe_trans_col
        extra["rpe_rot_deg"] = rpe_rot_col
        extra["rpe_scale"]   = rpe_scale_col

        write_trajectory_csv(
            out_path,
            t_est_valid,
            results["p_est_aligned"],
            results["q_est_aligned"],
            extra_columns=extra,
        )
        print(f"Aligned trajectory written to {out_path}")

    plot_errors(
        results,
        alignment_on=args.alignment_on,
        output_dir=output_dir,
        show=args.show,
    )

if __name__ == "__main__":
    main()