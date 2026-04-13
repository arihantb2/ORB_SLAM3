import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


def align_umeyama(model, data, with_scale=True):
    """
    Computes the Scale, Rotation, and Translation to align 'data' to 'model'.
    model: (N, 3) numpy array
    data:  (N, 3) numpy array
    Returns: s (float), R (3x3 matrix), t (3 vector)
    """
    mu_m = model.mean(axis=0)
    mu_d = data.mean(axis=0)

    m_centered = model - mu_m
    d_centered = data - mu_d

    # Correlation matrix
    H = d_centered.T @ m_centered / len(model)

    U, S, Vt = np.linalg.svd(H)
    Rot = Vt.T @ U.T

    # Reflection case
    if np.linalg.det(Rot) < 0:
        Vt[-1, :] *= -1
        Rot = Vt.T @ U.T

    if with_scale:
        d_rotated = d_centered @ Rot.T
        denom = np.sum(d_centered**2)
        # Degenerate segments (e.g., single sample) have zero variance.
        # Fall back to unit scale to avoid NaN propagation.
        if denom <= np.finfo(float).eps:
            s = 1.0
        else:
            s = np.sum(d_rotated * m_centered) / denom
    else:
        s = 1.0

    t = mu_m - s * (mu_d @ Rot.T)
    return s, Rot, t


def split_segments_by_gap(timestamps, gap_threshold):
    if len(timestamps) == 0:
        return []

    segments = []
    start_idx = 0
    for i in range(len(timestamps) - 1):
        if (timestamps[i + 1] - timestamps[i]) > gap_threshold:
            segments.append((start_idx, i + 1))
            start_idx = i + 1
    segments.append((start_idx, len(timestamps)))
    return segments


def compute_errors(
    df_est,
    df_ref,
    delta_t,
    tolerance,
    with_scale,
    segment_gap_seconds,
    min_segment_samples=1,
):
    df_ref = df_ref.drop_duplicates(subset="timestamp", keep="first").sort_values(
        "timestamp"
    )

    t_est = df_est["timestamp"].values
    p_est = df_est[["tx", "ty", "tz"]].values
    q_est = df_est[["qx", "qy", "qz", "qw"]].values

    t_ref = df_ref["timestamp"].values
    p_ref = df_ref[["tx", "ty", "tz"]].values
    q_ref = df_ref[["qx", "qy", "qz", "qw"]].values

    t_start = max(t_est.min(), t_ref.min())
    t_end = min(t_est.max(), t_ref.max())

    mask_est = (t_est >= t_start) & (t_est <= t_end)
    t_est_window = t_est[mask_est]
    p_est_window = p_est[mask_est]
    q_est_window = q_est[mask_est]

    if t_est_window.size == 0:
        raise ValueError("No overlapping timestamps between estimated and reference.")

    interp_pos = interp1d(t_ref, p_ref, axis=0, kind="linear")
    p_ref_window = interp_pos(t_est_window)

    rot_ref_obj = R.from_quat(q_ref)
    slerp = Slerp(t_ref, rot_ref_obj)
    q_ref_window = slerp(t_est_window).as_quat()
    rot_ref_window = R.from_quat(q_ref_window)

    min_segment_samples = max(1, int(min_segment_samples))
    segments_all = split_segments_by_gap(t_est_window, segment_gap_seconds)
    segments = [
        (start_idx, end_idx)
        for start_idx, end_idx in segments_all
        if (end_idx - start_idx) >= int(min_segment_samples)
    ]
    if not segments:
        raise ValueError(
            "No trajectory segments left after filtering. "
            "Lower --min-segment-samples or --segment-gap-seconds."
        )
    trimmed_segment_count = len(segments_all) - len(segments)
    trimmed_sample_count = int(
        sum(end_idx - start_idx for start_idx, end_idx in segments_all)
        - sum(end_idx - start_idx for start_idx, end_idx in segments)
    )
    trimmed_intervals = [
        (float(t_est_window[start_idx]), float(t_est_window[end_idx - 1]))
        for start_idx, end_idx in segments_all
        if (end_idx - start_idx) < min_segment_samples
    ]

    # Compact arrays so dropped segments do not leave uninitialized rows.
    kept_ranges = [np.arange(start_idx, end_idx) for start_idx, end_idx in segments]
    keep_idx = np.concatenate(kept_ranges)
    t_est_valid = t_est_window[keep_idx]
    p_est_valid = p_est_window[keep_idx]
    q_est_valid = q_est_window[keep_idx]
    p_ref_interp = p_ref_window[keep_idx]
    rot_ref_interp = rot_ref_window[keep_idx]

    # Re-index segments for compacted arrays.
    segments_compact = []
    cursor = 0
    for start_idx, end_idx in segments:
        seg_len = end_idx - start_idx
        segments_compact.append((cursor, cursor + seg_len))
        cursor += seg_len
    segments = segments_compact
    rot_est_local_all = R.from_quat(q_est_valid)
    p_est_aligned = np.zeros_like(p_est_valid)
    rot_est_final_quat = np.zeros_like(q_est_valid)
    scales = []
    rot_calibs = []
    ape_vec = np.zeros_like(p_est_valid)

    for start_idx, end_idx in segments:
        seg_slice = slice(start_idx, end_idx)
        p_ref_seg = p_ref_interp[seg_slice]
        p_est_seg = p_est_valid[seg_slice]
        q_est_seg = q_est_valid[seg_slice]
        rot_ref_seg = rot_ref_interp[seg_slice]

        rot_est_local = R.from_quat(q_est_seg)
        s, R_global, t_global = align_umeyama(
            p_ref_seg, p_est_seg, with_scale=with_scale
        )
        p_est_aligned[seg_slice] = s * (p_est_seg @ R_global.T) + t_global

        rot_est_global = R.from_matrix(R_global) * rot_est_local
        rot_diff = rot_est_global.inv() * rot_ref_seg
        q_diff = rot_diff.as_quat()
        M = np.dot(q_diff.T, q_diff)
        _, eigvecs = np.linalg.eigh(M)
        q_mean = eigvecs[:, -1]
        rot_calib = R.from_quat(q_mean)
        rot_est_final_seg = rot_est_global * rot_calib
        rot_est_final_quat[seg_slice] = rot_est_final_seg.as_quat()
        ape_vec[seg_slice] = p_ref_seg - p_est_aligned[seg_slice]

        scales.append(s)
        rot_calibs.append(rot_calib)

    rot_est_final = R.from_quat(rot_est_final_quat)

    ape_trans = np.linalg.norm(p_ref_interp - p_est_aligned, axis=1)
    ape_rot_deg = (rot_ref_interp.inv() * rot_est_final).magnitude() * (180 / np.pi)

    ref_dist = np.zeros_like(t_est_valid, dtype=float)
    if len(p_ref_interp) > 1:
        ref_dist[1:] = np.cumsum(np.linalg.norm(np.diff(p_ref_interp, axis=0), axis=1))

    idx_next = np.searchsorted(t_est_valid, t_est_valid + delta_t)
    rpe_trans_list = []
    rpe_rot_list = []
    rpe_times_list = []
    rpe_vec_list = []
    rpe_scale_list = []

    for i, j in enumerate(idx_next):
        if j < len(t_est_valid):
            if abs(t_est_valid[j] - t_est_valid[i] - delta_t) < tolerance:
                R_gt_i = rot_ref_interp[i]
                R_gt_j = rot_ref_interp[j]
                p_gt_i = p_ref_interp[i]
                p_gt_j = p_ref_interp[j]

                t_rel_gt = R_gt_i.inv().apply(p_gt_j - p_gt_i)
                R_rel_gt = R_gt_i.inv() * R_gt_j

                R_est_i = rot_est_final[i]
                R_est_j = rot_est_final[j]
                p_est_i = p_est_aligned[i]
                p_est_j = p_est_aligned[j]

                t_rel_est = R_est_i.inv().apply(p_est_j - p_est_i)
                R_rel_est = R_est_i.inv() * R_est_j

                err_t = np.linalg.norm(t_rel_gt - t_rel_est)
                err_R = (R_rel_gt.inv() * R_rel_est).magnitude()

                rpe_trans_list.append(err_t)
                rpe_rot_list.append(err_R)
                rpe_times_list.append(t_est_valid[i])
                rpe_vec_list.append(t_rel_gt - t_rel_est)

                R_est_i_raw = rot_est_local_all[i]
                R_est_j_raw = rot_est_local_all[j]
                t_rel_est_raw = R_est_i_raw.inv().apply(p_est_valid[j] - p_est_valid[i])
                est_norm = np.linalg.norm(t_rel_est_raw)
                gt_norm = np.linalg.norm(t_rel_gt)
                if est_norm > 0:
                    rpe_scale_list.append(gt_norm / est_norm)

    rpe_trans = np.array(rpe_trans_list)
    rpe_rot_deg = np.degrees(np.array(rpe_rot_list))
    rpe_times = np.array(rpe_times_list)
    rpe_vec = np.array(rpe_vec_list) if rpe_vec_list else np.zeros((0, 3))
    rpe_scale = np.array(rpe_scale_list)

    return {
        "t_est_valid": t_est_valid,
        "t_ref_full": t_ref,
        "p_ref_full": p_ref,
        "p_ref_interp": p_ref_interp,
        "p_est_aligned": p_est_aligned,
        "q_est_aligned": rot_est_final_quat,
        "ape_trans": ape_trans,
        "ape_rot_deg": ape_rot_deg,
        "ape_vec": ape_vec,
        "rpe_trans": rpe_trans,
        "rpe_rot_deg": rpe_rot_deg,
        "rpe_times": rpe_times,
        "rpe_vec": rpe_vec,
        "rpe_scale": rpe_scale,
        "segments": segments,
        "ref_dist": ref_dist,
        "segment_scales": scales,
        "segments_total": len(segments_all),
        "segments_kept": len(segments),
        "segments_trimmed": trimmed_segment_count,
        "samples_trimmed": trimmed_sample_count,
        "trimmed_segment_intervals": trimmed_intervals,
        "scale": np.mean(scales) if scales else 1.0,
        "rot_calib": rot_calibs[0] if rot_calibs else R.identity(),
        "delta_t": delta_t,
    }
