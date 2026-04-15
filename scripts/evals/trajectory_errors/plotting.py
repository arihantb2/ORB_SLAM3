import os

import matplotlib.pyplot as plt
import numpy as np

from plot_style import apply_paper_style


def plot_errors(results, alignment_on, output_dir=None, show=True):
    apply_paper_style()

    t_est_valid = results["t_est_valid"]
    p_ref_full = results["p_ref_full"]
    p_ref_interp = results["p_ref_interp"]
    p_est_aligned = results["p_est_aligned"]
    ape_trans = results["ape_trans"]
    ape_vec = results["ape_vec"]
    rpe_trans = results["rpe_trans"]
    rpe_times = results["rpe_times"]
    rpe_vec = results["rpe_vec"]
    rpe_scale = results["rpe_scale"]
    delta_t = results["delta_t"]
    segments = results["segments"]
    ref_dist = results["ref_dist"]

    seg_colors = [plt.get_cmap("tab10")(i % 10) for i in range(len(segments))]

    # -------------------------------------------------------------------------
    # APE errors
    # -------------------------------------------------------------------------
    fig_ape, axs_ape = plt.subplots(2, 1, figsize=(12, 7))

    for start_idx, end_idx in segments:
        t_start = t_est_valid[start_idx]
        t_end = t_est_valid[end_idx - 1]
        for ax in axs_ape:
            ax.axvline(t_start, color="k", linestyle="--", alpha=0.15, linewidth=0.8)
            ax.axvline(t_end, color="k", linestyle="--", alpha=0.15, linewidth=0.8)

    rms_ape_t = np.sqrt(np.mean(ape_trans**2))
    mean_ape_t = np.mean(ape_trans)
    med_ape_t = np.median(ape_trans)
    max_ape_t = np.max(ape_trans)

    labels_done = False
    for idx, (start_idx, end_idx) in enumerate(segments):
        seg_slice = slice(start_idx, end_idx)
        seg_ape_vec = ape_vec[seg_slice]
        if alignment_on == "trajectory_start" and seg_ape_vec.size > 0:
            seg_ape_vec = seg_ape_vec - seg_ape_vec[0]
        seg_ape = np.linalg.norm(seg_ape_vec, axis=1)

        axs_ape[0].plot(
            t_est_valid[seg_slice],
            seg_ape,
            color=seg_colors[idx],
            alpha=0.8,
            label="APE" if not labels_done else None,
        )
        axs_ape[1].plot(
            t_est_valid[seg_slice],
            seg_ape_vec[:, 0],
            color="C0",
            label="X" if not labels_done else None,
        )
        axs_ape[1].plot(
            t_est_valid[seg_slice],
            seg_ape_vec[:, 1],
            color="C1",
            label="Y" if not labels_done else None,
        )
        axs_ape[1].plot(
            t_est_valid[seg_slice],
            seg_ape_vec[:, 2],
            color="C2",
            label="Z" if not labels_done else None,
        )
        axs_ape[1].plot(
            t_est_valid[seg_slice],
            seg_ape,
            color="0.5",
            linestyle="--",
            linewidth=1.0,
            alpha=0.6,
            label="|Total|" if not labels_done else None,
        )
        labels_done = True

    axs_ape[0].set_title("APE Translation")
    axs_ape[0].set_xlabel("Time  (s)")
    axs_ape[0].set_ylabel("Error  (m)")
    axs_ape[0].set_ylim(bottom=0)
    axs_ape[0].legend(loc="upper left")
    stats_text = (
        f"RMSE  = {rms_ape_t:.3f} m\n"
        f"Mean  = {mean_ape_t:.3f} m\n"
        f"Median = {med_ape_t:.3f} m\n"
        f"Max   = {max_ape_t:.3f} m"
    )
    axs_ape[0].text(
        0.99, 0.97, stats_text,
        transform=axs_ape[0].transAxes,
        va="top", ha="right", fontsize=8.5,
        fontfamily="monospace",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.85),
    )

    axs_ape[1].set_title("APE Translation — Components")
    axs_ape[1].set_xlabel("Time  (s)")
    axs_ape[1].set_ylabel("Error  (m)")
    axs_ape[1].legend(loc="upper left")

    fig_ape.tight_layout()

    # -------------------------------------------------------------------------
    # RPE errors
    # -------------------------------------------------------------------------
    fig_rpe, axs_rpe = plt.subplots(2, 1, figsize=(12, 7))

    for start_idx, end_idx in segments:
        t_start = t_est_valid[start_idx]
        t_end = t_est_valid[end_idx - 1]
        for ax in axs_rpe:
            ax.axvline(t_start, color="k", linestyle="--", alpha=0.15, linewidth=0.8)
            ax.axvline(t_end, color="k", linestyle="--", alpha=0.15, linewidth=0.8)

    if rpe_trans.size > 0:
        rms_rpe_t = np.sqrt(np.mean(rpe_trans**2))
        mean_rpe_t = np.mean(rpe_trans)
        med_rpe_t = np.median(rpe_trans)
        max_rpe_t = np.max(rpe_trans)

        labels_done = False
        for idx, (start_idx, end_idx) in enumerate(segments):
            t_start = t_est_valid[start_idx]
            t_end = t_est_valid[end_idx - 1]
            seg_mask = (rpe_times >= t_start) & (rpe_times <= t_end)
            seg_rpe_vec = rpe_vec[seg_mask]
            if alignment_on == "trajectory_start" and seg_rpe_vec.size > 0:
                seg_rpe_vec = seg_rpe_vec - seg_rpe_vec[0]
            seg_rpe = np.linalg.norm(seg_rpe_vec, axis=1)

            axs_rpe[0].plot(
                rpe_times[seg_mask],
                seg_rpe,
                color=seg_colors[idx],
                alpha=0.8,
                label="RPE" if not labels_done else None,
            )
            if seg_rpe_vec.size > 0:
                axs_rpe[1].plot(
                    rpe_times[seg_mask],
                    seg_rpe_vec[:, 0],
                    color="C0",
                    label="X" if not labels_done else None,
                )
                axs_rpe[1].plot(
                    rpe_times[seg_mask],
                    seg_rpe_vec[:, 1],
                    color="C1",
                    label="Y" if not labels_done else None,
                )
                axs_rpe[1].plot(
                    rpe_times[seg_mask],
                    seg_rpe_vec[:, 2],
                    color="C2",
                    label="Z" if not labels_done else None,
                )
                axs_rpe[1].plot(
                    rpe_times[seg_mask],
                    seg_rpe,
                    color="0.5",
                    linestyle="--",
                    linewidth=1.0,
                    alpha=0.6,
                    label="|Total|" if not labels_done else None,
                )
            labels_done = True

        axs_rpe[0].set_title(f"RPE Translation  (\u0394t = {delta_t} s)")
        axs_rpe[0].set_xlabel("Time  (s)")
        axs_rpe[0].set_ylabel("Error  (m)")
        axs_rpe[0].set_ylim(bottom=0)
        axs_rpe[0].legend(loc="upper left")
        rpe_stats_text = (
            f"RMSE  = {rms_rpe_t:.3f} m\n"
            f"Mean  = {mean_rpe_t:.3f} m\n"
            f"Median = {med_rpe_t:.3f} m\n"
            f"Max   = {max_rpe_t:.3f} m"
        )
        axs_rpe[0].text(
            0.99, 0.97, rpe_stats_text,
            transform=axs_rpe[0].transAxes,
            va="top", ha="right", fontsize=8.5,
            fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.85),
        )

        axs_rpe[1].set_title("RPE Translation — Components")
        axs_rpe[1].set_xlabel("Time  (s)")
        axs_rpe[1].set_ylabel("Error  (m)")
        axs_rpe[1].legend(loc="upper left")

    fig_rpe.tight_layout()

    # -------------------------------------------------------------------------
    # Trajectory position array (aligned to reference start per segment)
    # -------------------------------------------------------------------------
    p_est_plot = p_est_aligned.copy()
    if alignment_on == "trajectory_start":
        for start_idx, end_idx in segments:
            seg_slice = slice(start_idx, end_idx)
            shift = p_ref_interp[start_idx] - p_est_plot[start_idx]
            p_est_plot[seg_slice] = p_est_plot[seg_slice] + shift

    # -------------------------------------------------------------------------
    # 3D trajectory
    # -------------------------------------------------------------------------
    fig_traj = plt.figure(figsize=(10, 8))
    ax_traj = fig_traj.add_subplot(111, projection="3d")

    ax_traj.plot(
        p_ref_full[:, 0], p_ref_full[:, 1], p_ref_full[:, 2],
        color="C1", linestyle="--", linewidth=1.2, alpha=0.6, label="Reference",
    )
    first_seg = True
    for start_idx, end_idx in segments:
        seg_slice = slice(start_idx, end_idx)
        seg_est = p_est_plot[seg_slice]
        ax_traj.plot(
            seg_est[:, 0], seg_est[:, 1], seg_est[:, 2],
            color="C0", linewidth=1.8,
            label="Estimated" if first_seg else None,
        )
        first_seg = False

    # Start / end markers on estimated trajectory
    p0 = p_est_plot[segments[0][0]]
    p1 = p_est_plot[segments[-1][1] - 1]
    ax_traj.scatter(*p0, marker="^", color="green", s=60, zorder=5, label="Start")
    ax_traj.scatter(*p1, marker="s", color="red",   s=60, zorder=5, label="End")

    ax_traj.set_title("Trajectory — 3D Overlay")
    ax_traj.set_xlabel("X  (m)")
    ax_traj.set_ylabel("Y  (m)")
    ax_traj.set_zlabel("Z  (m)")
    ax_traj.legend()
    ax_traj.invert_yaxis()
    ax_traj.invert_zaxis()

    # -------------------------------------------------------------------------
    # Spatial APE distribution (XY)
    # -------------------------------------------------------------------------
    fig_planes, ax_ape_map = plt.subplots(1, 1, figsize=(8, 7))

    sc_ape = ax_ape_map.scatter(
        p_est_plot[:, 0], p_est_plot[:, 1],
        c=ape_trans, cmap="plasma", s=6, alpha=0.8, linewidths=0,
    )
    fig_planes.colorbar(sc_ape, ax=ax_ape_map, label="APE  (m)", shrink=0.85)
    ax_ape_map.set_title("Spatial APE Distribution  (XY)")
    ax_ape_map.set_xlabel("X  (m)")
    ax_ape_map.set_ylabel("Y  (m)")
    ax_ape_map.set_aspect("equal", adjustable="box")

    fig_planes.tight_layout()

    # -------------------------------------------------------------------------
    # Cumulative drift vs distance
    # -------------------------------------------------------------------------
    fig_drift, ax_drift = plt.subplots(1, 1, figsize=(10, 5))

    for idx, (start_idx, end_idx) in enumerate(segments):
        seg_slice = slice(start_idx, end_idx)
        seg_ape_vec = ape_vec[seg_slice]
        if alignment_on == "trajectory_start" and seg_ape_vec.size > 0:
            seg_ape_vec = seg_ape_vec - seg_ape_vec[0]
        seg_ape = np.linalg.norm(seg_ape_vec, axis=1)
        ax_drift.plot(
            ref_dist[seg_slice], seg_ape,
            color=seg_colors[idx],
            label="APE drift" if idx == 0 else None,
        )
        # Annotate final drift % at end of each segment
        if len(ref_dist[seg_slice]) > 0 and ref_dist[seg_slice][-1] > 0:
            final_dist = ref_dist[seg_slice][-1]
            final_ape = seg_ape[-1]
            pct = 100.0 * final_ape / final_dist
            ax_drift.annotate(
                f"{pct:.1f}%",
                xy=(final_dist, final_ape),
                xytext=(4, 2), textcoords="offset points",
                fontsize=8, color=seg_colors[idx],
            )

    ax_drift.set_title("Cumulative Drift vs. Distance")
    ax_drift.set_xlabel("Distance travelled  (m)")
    ax_drift.set_ylabel("APE translation  (m)")
    ax_drift.legend()
    fig_drift.tight_layout()

    # -------------------------------------------------------------------------
    # Scale drift
    # -------------------------------------------------------------------------
    fig_scale, ax_scale = plt.subplots(1, 1, figsize=(10, 5))

    if rpe_scale.size > 0:
        labels_done = False
        for idx, (start_idx, end_idx) in enumerate(segments):
            t_start = t_est_valid[start_idx]
            t_end = t_est_valid[end_idx - 1]
            seg_mask = (rpe_times >= t_start) & (rpe_times <= t_end)
            ax_scale.plot(
                rpe_times[seg_mask], rpe_scale[seg_mask],
                color=seg_colors[idx],
                label="Scale" if not labels_done else None,
            )
            labels_done = True

    ax_scale.axhline(1.0, color="k", linestyle=":", linewidth=1.0, label="Scale = 1.0")
    ax_scale.set_title("Scale Drift Over Time  (RPE)")
    ax_scale.set_xlabel("Time  (s)")
    ax_scale.set_ylabel("Scale factor  (est / ref)")
    ax_scale.legend()
    fig_scale.tight_layout()

    # -------------------------------------------------------------------------
    # APE CDF
    # -------------------------------------------------------------------------
    fig_cdf, ax_cdf = plt.subplots(1, 1, figsize=(7, 5))

    sorted_ape = np.sort(ape_trans)
    cdf = np.arange(1, len(sorted_ape) + 1) / len(sorted_ape)
    ax_cdf.plot(sorted_ape, cdf, color="C0", linewidth=2.0)

    for pct, label in [(50, "50th"), (75, "75th"), (90, "90th"), (95, "95th")]:
        val = np.percentile(sorted_ape, pct)
        ax_cdf.axvline(val, linestyle="--", linewidth=0.9, alpha=0.6, color="0.4")
        ax_cdf.text(
            val, pct / 100.0,
            f" {label}: {val:.3f} m",
            fontsize=8, va="bottom", color="0.3",
        )

    ax_cdf.set_title("APE Translation — Cumulative Distribution")
    ax_cdf.set_xlabel("APE translation  (m)")
    ax_cdf.set_ylabel("Cumulative fraction of poses")
    ax_cdf.set_xlim(left=0)
    ax_cdf.set_ylim(0, 1)
    ax_cdf.text(
        0.99, 0.02,
        f"RMSE = {rms_ape_t:.3f} m",
        transform=ax_cdf.transAxes,
        va="bottom", ha="right", fontsize=9,
        fontfamily="monospace",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.85),
    )
    fig_cdf.tight_layout()

    # -------------------------------------------------------------------------
    # Save / show
    # -------------------------------------------------------------------------
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        fig_ape.savefig(os.path.join(output_dir, "ape_errors.png"))
        fig_rpe.savefig(os.path.join(output_dir, "rpe_errors.png"))
        fig_planes.savefig(os.path.join(output_dir, "trajectory_2d_planes.png"))
        fig_drift.savefig(os.path.join(output_dir, "drift_vs_distance.png"))
        fig_scale.savefig(os.path.join(output_dir, "scale_drift.png"))
        fig_cdf.savefig(os.path.join(output_dir, "ape_cdf.png"))
    if show:
        plt.show()
