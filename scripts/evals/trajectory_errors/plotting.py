import os

import matplotlib.pyplot as plt
import numpy as np


def plot_errors(results, alignment_on, output_dir=None, show=True):
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

    fig_ape, axs_ape = plt.subplots(2, 1, figsize=(30, 10))
    for start_idx, end_idx in segments:
        t_start = t_est_valid[start_idx]
        t_end = t_est_valid[end_idx - 1]
        for ax in axs_ape:
            ax.axvline(t_start, color="k", linestyle="--", alpha=0.2)
            ax.axvline(t_end, color="k", linestyle="--", alpha=0.2)

    rms_ape_t = np.sqrt(np.mean(ape_trans**2))
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
            alpha=0.7,
            label="Total" if not labels_done else None,
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
            color="C3",
            linestyle="--",
            alpha=0.2,
            label="Total envelope" if not labels_done else None,
        )
        axs_ape[1].plot(
            t_est_valid[seg_slice],
            -seg_ape,
            color="C3",
            linestyle="--",
            alpha=0.2,
        )
        axs_ape[1].fill_between(
            t_est_valid[seg_slice],
            -seg_ape,
            seg_ape,
            color="0.5",
            alpha=0.2,
            linewidth=0,
        )
        labels_done = True
    axs_ape[0].set_title(f"APE Translation (RMSE: {rms_ape_t:.2f} m)")
    axs_ape[0].set_xlabel("Timestamp (s)")
    axs_ape[0].set_ylabel("Error (m)")
    axs_ape[0].set_ylim(bottom=0)
    axs_ape[0].grid(True, axis="y")
    axs_ape[0].legend()
    axs_ape[1].set_title("APE Translation Components")
    axs_ape[1].set_xlabel("Timestamp (s)")
    axs_ape[1].set_ylabel("Error (m)")
    axs_ape[1].grid(True, axis="y")
    axs_ape[1].legend()

    fig_rpe, axs_rpe = plt.subplots(2, 1, figsize=(30, 10))
    for idx, (start_idx, end_idx) in enumerate(segments):
        t_start = t_est_valid[start_idx]
        t_end = t_est_valid[end_idx - 1]
        for ax in axs_rpe:
            ax.axvline(t_start, color="k", linestyle="--", alpha=0.2)
            ax.axvline(t_end, color="k", linestyle="--", alpha=0.2)
    if rpe_trans.size > 0:
        rms_rpe_t = np.sqrt(np.mean(rpe_trans**2))
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
                alpha=0.7,
                label="Total" if not labels_done else None,
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
                    color="C3",
                    linestyle="--",
                    alpha=0.2,
                    label="Total envelope" if not labels_done else None,
                )
                axs_rpe[1].plot(
                    rpe_times[seg_mask],
                    -seg_rpe,
                    color="C3",
                    linestyle="--",
                    alpha=0.2,
                )
                axs_rpe[1].fill_between(
                    rpe_times[seg_mask],
                    -seg_rpe,
                    seg_rpe,
                    color="0.5",
                    alpha=0.2,
                    linewidth=0,
                )
            labels_done = True
        axs_rpe[0].set_title(
            f"RPE Translation (dt={delta_t}s, RMSE: {rms_rpe_t:.2f} m)"
        )
        axs_rpe[0].set_xlabel("Timestamp (s)")
        axs_rpe[0].set_ylabel("Error (m)")
        axs_rpe[0].set_ylim(bottom=0)
        axs_rpe[0].grid(True, axis="y")
        axs_rpe[0].legend()
        axs_rpe[1].set_title("RPE Translation Components")
        axs_rpe[1].set_xlabel("Timestamp (s)")
        axs_rpe[1].set_ylabel("Error (m)")
        axs_rpe[1].grid(True, axis="y")
        axs_rpe[1].legend()

    fig_ape.tight_layout()
    fig_rpe.tight_layout()

    p_est_plot = p_est_aligned.copy()
    if alignment_on == "trajectory_start":
        for start_idx, end_idx in segments:
            seg_slice = slice(start_idx, end_idx)
            shift = p_ref_interp[start_idx] - p_est_plot[start_idx]
            p_est_plot[seg_slice] = p_est_plot[seg_slice] + shift

    time_norm = plt.Normalize(vmin=t_est_valid.min(), vmax=t_est_valid.max())

    fig_traj = plt.figure(figsize=(12, 10))
    ax_traj = fig_traj.add_subplot(111, projection="3d")
    ax_traj.plot(
        p_ref_full[:, 0],
        p_ref_full[:, 1],
        p_ref_full[:, 2],
        color="C1",
        linestyle="--",
        alpha=0.5,
        label="Ref trajectory",
    )
    for start_idx, end_idx in segments:
        seg_slice = slice(start_idx, end_idx)
        seg_est = p_est_plot[seg_slice]
        ax_traj.plot(
            seg_est[:, 0],
            seg_est[:, 1],
            seg_est[:, 2],
            color="C0",
            linewidth=2.5,
            label="Test trajectory" if start_idx == segments[0][0] else None,
        )
    est_scatter = ax_traj.scatter(
        p_est_plot[:, 0],
        p_est_plot[:, 1],
        p_est_plot[:, 2],
        c=t_est_valid,
        cmap="viridis",
        norm=time_norm,
        s=8,
        alpha=0.7,
        label="Test time",
    )
    fig_traj.colorbar(est_scatter, ax=ax_traj, pad=0.1, label="Test time (s)")
    ax_traj.set_title("3D Trajectory Overlay")
    ax_traj.set_xlabel("X (m)")
    ax_traj.set_ylabel("Y (m)")
    ax_traj.set_zlabel("Z (m)")
    ax_traj.legend()

    # Invert y and z axes NED
    ax_traj.invert_yaxis()
    ax_traj.invert_zaxis()

    fig_planes, axs_planes = plt.subplots(2, 2, figsize=(30, 16))
    ax_xy = axs_planes[0, 0]
    ax_zy = axs_planes[0, 1]
    ax_xz = axs_planes[1, 0]
    axs_planes[1, 1].remove()
    ax_3d = fig_planes.add_subplot(2, 2, 4, projection="3d")
    plane_defs = [
        (ax_xy, "XY", 0, 1),
        (ax_zy, "ZY", 2, 1),
        (ax_xz, "XZ", 0, 2),
    ]
    plotted_points = []
    for ax, label, xi, yi in plane_defs:
        ax.plot(
            p_ref_full[:, xi],
            p_ref_full[:, yi],
            color="C1",
            linestyle="--",
            alpha=0.5,
            label="Ref trajectory",
        )
        plotted_points.append(p_ref_full)
        for start_idx, end_idx in segments:
            seg_slice = slice(start_idx, end_idx)
            seg_est = p_est_plot[seg_slice]
            plotted_points.append(seg_est)
            ax.plot(
                seg_est[:, xi],
                seg_est[:, yi],
                color="C0",
                linewidth=2.5,
                label="Test trajectory" if start_idx == segments[0][0] else None,
            )
        ax.set_title(f"{label} Overlay")
        ax.set_xlabel(f"{label[0]} (m)")
        ax.set_ylabel(f"{label[1]} (m)")
        ax.set_aspect("equal", adjustable="box")
        ax.legend()
    all_points = np.vstack(plotted_points) if plotted_points else p_ref_interp
    x_min, y_min, z_min = np.min(all_points, axis=0)
    x_max, y_max, z_max = np.max(all_points, axis=0)
    ax_xy.set_xlim(x_min, x_max)
    ax_xz.set_xlim(x_min, x_max)
    ax_xy.set_ylim(y_min, y_max)
    ax_zy.set_ylim(y_min, y_max)
    ax_xz.set_ylim(z_min, z_max)
    ax_zy.set_xlim(z_min, z_max)

    ax_3d.plot(
        p_ref_full[:, 0],
        p_ref_full[:, 1],
        p_ref_full[:, 2],
        color="C1",
        linestyle="--",
        alpha=0.5,
        label="Ref trajectory",
    )
    for start_idx, end_idx in segments:
        seg_slice = slice(start_idx, end_idx)
        seg_est = p_est_plot[seg_slice]
        ax_3d.plot(
            seg_est[:, 0],
            seg_est[:, 1],
            seg_est[:, 2],
            color="C0",
            linewidth=2.5,
            label="Test trajectory" if start_idx == segments[0][0] else None,
        )
    planes_scatter = ax_3d.scatter(
        p_est_plot[:, 0],
        p_est_plot[:, 1],
        p_est_plot[:, 2],
        c=t_est_valid,
        cmap="viridis",
        norm=time_norm,
        s=8,
        alpha=0.7,
        label="Test time",
    )
    fig_planes.colorbar(planes_scatter, ax=ax_3d, pad=0.1, label="Test time (s)")
    ax_3d.set_title("3D Overlay")
    ax_3d.set_xlabel("X (m)")
    ax_3d.set_ylabel("Y (m)")
    ax_3d.set_zlabel("Z (m)")
    ax_3d.set_box_aspect((1, 1, 1))
    ax_3d.legend()

    # Invert y and z axes NED
    ax_3d.invert_yaxis()
    ax_3d.invert_zaxis()

    fig_drift, ax_drift = plt.subplots(1, 1, figsize=(12, 6))
    for idx, (start_idx, end_idx) in enumerate(segments):
        seg_slice = slice(start_idx, end_idx)
        seg_ape_vec = ape_vec[seg_slice]
        if alignment_on == "trajectory_start" and seg_ape_vec.size > 0:
            seg_ape_vec = seg_ape_vec - seg_ape_vec[0]
        seg_ape = np.linalg.norm(seg_ape_vec, axis=1)
        ax_drift.plot(
            ref_dist[seg_slice],
            seg_ape,
            color=seg_colors[idx],
            label="APE drift" if start_idx == segments[0][0] else None,
        )
    ax_drift.set_title("Cumulative Drift vs Distance (Ref)")
    ax_drift.set_xlabel("Distance traveled (m)")
    ax_drift.set_ylabel("Error (m)")
    ax_drift.grid(True, axis="y")
    ax_drift.legend()

    fig_scale, ax_scale = plt.subplots(1, 1, figsize=(12, 6))
    if rpe_scale.size > 0:
        labels_done = False
        for idx, (start_idx, end_idx) in enumerate(segments):
            t_start = t_est_valid[start_idx]
            t_end = t_est_valid[end_idx - 1]
            seg_mask = (rpe_times >= t_start) & (rpe_times <= t_end)
            ax_scale.plot(
                rpe_times[seg_mask],
                rpe_scale[seg_mask],
                color=seg_colors[idx],
                label="Scale" if not labels_done else None,
            )
            labels_done = True
    ax_scale.set_title("Scale Drift Over Time (RPE)")
    ax_scale.set_xlabel("Timestamp (s)")
    ax_scale.set_ylabel("Scale")
    ax_scale.grid(True, axis="y")
    ax_scale.legend()
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        fig_ape.savefig(os.path.join(output_dir, "ape_errors.png"), dpi=150)
        fig_rpe.savefig(os.path.join(output_dir, "rpe_errors.png"), dpi=150)
        fig_traj.savefig(os.path.join(output_dir, "trajectory_3d.png"), dpi=150)
        fig_planes.savefig(os.path.join(output_dir, "trajectory_2d_planes.png"), dpi=150)
        fig_drift.savefig(os.path.join(output_dir, "drift_vs_distance.png"), dpi=150)
        fig_scale.savefig(os.path.join(output_dir, "scale_drift.png"), dpi=150)
    if show:
        plt.show()
