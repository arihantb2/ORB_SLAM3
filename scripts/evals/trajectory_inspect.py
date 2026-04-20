"""Inspect pose CSV/XML files: 3D trajectory, motion distributions, time series.

Each CSV must have columns: timestamp, tx, ty, tz, qx, qy, qz, qw
XML files must be Metashape-style (see --ref-group-id).

Generates three figures (saved when -o is given, shown when --show is passed):
  trajectory_2d.png       — best-fit 2D projection of trajectory (PCA)
  motion_distribution.png — 6-panel distribution overview (2 rows × 3 cols)
  motion_timeseries.png   — speed and angular-rate profiles (2 rows × 1 col)

Usage
-----
  # single CSV
  python pose_inspect.py stereo_pose_est.csv -o /tmp/figs --show

  # overlay two files with custom labels
  python pose_inspect.py stereo_pose_est.csv vehicle_pose_est.csv \\
      --labels "Camera" "Vehicle" -o /tmp/figs --show
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.gridspec import GridSpec
from scipy.spatial.transform import Rotation
from scipy.stats import gaussian_kde

import plot_style
from trajectory_evals.io import load_csv, load_xml, load_pose_file as _load_pose_file


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def load_pose_file(path: str, label: str, group_id: int = 0) -> pd.DataFrame:
    try:
        return _load_pose_file(path, label, group_id=group_id)
    except ValueError as e:
        sys.exit(f"ERROR: {e}")


# ---------------------------------------------------------------------------
# Motion statistics
# ---------------------------------------------------------------------------

def compute_motion_stats(df: pd.DataFrame) -> dict:
    pos = df[["tx", "ty", "tz"]].to_numpy()
    quats = df[["qx", "qy", "qz", "qw"]].to_numpy()   # scipy: x,y,z,w order
    timestamps = df["timestamp"].to_numpy()

    dpos = np.diff(pos, axis=0)
    dt = np.diff(timestamps)
    dt = np.where(dt > 0, dt, np.nan)   # guard duplicate timestamps

    step = np.linalg.norm(dpos, axis=1)
    speed = step / dt

    rots = Rotation.from_quat(quats)
    r_delta = rots[:-1].inv() * rots[1:]
    rot_step = np.linalg.norm(r_delta.as_rotvec(), axis=1) * (180.0 / np.pi)
    ang_rate = rot_step / dt

    euler = r_delta.as_euler("ZYX", degrees=True)   # [yaw, pitch, roll]
    t_rel = timestamps[1:] - timestamps[0]

    return dict(
        dx=dpos[:, 0], dy=dpos[:, 1], dz=dpos[:, 2],
        step=step, speed=speed,
        droll=euler[:, 2], dpitch=euler[:, 1], dyaw=euler[:, 0],
        rot_step=rot_step, ang_rate=ang_rate,
        t_rel=t_rel,
    )


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _hist_kde(ax, data_list, labels, colors, xlabel, title):
    finite_all = [d[np.isfinite(d)] for d in data_list]
    lo = min(d.min() for d in finite_all)
    hi = max(d.max() for d in finite_all)
    bins = np.linspace(lo, hi, 60)
    for data, label, color in zip(finite_all, labels, colors):
        ax.hist(data, bins=bins, density=True, alpha=0.35, color=color)
        kde = gaussian_kde(data, bw_method="scott")
        x = np.linspace(lo, hi, 400)
        ax.plot(x, kde(x), color=color, label=label)
        ax.axvline(np.median(data), color=color, linestyle="--", linewidth=1.0, alpha=0.8)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Density")
    ax.set_title(title)
    ax.legend()


def _time_series(ax, t_list, y_list, labels, colors, ylabel, title):
    for t, y, label, color in zip(t_list, y_list, labels, colors):
        finite = np.isfinite(y)
        ax.plot(t[finite], y[finite], color=color, alpha=0.8, linewidth=0.8, label=label)
    ax.set_xlabel("Time (s, relative)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()


def _violins(ax, data_lists, labels, axis_labels, colors, ylabel, title):
    from matplotlib.patches import Patch
    n_axes = len(axis_labels)
    n_files = len(data_lists)
    group_width = 0.8
    offsets = np.linspace(-group_width / 2, group_width / 2, n_files) if n_files > 1 else [0.0]
    for fi, (dataset, color) in enumerate(zip(data_lists, colors)):
        positions = np.arange(n_axes) + offsets[fi]
        parts = ax.violinplot(
            [d[np.isfinite(d)] for d in dataset],
            positions=positions,
            widths=group_width / max(n_files, 1) * 0.85,
            showmedians=True,
            showextrema=False,
        )
        for pc in parts["bodies"]:
            pc.set_facecolor(color)
            pc.set_alpha(0.55)
        parts["cmedians"].set_color(color)
        parts["cmedians"].set_linewidth(1.5)
    ax.axhline(0, color="black", linewidth=0.7, linestyle="--", alpha=0.5)
    ax.set_xticks(np.arange(n_axes))
    ax.set_xticklabels(axis_labels)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(handles=[
        Patch(facecolor=colors[i], alpha=0.6, label=labels[i]) for i in range(len(labels))
    ])


def _scatter_density(ax, dx_list, dy_list, labels, colors, title):
    for dx, dy, label, color in zip(dx_list, dy_list, labels, colors):
        finite = np.isfinite(dx) & np.isfinite(dy)
        x, y = dx[finite], dy[finite]
        ax.scatter(x, y, s=2, alpha=0.2, color=color, label=label, rasterized=True)
        try:
            xy = np.vstack([x, y])
            kde = gaussian_kde(xy)
            xg = np.linspace(x.min(), x.max(), 80)
            yg = np.linspace(y.min(), y.max(), 80)
            Xg, Yg = np.meshgrid(xg, yg)
            Z = kde(np.vstack([Xg.ravel(), Yg.ravel()])).reshape(Xg.shape)
            ax.contour(Xg, Yg, Z, levels=5, colors=[color], alpha=0.7, linewidths=0.8)
        except Exception:
            pass
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("Δx (m)")
    ax.set_ylabel("Δy (m)")
    ax.set_title(title)
    ax.legend(markerscale=4)


def _rose(ax, dyaw_list, labels, colors, title):
    bin_edges = np.linspace(-np.pi, np.pi, 37)   # 36 bins × 10°
    width = 2 * np.pi / 36
    for dyaw, label, color in zip(dyaw_list, labels, colors):
        finite = dyaw[np.isfinite(dyaw)]
        rad = np.deg2rad(finite)
        counts, _ = np.histogram(rad, bins=bin_edges)
        theta = 0.5 * (bin_edges[:-1] + bin_edges[1:])
        ax.bar(theta, counts, width=width * 0.9, alpha=0.55, color=color,
               label=label, align="center")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_xlabel("Count", labelpad=20)
    ax.set_title(title, pad=14)
    ax.legend(loc="lower left", bbox_to_anchor=(-0.25, -0.15))


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------

def _pca_project(dfs: list):
    """Return (proj_list, pc_labels) using PCA on the union of all positions."""
    all_pts = np.vstack([df[["tx", "ty", "tz"]].to_numpy() for df in dfs])
    centroid = all_pts.mean(axis=0)
    centered = all_pts - centroid
    _, S, Vt = np.linalg.svd(centered, full_matrices=False)
    var_ratio = S**2 / (S**2).sum()
    # project each trajectory onto the top-2 principal axes
    proj_list = [
        (df[["tx", "ty", "tz"]].to_numpy() - centroid) @ Vt[:2].T
        for df in dfs
    ]
    pc_labels = (
        f"PC1 ({var_ratio[0]:.1%} var) (m)",
        f"PC2 ({var_ratio[1]:.1%} var) (m)",
    )
    return proj_list, pc_labels


def plot_trajectory_2d(
    dfs: list,
    labels: list,
    output_path: str,
    show: bool,
    suptitle: str = "",
) -> None:
    colors = [plt.get_cmap("tab10")(i % 10) for i in range(len(dfs))]
    proj_list, pc_labels = _pca_project(dfs)

    fig, ax = plt.subplots(figsize=(10, 8))

    for proj, label, color in zip(proj_list, labels, colors):
        ax.plot(proj[:, 0], proj[:, 1], color=color, linewidth=1.2, label=label)
        ax.plot(proj[0, 0],  proj[0, 1],  marker="o", color=color, markersize=5)
        ax.plot(proj[-1, 0], proj[-1, 1], marker="s", color=color, markersize=5)

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel(pc_labels[0])
    ax.set_ylabel(pc_labels[1])
    ax.set_title("Trajectory — Best-Fit 2D Projection (PCA)")
    if len(dfs) > 1:
        ax.legend()

    title = suptitle or "Trajectory"
    fig.suptitle(title, fontsize=13, fontweight="bold")
    fig.tight_layout()

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    print(f"Saved: {output_path}")

    if not show:
        plt.close(fig)


def plot_motion_distribution(
    stats_list: list,
    labels: list,
    output_path: str,
    show: bool,
    suptitle: str = "",
) -> None:
    colors = [plt.get_cmap("tab10")(i % 10) for i in range(len(stats_list))]

    fig = plt.figure(figsize=(13, 8))
    gs = GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.38)

    _hist_kde(fig.add_subplot(gs[0, 0]),
              [s["step"] for s in stats_list], labels, colors,
              "Step size (m)", "Translation Step Size")
    _violins(fig.add_subplot(gs[0, 1]),
             [[s["dx"], s["dy"], s["dz"]] for s in stats_list],
             labels, ["Δx", "Δy", "Δz"], colors, "Δposition (m)", "Per-Axis Translation Δ")
    _scatter_density(fig.add_subplot(gs[0, 2]),
                     [s["dx"] for s in stats_list],
                     [s["dy"] for s in stats_list],
                     labels, colors, "XY Motion Pattern")

    _hist_kde(fig.add_subplot(gs[1, 0]),
              [s["rot_step"] for s in stats_list], labels, colors,
              "Step size (°)", "Rotation Step Size")
    _violins(fig.add_subplot(gs[1, 1]),
             [[s["droll"], s["dpitch"], s["dyaw"]] for s in stats_list],
             labels, ["Δroll", "Δpitch", "Δyaw"], colors, "Δangle (°)", "Per-Axis Rotation Δ")
    _rose(fig.add_subplot(gs[1, 2], polar=True),
          [s["dyaw"] for s in stats_list],
          labels, colors, "Yaw Change Distribution")

    title = suptitle or "Motion Distribution"
    fig.suptitle(title, fontsize=13, fontweight="bold")

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    print(f"Saved: {output_path}")

    if not show:
        plt.close(fig)


def plot_time_series(
    stats_list: list,
    labels: list,
    output_path: str,
    show: bool,
    suptitle: str = "",
) -> None:
    colors = [plt.get_cmap("tab10")(i % 10) for i in range(len(stats_list))]

    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(14, 6), sharex=False)
    fig.subplots_adjust(hspace=0.42)

    _time_series(ax_top,
                 [s["t_rel"] for s in stats_list],
                 [s["speed"] for s in stats_list],
                 labels, colors, "Speed (m/s)", "Speed Profile")
    _time_series(ax_bot,
                 [s["t_rel"] for s in stats_list],
                 [s["ang_rate"] for s in stats_list],
                 labels, colors, "Angular rate (°/s)", "Angular Rate Profile")

    title = (suptitle or "Motion Time Series").replace("Motion Distribution", "Motion Time Series")
    fig.suptitle(title, fontsize=13, fontweight="bold")

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    fig.savefig(output_path, bbox_inches="tight")
    print(f"Saved: {output_path}")

    if not show:
        plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "files", nargs="+", metavar="FILE",
        help="One or more pose files (.csv or .xml)",
    )
    parser.add_argument(
        "--labels", nargs="+", metavar="LABEL",
        help="Display label for each file (default: filename stem)",
    )
    parser.add_argument(
        "--ref-group-id", type=int, default=0, metavar="N",
        help="Metashape XML cameras group id, applied to all XML inputs (default: 0)",
    )
    parser.add_argument(
        "-o", "--output-dir", default=None, metavar="DIR",
        help="Directory to save figures (default: directory of first input file)",
    )
    parser.add_argument(
        "--title", default="", metavar="TITLE",
        help="Figure suptitle prefix (default: derived from file names)",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Display all figures interactively after saving",
    )
    args = parser.parse_args()

    if args.labels is not None and len(args.labels) != len(args.files):
        parser.error(
            f"--labels count ({len(args.labels)}) must match file count ({len(args.files)})"
        )
    if args.labels is None:
        args.labels = [os.path.splitext(os.path.basename(p))[0] for p in args.files]
    if args.output_dir is None:
        args.output_dir = os.path.dirname(os.path.abspath(args.files[0]))
    if not args.title:
        args.title = ", ".join(args.labels)

    return args


def main():
    args = parse_args()
    plot_style.apply_paper_style()

    dfs, stats_list = [], []
    for path, label in zip(args.files, args.labels):
        df = load_pose_file(path, label, group_id=args.ref_group_id)
        df = df.sort_values("timestamp").reset_index(drop=True)
        dfs.append(df)
        stats_list.append(compute_motion_stats(df))

    out = args.output_dir
    plot_trajectory_2d(dfs, args.labels,
                       os.path.join(out, "trajectory_2d.png"),
                       show=args.show, suptitle=args.title)
    plot_motion_distribution(stats_list, args.labels,
                             os.path.join(out, "motion_distribution.png"),
                             show=args.show, suptitle=args.title)
    plot_time_series(stats_list, args.labels,
                     os.path.join(out, "motion_timeseries.png"),
                     show=args.show, suptitle=args.title)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
