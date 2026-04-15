#!/usr/bin/env python3
"""Visualize camera intrinsics as a distortion warp on a regular grid.

Takes a perfectly square (undistorted) grid, pushes it through the distortion
model, and shows the resulting warp — one panel per camera for stereo configs.
Node colour encodes displacement magnitude.  Works without any actual images.

Usage:
    python cam_intrinsics.py path/to/camera.yaml
    python cam_intrinsics.py path/to/camera.yaml -o distortion.png
    python cam_intrinsics.py path/to/camera.yaml -n 20
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from stereo import load_camera_intrinsics


# ---------------------------------------------------------------------------
# Grid warp
# ---------------------------------------------------------------------------


def _warp_grid(
    K: np.ndarray,
    D: np.ndarray,
    width: int,
    height: int,
    n_cols: int,
    n_rows: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Map a regular undistorted grid to its distorted counterpart.

    Returns ``(undist_pts, dist_pts)``, each ``(n_rows, n_cols, 2)`` in pixel coords.
    """
    margin_x, margin_y = width * 0.05, height * 0.05
    xs = np.linspace(margin_x, width - margin_x, n_cols)
    ys = np.linspace(margin_y, height - margin_y, n_rows)
    gx, gy = np.meshgrid(xs, ys)
    undist_pts = np.stack([gx, gy], axis=-1).astype(np.float64)

    # Undistorted pixel → unit-depth 3-D point → projectPoints applies distortion
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    x_n = (undist_pts[..., 0] - cx) / fx
    y_n = (undist_pts[..., 1] - cy) / fy
    pts_3d = np.stack([x_n, y_n, np.ones_like(x_n)], axis=-1).reshape(-1, 1, 3)

    dist_px, _ = cv2.projectPoints(pts_3d, np.zeros(3), np.zeros(3), K, D)

    return undist_pts, dist_px.reshape(n_rows, n_cols, 2)


def _grid_segs(pts: np.ndarray) -> list[np.ndarray]:
    n_rows, n_cols = pts.shape[:2]
    return [pts[r, :, :] for r in range(n_rows)] + [pts[:, c, :] for c in range(n_cols)]


# ---------------------------------------------------------------------------
# Per-camera plot
# ---------------------------------------------------------------------------


def _plot_camera_warp(
    ax: plt.Axes,
    K: np.ndarray,
    D: np.ndarray,
    width: int,
    height: int,
    title: str,
    n_grid: int,
) -> plt.cm.ScalarMappable:
    undist_pts, dist_pts = _warp_grid(K, D, width, height, n_grid, n_grid)

    ax.add_collection(LineCollection(
        _grid_segs(undist_pts), colors="lightgray", linewidths=0.8,
        linestyles="dashed", alpha=0.7, label="Undistorted (ideal)", zorder=2,
    ))
    ax.add_collection(LineCollection(
        _grid_segs(dist_pts), colors="steelblue", linewidths=1.1,
        alpha=0.85, label="Distorted", zorder=3,
    ))

    mag = np.hypot(
        dist_pts[..., 0] - undist_pts[..., 0],
        dist_pts[..., 1] - undist_pts[..., 1],
    )
    sc = ax.scatter(
        dist_pts[..., 0].ravel(), dist_pts[..., 1].ravel(),
        c=mag.ravel(), cmap="plasma", s=12, zorder=5, linewidths=0,
    )

    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x  (px)")
    ax.set_ylabel("y  (px)")
    ax.legend(loc="lower right", fontsize=7)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d_labels = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]
    lines = [f"fx={fx:.1f}  fy={fy:.1f}", f"cx={cx:.1f}  cy={cy:.1f}"]
    if K[0, 1] != 0.0:
        lines.append(f"skew={K[0,1]:.4f}")
    for i, val in enumerate(D):
        if val != 0.0 and i < len(d_labels):
            lines.append(f"{d_labels[i]}={val:.5f}")
    lines.append(f"max|disp| = {mag.max():.1f} px")

    ax.text(
        0.02, 0.02, "\n".join(lines), transform=ax.transAxes, fontsize=9, va="bottom",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.85), zorder=10,
    )
    return sc


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize camera intrinsics distortion as a grid warp."
    )
    parser.add_argument("config", help="ORB-SLAM3 camera config YAML")
    parser.add_argument(
        "-o", "--output", default=None,
        help=(
            "Output figure path. "
            "Defaults to <config_stem>_intrinsics.png next to the config file."
        ),
    )
    parser.add_argument(
        "-n", "--grid-size", type=int, default=16,
        help="Grid resolution per axis (default: 16)",
    )
    args = parser.parse_args()

    plt.style.use(["seaborn-v0_8-paper", "seaborn-v0_8-whitegrid"])
    plt.rcParams.update({
        "savefig.dpi": 300,
        "figure.dpi": 100,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
    })

    width, height, cameras = load_camera_intrinsics(args.config)
    n_cams = len(cameras)

    fig, axes = plt.subplots(1, n_cams, figsize=(7 * n_cams, 7.5))
    if n_cams == 1:
        axes = [axes]

    for ax, cam in zip(axes, cameras):
        sc = _plot_camera_warp(
            ax, cam["K"], cam["D"], width, height,
            cam["name"], args.grid_size,
        )
        plt.colorbar(sc, ax=ax, label="Displacement  (px)", shrink=0.75)

    config_name = Path(args.config).name
    fig.suptitle(
        f"Camera intrinsics — distortion warp\n{config_name}  ({width}\u00d7{height})",
        fontsize=12,
    )
    plt.tight_layout()

    output = Path(args.output) if args.output else \
        Path(args.config).with_name(Path(args.config).stem + "_intrinsics.png")
    plt.savefig(output, bbox_inches="tight")
    print(f"Saved: {output}")


if __name__ == "__main__":
    main()
