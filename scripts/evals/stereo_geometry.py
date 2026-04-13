#!/usr/bin/env python3
"""Visualize stereo rectification warp and epipolar geometry.

Produces a 3×2 figure — no actual images required, pure geometry:

  Row 0  Rectification warp per camera: original distorted grid (dashed)
         → rectified grid (solid), nodes coloured by displacement magnitude.

  Row 1  Pre-rectification epipolar geometry in undistorted pixel space:
         left sample points / right oblique epipolar lines
         (from F = K2⁻ᵀ [t]ₓ R K1⁻¹).

  Row 2  Post-rectification epipolar geometry in rectified pixel space:
         same sample points warped to rectified left / corresponding epipolar
         lines in rectified right (horizontal — same-row constraint).

Needs ``Stereo.T_c1_c2`` in the camera YAML, or ``--platform-config`` to
resolve extrinsics from a static_tf file.

Usage:
    python stereo_geometry.py --config stereo.yaml --platform-config platform.yaml
    python stereo_geometry.py --config stereo.yaml --platform-config platform.yaml -o out.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from stereo import fundamental_matrix_from_rtk, load_stereo_params
from stereo.cli import add_stereo_config_args


# ---------------------------------------------------------------------------
# Rectification warp helpers
# ---------------------------------------------------------------------------


def _warp_grid_rectify(
    K: np.ndarray,
    D: np.ndarray,
    R_rect: np.ndarray,
    P_rect: np.ndarray,
    width: int,
    height: int,
    n_cols: int,
    n_rows: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Original distorted grid → rectified grid.

    Returns ``(orig_pts, rect_pts)`` each ``(n_rows, n_cols, 2)``.
    """
    margin_x, margin_y = width * 0.05, height * 0.05
    xs = np.linspace(margin_x, width - margin_x, n_cols)
    ys = np.linspace(margin_y, height - margin_y, n_rows)
    gx, gy = np.meshgrid(xs, ys)
    orig_pts = np.stack([gx, gy], axis=-1).astype(np.float64)

    pts_flat = orig_pts.reshape(-1, 1, 2)
    P33 = np.asarray(P_rect)[:3, :3]
    rect_flat = cv2.undistortPoints(pts_flat, K, D, R=R_rect, P=P33)

    return orig_pts, rect_flat.reshape(n_rows, n_cols, 2)


def _grid_segs(pts: np.ndarray) -> list[np.ndarray]:
    n_rows, n_cols = pts.shape[:2]
    return [pts[r, :, :] for r in range(n_rows)] + [pts[:, c, :] for c in range(n_cols)]


def _plot_rectification_warp(
    ax: plt.Axes,
    K: np.ndarray,
    D: np.ndarray,
    R_rect: np.ndarray,
    P_rect: np.ndarray,
    width: int,
    height: int,
    title: str,
    n_grid: int,
) -> plt.cm.ScalarMappable:
    orig_pts, rect_pts = _warp_grid_rectify(
        K, D, R_rect, P_rect, width, height, n_grid, n_grid
    )

    ax.add_collection(LineCollection(
        _grid_segs(orig_pts), colors="lightgray", linewidths=0.8,
        linestyles="dashed", alpha=0.7, label="Original (distorted)", zorder=2,
    ))
    ax.add_collection(LineCollection(
        _grid_segs(rect_pts), colors="steelblue", linewidths=1.1,
        alpha=0.85, label="Rectified", zorder=3,
    ))

    mag = np.hypot(
        rect_pts[..., 0] - orig_pts[..., 0],
        rect_pts[..., 1] - orig_pts[..., 1],
    )
    sc = ax.scatter(
        rect_pts[..., 0].ravel(), rect_pts[..., 1].ravel(),
        c=mag.ravel(), cmap="plasma", s=12, zorder=5, linewidths=0,
    )

    ax.set_xlim(0, width)
    ax.set_ylim(height, 0)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x  (px)")
    ax.set_ylabel("y  (px)")
    ax.legend(loc="lower right", fontsize=7)
    ax.text(
        0.02, 0.02, f"max|disp| = {mag.max():.1f} px",
        transform=ax.transAxes, fontsize=7, va="bottom",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.75),
    )
    return sc


# ---------------------------------------------------------------------------
# Epipolar geometry helpers
# ---------------------------------------------------------------------------


def _sample_grid_points(
    width: int, height: int, n_cols: int = 5, n_rows: int = 4
) -> np.ndarray:
    """Regular grid of points with small jitter, in undistorted pixel space."""
    rng = np.random.default_rng(7)
    xs = np.linspace(0.12 * width,  0.88 * width,  n_cols)
    ys = np.linspace(0.18 * height, 0.82 * height, n_rows)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.stack([gx.ravel(), gy.ravel()], axis=-1).astype(np.float64)
    pts[:, 0] += rng.uniform(-0.03 * width,  0.03 * width,  len(pts))
    pts[:, 1] += rng.uniform(-0.03 * height, 0.03 * height, len(pts))
    return pts


def _clip_line(
    a: float, b: float, c: float,
    x0: float, x1: float, y0: float, y1: float,
) -> tuple | None:
    """Clip line ax+by+c=0 to bounding box. Returns (px0,py0,px1,py1) or None."""
    cands: list[tuple[float, float]] = []
    if abs(b) > 1e-12:
        for x in (x0, x1):
            y = -(a * x + c) / b
            if y0 <= y <= y1:
                cands.append((float(x), float(y)))
    if abs(a) > 1e-12:
        for y in (y0, y1):
            x = -(b * y + c) / a
            if x0 <= x <= x1:
                cands.append((float(x), float(y)))
    unique: list[tuple[float, float]] = []
    for p in cands:
        if not any(abs(p[0] - q[0]) < 1.0 and abs(p[1] - q[1]) < 1.0 for q in unique):
            unique.append(p)
    if len(unique) < 2:
        return None
    unique.sort()
    return (*unique[0], *unique[-1])


def _undistorted_to_rectified(
    pts: np.ndarray,
    K: np.ndarray,
    R_rect: np.ndarray,
    P_rect: np.ndarray,
) -> np.ndarray:
    """Map undistorted pixel coords → rectified pixel coords (no distortion step)."""
    pts_flat = pts.reshape(-1, 1, 2).astype(np.float64)
    P33 = np.asarray(P_rect)[:3, :3]
    out = cv2.undistortPoints(pts_flat, K, None, R=R_rect, P=P33)
    return out.reshape(-1, 2)


def _setup_image_axes(
    ax: plt.Axes, width: int, height: int, title: str
) -> None:
    pad_x, pad_y = width * 0.02, height * 0.02
    ax.set_facecolor("#f5f6fa")
    ax.add_patch(plt.Rectangle(
        (0, 0), width, height,
        fill=False, edgecolor="#888", linewidth=1.5, zorder=1,
    ))
    ax.set_xlim(-pad_x, width + pad_x)
    ax.set_ylim(height + pad_y, -pad_y)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x  (px)")
    ax.set_ylabel("y  (px)")


def _plot_epipolar_row(
    ax_left: plt.Axes,
    ax_right: plt.Axes,
    pts_left: np.ndarray,
    width: int,
    height: int,
    title_left: str,
    title_right: str,
    *,
    F: np.ndarray | None = None,
    rectified: bool = False,
) -> None:
    """Draw sample points (left) and their epipolar lines (right).

    ``rectified=False``: epilines are oblique, computed from F.
    ``rectified=True``:  epilines are horizontal (y = const).
    """
    n = len(pts_left)
    colors = [plt.cm.gist_rainbow(i / max(n - 1, 1)) for i in range(n)]

    _setup_image_axes(ax_left,  width, height, title_left)
    _setup_image_axes(ax_right, width, height, title_right)

    for pt, color in zip(pts_left, colors):
        ax_left.plot(
            pt[0], pt[1], "o", color=color, markersize=5, zorder=3,
            markeredgewidth=0.5, markeredgecolor="white",
        )

    if rectified:
        for pt, color in zip(pts_left, colors):
            y = pt[1]
            if 0 <= y <= height:
                ax_right.hlines(y, 0, width, colors=color, linewidth=1.2, alpha=0.85, zorder=2)
    else:
        assert F is not None
        pts_h = np.hstack([pts_left, np.ones((n, 1))])
        epilines = (F @ pts_h.T).T  # (n, 3) — [a, b, c]
        for (a, b, c), color in zip(epilines, colors):
            seg = _clip_line(a, b, c, 0, width, 0, height)
            if seg:
                x0, y0, x1, y1 = seg
                ax_right.plot([x0, x1], [y0, y1], color=color, linewidth=1.2, alpha=0.85, zorder=2)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize stereo rectification warp and epipolar geometry."
    )
    parser.add_argument("--config", required=True, help="ORB-SLAM3 stereo camera YAML")
    add_stereo_config_args(parser)
    parser.add_argument(
        "-o", "--output", default=None,
        help=(
            "Output figure path. "
            "Defaults to <config_stem>_stereo_geometry.png next to the config file."
        ),
    )
    parser.add_argument(
        "-n", "--grid-size", type=int, default=14,
        help="Rectification warp grid resolution (default: 14)",
    )
    args = parser.parse_args()

    params = load_stereo_params(
        args.config,
        platform_config_path=args.platform_config,
        stereo_left_frame=args.stereo_left_frame,
        stereo_right_frame=args.stereo_right_frame,
    )
    (width, height), K1, D1, K2, D2, R, t, camera_type = params
    image_size = (width, height)

    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, image_size, R, t,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
    )

    F = fundamental_matrix_from_rtk(K1, K2, R, t)

    pts_undist = _sample_grid_points(width, height)
    pts_rect   = _undistorted_to_rectified(pts_undist, K1, R1, P1)
    inside = (
        (pts_rect[:, 0] >= 0) & (pts_rect[:, 0] <= width) &
        (pts_rect[:, 1] >= 0) & (pts_rect[:, 1] <= height)
    )
    pts_undist_vis = pts_undist[inside]
    pts_rect_vis   = pts_rect[inside]

    fig, axes = plt.subplots(3, 2, figsize=(14, 18))

    # Row 0: rectification warps
    for col, (K, D, R_rect, P_rect, label) in enumerate([
        (K1, D1, R1, P1, "Camera 1 (Left)"),
        (K2, D2, R2, P2, "Camera 2 (Right)"),
    ]):
        sc = _plot_rectification_warp(
            axes[0, col], K, D, R_rect, P_rect, width, height,
            f"{label} — rectification warp\n(dashed = original,  solid = rectified)",
            args.grid_size,
        )
        plt.colorbar(sc, ax=axes[0, col], label="Displacement  (px)", shrink=0.75)

    # Row 1: pre-rectification epipolar geometry
    _plot_epipolar_row(
        axes[1, 0], axes[1, 1],
        pts_undist_vis, width, height,
        "Pre-rectification — left\n(undistorted pixel space)",
        "Pre-rectification — epipolar lines in right\n(oblique — converge toward epipole)",
        F=F, rectified=False,
    )

    # Row 2: post-rectification epipolar geometry
    _plot_epipolar_row(
        axes[2, 0], axes[2, 1],
        pts_rect_vis, width, height,
        "Post-rectification — left\n(rectified pixel space)",
        "Post-rectification — epipolar lines in right\n(horizontal — same-row constraint)",
        rectified=True,
    )

    config_name = Path(args.config).name
    fig.suptitle(
        f"Stereo geometry — {camera_type}\n{config_name}  ({width}×{height})",
        fontsize=12,
    )
    plt.tight_layout()

    output = Path(args.output) if args.output else \
        Path(args.config).with_name(Path(args.config).stem + "_stereo_geometry.png")
    plt.savefig(output, dpi=150, bbox_inches="tight")
    print(f"Saved: {output}")


if __name__ == "__main__":
    main()
