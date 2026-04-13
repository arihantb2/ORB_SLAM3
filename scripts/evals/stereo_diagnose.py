#!/usr/bin/env python3
"""Diagnose stereo calibration: spatial dy, left/right reprojection RMS, pre-rect vs rectified.

Usage:
    python stereo_diagnose.py --config stereo.yaml --left left.png --right right.png
    python stereo_diagnose.py --config stereo.yaml --left left.png --right right.png \\
        --platform-config platform.yaml --plots
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import cv2
import numpy as np

from stereo import (
    detect_and_match,
    estimate_fundamental_inliers,
    fundamental_matrix_from_rtk,
    load_stereo_params,
    matches_to_points,
    rectify_pair,
    sampson_distance,
)
from stereo.cli import add_stereo_config_args


def _triangulate_and_reproject_rect(
    pts1: np.ndarray,
    pts2: np.ndarray,
    P1: np.ndarray,
    P2: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Triangulate from rectified correspondences; return reproj_left (N,2), reproj_right (N,2), X (4,N)."""
    X = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)
    X = X / np.maximum(X[3:4, :], 1e-12)
    reproj_left  = ((P1 @ X)[:2, :] / (P1 @ X)[2:3, :]).T
    reproj_right = ((P2 @ X)[:2, :] / (P2 @ X)[2:3, :]).T
    return reproj_left, reproj_right, X


def _reprojection_errors(
    pts1: np.ndarray,
    pts2: np.ndarray,
    P1: np.ndarray,
    P2: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Per-point reprojection error (L2 norm in px) in rectified space."""
    reproj_left, reproj_right, _ = _triangulate_and_reproject_rect(pts1, pts2, P1, P2)
    return np.linalg.norm(pts1 - reproj_left, axis=1), np.linalg.norm(pts2 - reproj_right, axis=1)


def _unrectify_points(
    X_rect: np.ndarray,
    K1: np.ndarray,
    K2: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    R1: np.ndarray,
    *,
    min_z: float = 1e-6,
    right_from_left_inverse: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Map 3D from rectified left frame to undistorted left/right image points. X_rect (4, N).

    Returns (pt_left (N,2), pt_right (N,2), valid_mask (N,)).
    If right_from_left_inverse is True, use p_right = R.T @ (p_left - T)
    (for configs that store T_c2_c1).
    """
    x3 = X_rect[:3, :] / np.maximum(X_rect[3:4, :], 1e-12)
    x_left = R1.T @ x3
    t_col = t.reshape(3, 1)
    x_right = R.T @ (x_left - t_col) if right_from_left_inverse else R @ x_left + t_col
    z_left  = (K1 @ x_left)[2:3, :].ravel()
    z_right = (K2 @ x_right)[2:3, :].ravel()
    valid = (z_left > min_z) & (z_right > min_z)
    pt_left  = (K1 @ x_left)[:2,  :] / np.maximum((K1 @ x_left)[2:3,  :], 1e-12)
    pt_right = (K2 @ x_right)[:2, :] / np.maximum((K2 @ x_right)[2:3, :], 1e-12)
    return pt_left.T, pt_right.T, valid


def _spatial_dy_metrics(
    pts1: np.ndarray, pts2: np.ndarray,
    width: float, height: float, n_quad: int = 2,
) -> dict:
    """Quadrant median |dy| + linear fit dy = a*x_norm + b*y_norm + c, residual RMS."""
    dy = pts1[:, 1] - pts2[:, 1]
    abs_dy = np.abs(dy)
    x_norm = pts1[:, 0] / max(width, 1e-9)
    y_norm = pts1[:, 1] / max(height, 1e-9)

    step_x, step_y = 1.0 / n_quad, 1.0 / n_quad
    quad_medians = {}
    for i in range(n_quad):
        for j in range(n_quad):
            mask = (
                (x_norm >= i * step_x) & (x_norm < (i + 1) * step_x)
                & (y_norm >= j * step_y) & (y_norm < (j + 1) * step_y)
            )
            quad_medians[(i, j)] = float(np.median(abs_dy[mask])) if mask.any() else float("nan")

    design = np.column_stack([x_norm, y_norm, np.ones_like(x_norm)])
    coeffs, _, _, _ = np.linalg.lstsq(design, dy, rcond=None)
    fitted = design @ coeffs
    residual_rms = float(np.sqrt(np.mean((dy - fitted) ** 2)))

    return {
        "mean_dy_signed": float(np.mean(dy)),
        "quadrant_median_abs_dy": quad_medians,
        "linear_coeffs": {"a": coeffs[0], "b": coeffs[1], "c": coeffs[2]},
        "residual_rms": residual_rms,
        "n_quad": n_quad,
    }


def _run_diagnostics(
    params: tuple,
    rect_transforms: dict,
    pts1: np.ndarray,
    pts2: np.ndarray,
) -> tuple[dict, dict, dict]:
    """Run all three diagnostics. Returns (spatial_metrics, reproj_metrics, prerect_metrics)."""
    (width, height), K1, D1, K2, D2, R, t, _ = params
    P1 = rect_transforms["P1"]
    P2 = rect_transforms["P2"]
    R1 = rect_transforms["r1"]

    spatial_metrics = _spatial_dy_metrics(pts1, pts2, float(width), float(height), n_quad=2)

    err_left, err_right = _reprojection_errors(pts1, pts2, P1, P2)
    reproj_metrics = {
        "rms_left_px":  float(np.sqrt(np.mean(err_left  ** 2))),
        "rms_right_px": float(np.sqrt(np.mean(err_right ** 2))),
        "err_left":  err_left,
        "err_right": err_right,
        "mean_left":  float(np.mean(err_left)),   "std_left":  float(np.std(err_left)),
        "mean_right": float(np.mean(err_right)),  "std_right": float(np.std(err_right)),
        "min_left":  float(np.min(err_left)),     "max_left":  float(np.max(err_left)),
        "min_right": float(np.min(err_right)),    "max_right": float(np.max(err_right)),
    }

    _, _, X_rect = _triangulate_and_reproject_rect(pts1, pts2, P1, P2)
    pt_left_std, pt_right_std, valid_std = _unrectify_points(
        X_rect, K1, K2, R, t, R1, right_from_left_inverse=False
    )
    n_std = int(np.sum(valid_std))
    n_total = len(valid_std)
    if n_std < max(8, 0.1 * n_total):
        pt_left_inv, pt_right_inv, valid_inv = _unrectify_points(
            X_rect, K1, K2, R, t, R1, right_from_left_inverse=True
        )
        if int(np.sum(valid_inv)) > n_std:
            pt_left_u, pt_right_u, valid_front = pt_left_inv, pt_right_inv, valid_inv
            F = fundamental_matrix_from_rtk(K1, K2, R.T, -R.T @ t)
            conv_note = {"extrinsics_convention": "T_c2_c1 (inverse)"}
        else:
            pt_left_u, pt_right_u, valid_front = pt_left_std, pt_right_std, valid_std
            F = fundamental_matrix_from_rtk(K1, K2, R, t)
            conv_note = {"extrinsics_convention": "T_c1_c2 (standard)"}
    else:
        pt_left_u, pt_right_u, valid_front = pt_left_std, pt_right_std, valid_std
        F = fundamental_matrix_from_rtk(K1, K2, R, t)
        conv_note = {"extrinsics_convention": "T_c1_c2 (standard)"}

    sampson_all = sampson_distance(F, pt_left_u, pt_right_u)
    sampson = sampson_all[valid_front] if np.any(valid_front) else np.array([])
    prerect_metrics = {
        "median_sampson_undistorted": float(np.median(sampson)) if len(sampson) > 0 else float("nan"),
        "mean_sampson_undistorted":   float(np.mean(sampson))   if len(sampson) > 0 else float("nan"),
        "median_abs_dy_rectified":    float(np.median(np.abs(pts1[:, 1] - pts2[:, 1]))),
        "n_valid_front": int(np.sum(valid_front)),
        "n_total": n_total,
        **conv_note,
    }
    return spatial_metrics, reproj_metrics, prerect_metrics


def _print_report(
    spatial_metrics: dict, reproj_metrics: dict, prerect_metrics: dict, n_inliers: int
) -> None:
    print("\n--- Diagnostic report ---")
    print(f"Inlier correspondences: {n_inliers}")

    print("\n1. Spatial distribution of dy (rectified)")
    print(f"   Mean dy (signed): {spatial_metrics['mean_dy_signed']:.4f} px")
    nq = spatial_metrics["n_quad"]
    print(f"   Median |dy| per quadrant ({nq}×{nq}):")
    for (i, j), med in spatial_metrics["quadrant_median_abs_dy"].items():
        val = f"{med:.3f}" if not np.isnan(med) else "nan"
        print(f"     quadrant ({i},{j}): {val} px")
    c = spatial_metrics["linear_coeffs"]
    print(f"   Linear fit dy = {c['a']:.4f}·x_norm + {c['b']:.4f}·y_norm + {c['c']:.4f}")
    print(f"   Residual RMS: {spatial_metrics['residual_rms']:.4f} px")
    print("   (low residual → extrinsic-like error; high → intrinsic-like)")

    print("\n2. Left vs right reprojection (rectified)")
    m = reproj_metrics
    print(f"   Left:  mean={m['mean_left']:.4f}  std={m['std_left']:.4f}  "
          f"range=[{m['min_left']:.4f}, {m['max_left']:.4f}]  RMS={m['rms_left_px']:.4f} px")
    print(f"   Right: mean={m['mean_right']:.4f}  std={m['std_right']:.4f}  "
          f"range=[{m['min_right']:.4f}, {m['max_right']:.4f}]  RMS={m['rms_right_px']:.4f} px")
    print("   (large asymmetry → that camera's parameters are more suspect)")

    print("\n3. Pre-rectified vs rectified geometry")
    n_valid = prerect_metrics.get("n_valid_front", prerect_metrics.get("n_total", 0))
    n_tot   = prerect_metrics.get("n_total", 0)
    if n_tot > 0:
        print(f"   Points in front of both cameras: {n_valid} / {n_tot}")
    conv = prerect_metrics.get("extrinsics_convention", "")
    if conv and "inverse" in conv:
        print(f"   (Used {conv} for unrectify; config may store right-to-left transform.)")
    print(f"   Undistorted (pre-rect) median Sampson: {prerect_metrics['median_sampson_undistorted']:.4f}")
    print(f"   Rectified median |dy|:                {prerect_metrics['median_abs_dy_rectified']:.4f} px")
    print("   (pre-rect good but rectified bad → rectification step suspect; both bad → intrinsics/extrinsics)")


def _save_plots(
    spatial_metrics: dict,
    reproj_metrics: dict,
    pts1: np.ndarray,
    pts2: np.ndarray,
    output_prefix: str,
) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skipping --plots.")
        return

    out_dir = Path(output_prefix).parent
    if out_dir and not out_dir.exists():
        out_dir.mkdir(parents=True, exist_ok=True)

    # 1) Quadrant median |dy| bar chart
    fig, ax = plt.subplots()
    nq   = spatial_metrics["n_quad"]
    quad = spatial_metrics["quadrant_median_abs_dy"]
    labels = [f"({i},{j})" for (i, j) in sorted(quad.keys())]
    vals   = [quad[k] if not np.isnan(quad[k]) else 0.0 for k in sorted(quad.keys())]
    ax.bar(labels, vals, color="steelblue", edgecolor="black")
    ax.set_ylabel("Median |dy| (px)")
    ax.set_title("Median |dy| by quadrant")
    fig.tight_layout()
    fig.savefig(f"{output_prefix}_quadrants.png", dpi=120)
    plt.close(fig)

    # 2) Left vs right reprojection: violin plot
    fig, ax = plt.subplots()
    parts = ax.violinplot(
        [reproj_metrics["err_left"], reproj_metrics["err_right"]],
        positions=[0, 1], showmeans=True, showmedians=True,
    )
    for i, pc in enumerate(parts["bodies"]):
        pc.set_facecolor("coral" if i == 0 else "seagreen")
        pc.set_alpha(0.7)
    ax.set_xticks([0, 1])
    ax.set_xticklabels(["Left", "Right"])
    ax.set_ylabel("Reprojection error (px)")
    ax.set_title("Reprojection error distribution")
    fig.tight_layout()
    fig.savefig(f"{output_prefix}_reproj.png", dpi=120)
    plt.close(fig)

    # 3) dy vs position scatter
    dy = pts1[:, 1] - pts2[:, 1]
    width_  = max(np.ptp(pts1[:, 0]), 1e-9)
    height_ = max(np.ptp(pts1[:, 1]), 1e-9)
    x_norm = (pts1[:, 0] - np.min(pts1[:, 0])) / width_
    y_norm = (pts1[:, 1] - np.min(pts1[:, 1])) / height_
    fig, ax = plt.subplots()
    sc = ax.scatter(x_norm, dy, c=y_norm, cmap="viridis", s=8, alpha=0.7)
    ax.axhline(0, color="gray", linestyle="--")
    ax.set_xlabel("x (normalized)")
    ax.set_ylabel("dy (px)")
    ax.set_title("dy vs x (colour = y)")
    plt.colorbar(sc, ax=ax, label="y norm")
    fig.tight_layout()
    fig.savefig(f"{output_prefix}_dy_spatial.png", dpi=120)
    plt.close(fig)

    print(f"Diagnosis plots saved to {output_prefix}_*.png")


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Diagnose stereo calibration: spatial dy, reprojection RMS, pre-rect vs rectified."
    )
    parser.add_argument("--config", required=True, help="Stereo ORB-SLAM3 YAML config path.")
    parser.add_argument("--left",   required=True, help="Left image path.")
    parser.add_argument("--right",  required=True, help="Right image path.")
    add_stereo_config_args(parser)
    parser.add_argument(
        "--detector", choices=["orb", "sift"], default="orb",
        help="Feature detector for matching (default: orb).",
    )
    parser.add_argument(
        "--ratio-test", type=float, default=0.7,
        help="Lowe ratio-test threshold (default: 0.7).",
    )
    parser.add_argument(
        "--ransac-reproj-thresh", type=float, default=1.0,
        help="RANSAC reprojection threshold (px) for fundamental matrix inliers (default: 1.0).",
    )
    parser.add_argument(
        "-o", "--output", default=None,
        help=(
            "Output path prefix for diagnostic plots "
            "(e.g. out/diagnose → out/diagnose_quadrants.png, _reproj.png, _dy_spatial.png). "
            "Defaults to <config_stem>_diagnose next to the config file."
        ),
    )
    parser.add_argument(
        "--plots", action="store_true",
        help="Generate and save diagnosis plots (requires matplotlib).",
    )
    return parser.parse_args()


def main():
    args = _parse_args()

    for path in (args.config, args.left, args.right):
        if not os.path.exists(path):
            raise FileNotFoundError(f"Path not found: {path}")

    left_bgr  = cv2.imread(args.left,  cv2.IMREAD_COLOR)
    right_bgr = cv2.imread(args.right, cv2.IMREAD_COLOR)
    if left_bgr is None or right_bgr is None:
        raise ValueError("Could not read one or both input images.")
    if left_bgr.shape[:2] != right_bgr.shape[:2]:
        raise ValueError("Left and right images must have identical dimensions.")

    params = load_stereo_params(
        args.config,
        platform_config_path=args.platform_config,
        stereo_left_frame=args.stereo_left_frame,
        stereo_right_frame=args.stereo_right_frame,
    )
    print(f"Loaded camera model: {params[-1]}")

    rect_left, rect_right, rect_transforms = rectify_pair(
        left_bgr, right_bgr, params, return_transforms=True
    )

    left_gray  = cv2.cvtColor(rect_left,  cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
    kp1, kp2, all_matches = detect_and_match(left_gray, right_gray, args.detector, args.ratio_test)
    pts1, pts2 = matches_to_points(kp1, kp2, all_matches)
    F, inlier_mask = estimate_fundamental_inliers(pts1, pts2, args.ransac_reproj_thresh)

    if F is None or np.sum(inlier_mask) < 8:
        print("Too few inliers for diagnosis. Try relaxing --ransac-reproj-thresh or use a different image pair.")
        return

    inlier_pts1 = pts1[inlier_mask]
    inlier_pts2 = pts2[inlier_mask]

    spatial_metrics, reproj_metrics, prerect_metrics = _run_diagnostics(
        params, rect_transforms, inlier_pts1, inlier_pts2
    )
    _print_report(spatial_metrics, reproj_metrics, prerect_metrics, len(inlier_pts1))

    if args.plots:
        output_prefix = args.output or str(
            Path(args.config).with_name(Path(args.config).stem + "_diagnose")
        )
        _save_plots(spatial_metrics, reproj_metrics, inlier_pts1, inlier_pts2, output_prefix)


if __name__ == "__main__":
    main()
