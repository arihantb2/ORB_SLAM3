"""Rigorous geometric epipolar validation with configurable pass/fail thresholds."""

from __future__ import annotations

import dataclasses

import numpy as np

from .geometry import estimate_fundamental_inliers, sampson_distance
from .matching import epipolar_stats, matches_to_points


@dataclasses.dataclass(frozen=True)
class RigorousValidationArgs:
    """Thresholds for rigorous epipolar validation."""

    ransac_reproj_thresh: float
    min_inliers: int
    min_inlier_ratio: float
    max_median_dy: float | None
    max_p95_dy: float | None
    max_median_dy_norm: float | None
    max_p95_dy_norm: float | None
    max_median_sampson: float


def rigorous_epipolar_validation(
    kp1: list,
    kp2: list,
    matches: list,
    rect_transforms: dict,
    args: RigorousValidationArgs,
) -> tuple[dict, list]:
    """Return (metrics dict with success/reason, list of inlier matches)."""
    if not matches:
        return {
            "success": False,
            "reason": "no matches after descriptor filtering",
            "candidate_count": 0,
            "inlier_count": 0,
            "inlier_ratio": 0.0,
        }, []

    pts1, pts2 = matches_to_points(kp1, kp2, matches)
    F, inlier_mask = estimate_fundamental_inliers(pts1, pts2, args.ransac_reproj_thresh)
    inlier_matches = [m for m, keep in zip(matches, inlier_mask) if keep]
    inlier_ratio = float(np.mean(inlier_mask)) if len(inlier_mask) else 0.0

    if F is None or not inlier_matches:
        return {
            "success": False,
            "reason": "fundamental matrix estimation failed or yielded no inliers",
            "candidate_count": len(matches),
            "inlier_count": 0,
            "inlier_ratio": inlier_ratio,
        }, inlier_matches

    inlier_pts1 = pts1[inlier_mask]
    inlier_pts2 = pts2[inlier_mask]
    abs_dy = np.abs(inlier_pts1[:, 1] - inlier_pts2[:, 1])
    image_h = max(float(rect_transforms["image_size"][1]), 1.0)
    abs_dy_norm = abs_dy / image_h
    sampson = sampson_distance(F, inlier_pts1, inlier_pts2)

    metrics = {
        "candidate_count": len(matches),
        "inlier_count": int(np.sum(inlier_mask)),
        "inlier_ratio": inlier_ratio,
        "mean_abs_dy": float(np.mean(abs_dy)),
        "median_abs_dy": float(np.median(abs_dy)),
        "p95_abs_dy": float(np.percentile(abs_dy, 95)),
        "max_abs_dy": float(np.max(abs_dy)),
        "mean_abs_dy_norm": float(np.mean(abs_dy_norm)),
        "median_abs_dy_norm": float(np.median(abs_dy_norm)),
        "p95_abs_dy_norm": float(np.percentile(abs_dy_norm, 95)),
        "max_abs_dy_norm": float(np.max(abs_dy_norm)),
        "mean_sampson": float(np.mean(sampson)),
        "median_sampson": float(np.median(sampson)),
        "p95_sampson": float(np.percentile(sampson, 95)),
    }

    fail_reasons = []
    if metrics["inlier_count"] < args.min_inliers:
        fail_reasons.append(f"inlier_count {metrics['inlier_count']} < {args.min_inliers}")
    if metrics["inlier_ratio"] < args.min_inlier_ratio:
        fail_reasons.append(
            f"inlier_ratio {metrics['inlier_ratio']:.3f} < {args.min_inlier_ratio:.3f}"
        )
    if args.max_median_dy is not None and metrics["median_abs_dy"] > args.max_median_dy:
        fail_reasons.append(
            f"median|dy| {metrics['median_abs_dy']:.3f} > {args.max_median_dy:.3f}"
        )
    if args.max_p95_dy is not None and metrics["p95_abs_dy"] > args.max_p95_dy:
        fail_reasons.append(
            f"p95|dy| {metrics['p95_abs_dy']:.3f} > {args.max_p95_dy:.3f}"
        )
    if (
        args.max_median_dy_norm is not None
        and metrics["median_abs_dy_norm"] > args.max_median_dy_norm
    ):
        fail_reasons.append(
            f"median|dy|/h {metrics['median_abs_dy_norm']:.6f} > {args.max_median_dy_norm:.6f}"
        )
    if (
        args.max_p95_dy_norm is not None
        and metrics["p95_abs_dy_norm"] > args.max_p95_dy_norm
    ):
        fail_reasons.append(
            f"p95|dy|/h {metrics['p95_abs_dy_norm']:.6f} > {args.max_p95_dy_norm:.6f}"
        )
    if metrics["median_sampson"] > args.max_median_sampson:
        fail_reasons.append(
            f"median sampson {metrics['median_sampson']:.3f} > {args.max_median_sampson:.3f}"
        )

    metrics["success"] = len(fail_reasons) == 0
    metrics["reason"] = "PASS" if metrics["success"] else "; ".join(fail_reasons)
    return metrics, inlier_matches
