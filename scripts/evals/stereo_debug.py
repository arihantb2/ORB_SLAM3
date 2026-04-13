#!/usr/bin/env python3
"""Rectify a stereo pair and run an epipolar sanity check from feature matches.

Usage:
    python stereo_debug.py --config stereo.yaml --left left.png --right right.png
    python stereo_debug.py --config stereo.yaml --left left.png --right right.png \\
        --platform-config platform.yaml --rigorous -o debug/run1
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2

from stereo import (
    RigorousValidationArgs,
    detect_and_match,
    draw_matches_overlay,
    epipolar_stats,
    load_stereo_params,
    rectify_pair,
    resize_to_width,
    rigorous_epipolar_validation,
    wait_for_windows_or_key,
    window_search_debug,
)
from stereo.cli import add_stereo_config_args


def _parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Rectify a stereo pair using an ORB-SLAM3 stereo YAML and run a quick "
            "epipolar sanity check from feature matches."
        )
    )
    parser.add_argument("--config", required=True, help="Stereo ORB-SLAM3 YAML config path.")
    parser.add_argument("--left",   required=True, help="Left image path.")
    parser.add_argument("--right",  required=True, help="Right image path.")
    add_stereo_config_args(parser)
    parser.add_argument(
        "--detector", choices=["orb", "sift"], default="orb",
        help="Feature detector used for matching on rectified images (default: orb).",
    )
    parser.add_argument(
        "--max-matches", type=int, default=200,
        help="Maximum number of best matches to keep for stats/visualization (default: 200).",
    )
    parser.add_argument(
        "--line-step", type=int, default=40,
        help="Vertical pixel step for epipolar guideline overlay (default: 40).",
    )
    parser.add_argument(
        "-o", "--output", default=None,
        help=(
            "Output path prefix for debug images. "
            "Writes <output>_matches.png and <output>_window_search.png. "
            "Defaults to <config_stem>_debug next to the config file."
        ),
    )
    parser.add_argument(
        "--rigorous", action="store_true",
        help="Enable rigorous geometric epipolar validation with pass/fail thresholds.",
    )
    parser.add_argument(
        "--ratio-test", type=float, default=0.7,
        help="Lowe ratio-test threshold (default: 0.7).",
    )
    parser.add_argument(
        "--ransac-reproj-thresh", type=float, default=1.0,
        help="RANSAC/USAC reprojection threshold (px) for fundamental matrix estimation (default: 1.0).",
    )
    parser.add_argument(
        "--min-inliers", type=int, default=50,
        help="Minimum number of geometric inliers required to pass (default: 50).",
    )
    parser.add_argument(
        "--min-inlier-ratio", type=float, default=0.40,
        help="Minimum inlier ratio required to pass (default: 0.40).",
    )
    parser.add_argument("--max-median-dy", type=float, default=None)
    parser.add_argument("--max-p95-dy",    type=float, default=None)
    parser.add_argument(
        "--max-median-dy-norm", type=float, default=0.002,
        help="Maximum allowed median |dy|/height on rectified inliers (default: 0.002).",
    )
    parser.add_argument(
        "--max-p95-dy-norm", type=float, default=0.004,
        help="Maximum allowed p95 |dy|/height on rectified inliers (default: 0.004).",
    )
    parser.add_argument(
        "--max-median-sampson", type=float, default=0.50,
        help="Maximum allowed median Sampson distance (px²) on geometric inliers (default: 0.50).",
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
    matches = all_matches[:args.max_matches]
    stats   = epipolar_stats(kp1, kp2, matches)

    rigorous_metrics = None
    if args.rigorous:
        val_args = RigorousValidationArgs(
            ransac_reproj_thresh=args.ransac_reproj_thresh,
            min_inliers=args.min_inliers,
            min_inlier_ratio=args.min_inlier_ratio,
            max_median_dy=args.max_median_dy,
            max_p95_dy=args.max_p95_dy,
            max_median_dy_norm=args.max_median_dy_norm,
            max_p95_dy_norm=args.max_p95_dy_norm,
            max_median_sampson=args.max_median_sampson,
        )
        rigorous_metrics, inlier_matches = rigorous_epipolar_validation(
            kp1, kp2, all_matches, rect_transforms, val_args
        )
        if inlier_matches:
            matches = inlier_matches[:args.max_matches]
            stats   = epipolar_stats(kp1, kp2, matches)

    matches_vis = draw_matches_overlay(
        rect_left, rect_right, kp1, kp2, matches, args.line_step
    )
    window_search_vis, valid_ratio, low_texture_ratio = window_search_debug(
        left_gray, right_gray
    )

    stats_text = (
        f"matches={stats['count']} | "
        f"mean|dy|={stats['mean_abs_dy']:.2f}px | "
        f"median|dy|={stats['median_abs_dy']:.2f}px | "
        f"max|dy|={stats['max_abs_dy']:.2f}px"
    )
    cv2.putText(
        matches_vis, stats_text, (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA,
    )

    print(stats_text)
    print(
        f"window-search valid ratio={valid_ratio*100:.1f}% | "
        f"low-texture ratio={low_texture_ratio*100:.1f}%"
    )

    if rigorous_metrics is not None:
        image_height = float(left_gray.shape[0])
        print("Rigorous epipolar validation:")
        print(
            f"  candidates={rigorous_metrics['candidate_count']} "
            f"| inliers={rigorous_metrics['inlier_count']} "
            f"| inlier_ratio={rigorous_metrics['inlier_ratio']:.3f}"
        )
        print(
            f"  rectified: mean|dy|={rigorous_metrics['mean_abs_dy']:.3f}px "
            f"| median|dy|={rigorous_metrics['median_abs_dy']:.3f}px "
            f"| p95|dy|={rigorous_metrics['p95_abs_dy']:.3f}px "
            f"| max|dy|={rigorous_metrics['max_abs_dy']:.3f}px"
        )
        print(
            f"  normalized: mean|dy|/h={rigorous_metrics['mean_abs_dy_norm']:.6f} "
            f"| median={rigorous_metrics['median_abs_dy_norm']:.6f} "
            f"| p95={rigorous_metrics['p95_abs_dy_norm']:.6f} "
            f"| max={rigorous_metrics['max_abs_dy_norm']:.6f}"
        )
        print(
            f"  Sampson: mean={rigorous_metrics['mean_sampson']:.3f} "
            f"| median={rigorous_metrics['median_sampson']:.3f} "
            f"| p95={rigorous_metrics['p95_sampson']:.3f}"
        )
        if args.max_median_dy_norm is not None:
            print(
                f"  threshold median|dy|/h <= {args.max_median_dy_norm:.6f} "
                f"(~{args.max_median_dy_norm * image_height:.3f}px at h={int(image_height)})"
            )
        if args.max_p95_dy_norm is not None:
            print(
                f"  threshold p95|dy|/h <= {args.max_p95_dy_norm:.6f} "
                f"(~{args.max_p95_dy_norm * image_height:.3f}px at h={int(image_height)})"
            )
        print(
            f"  result: {'PASS' if rigorous_metrics['success'] else 'FAIL'} "
            f"({rigorous_metrics['reason']})"
        )

    matches_vis        = resize_to_width(matches_vis,        target_width=1440)
    window_search_vis  = resize_to_width(window_search_vis,  target_width=720)

    from pathlib import Path
    output_prefix = args.output or str(
        Path(args.config).with_name(Path(args.config).stem + "_debug")
    )
    out_dir = os.path.dirname(output_prefix)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    matches_path      = f"{output_prefix}_matches.png"
    window_search_path = f"{output_prefix}_window_search.png"
    if not cv2.imwrite(matches_path, matches_vis):
        raise ValueError(f"Failed to write image: {matches_path}")
    if not cv2.imwrite(window_search_path, window_search_vis):
        raise ValueError(f"Failed to write image: {window_search_path}")
    print(f"Saved:\n  {matches_path}\n  {window_search_path}")

    print("Press any key in an image window to exit.")
    matches_win      = "Feature Matches on Rectified Pair"
    window_search_win = "Window Search Debug (StereoBM)"
    cv2.imshow(matches_win,      matches_vis)
    cv2.imshow(window_search_win, window_search_vis)
    wait_for_windows_or_key([matches_win, window_search_win])
    cv2.destroyAllWindows()

    if rigorous_metrics is not None and not rigorous_metrics["success"]:
        sys.exit(2)


if __name__ == "__main__":
    main()
