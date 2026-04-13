"""Feature detection, descriptor matching, and epipolar statistics."""

from __future__ import annotations

import cv2
import numpy as np


def detect_and_match(
    left_gray: np.ndarray,
    right_gray: np.ndarray,
    detector_name: str,
    ratio_test: float,
) -> tuple[list, list, list]:
    """Return (kp1, kp2, matches) sorted by distance."""
    if detector_name == "sift":
        if not hasattr(cv2, "SIFT_create"):
            raise ValueError("SIFT is not available in this OpenCV build.")
        detector = cv2.SIFT_create()
        matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    else:
        detector = cv2.ORB_create(nfeatures=3000)
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    kp1, des1 = detector.detectAndCompute(left_gray, None)
    kp2, des2 = detector.detectAndCompute(right_gray, None)

    if des1 is None or des2 is None:
        return kp1, kp2, []

    knn = matcher.knnMatch(des1, des2, k=2)
    matches = [
        m for pair in knn
        if len(pair) >= 2 and (m := pair[0]).distance < ratio_test * pair[1].distance
    ]
    matches.sort(key=lambda m: m.distance)
    return kp1, kp2, matches


def matches_to_points(
    kp1: list, kp2: list, matches: list
) -> tuple[np.ndarray, np.ndarray]:
    """Extract (N, 2) point arrays from keypoints and matches."""
    pts1 = np.array([kp1[m.queryIdx].pt for m in matches], dtype=np.float64)
    pts2 = np.array([kp2[m.trainIdx].pt for m in matches], dtype=np.float64)
    return pts1, pts2


def epipolar_stats(
    kp1: list, kp2: list, matches: list
) -> dict[str, float | int]:
    """Return count, mean_abs_dy, median_abs_dy, max_abs_dy."""
    if not matches:
        return {
            "count": 0,
            "mean_abs_dy": float("nan"),
            "median_abs_dy": float("nan"),
            "max_abs_dy": float("nan"),
        }
    dy = np.array(
        [kp1[m.queryIdx].pt[1] - kp2[m.trainIdx].pt[1] for m in matches],
        dtype=np.float64,
    )
    abs_dy = np.abs(dy)
    return {
        "count": len(matches),
        "mean_abs_dy": float(np.mean(abs_dy)),
        "median_abs_dy": float(np.median(abs_dy)),
        "max_abs_dy": float(np.max(abs_dy)),
    }
