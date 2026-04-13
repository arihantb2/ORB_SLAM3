"""Epipolar geometry: fundamental matrix, Sampson distance, RANSAC inlier estimation."""

from __future__ import annotations

import cv2
import numpy as np


def fundamental_matrix_from_rtk(
    K1: np.ndarray,
    K2: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
) -> np.ndarray:
    """Fundamental matrix from intrinsics and relative pose (left→right).

    F such that x2ᵀ F x1 = 0, with x1, x2 in undistorted pixel coordinates.
    """
    t_skew = np.array(
        [[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]],
        dtype=np.float64,
    )
    E = t_skew @ R
    return np.linalg.inv(K2).T @ E @ np.linalg.inv(K1)


def sampson_distance(
    F: np.ndarray,
    pts1: np.ndarray,
    pts2: np.ndarray,
) -> np.ndarray:
    """Sampson distance (squared) for each point pair."""
    x1 = np.hstack([pts1, np.ones((pts1.shape[0], 1), dtype=np.float64)])
    x2 = np.hstack([pts2, np.ones((pts2.shape[0], 1), dtype=np.float64)])
    Fx1 = (F @ x1.T).T
    Ftx2 = (F.T @ x2.T).T
    numer = np.square(np.sum(x2 * Fx1, axis=1))
    denom = (
        np.square(Fx1[:, 0]) + np.square(Fx1[:, 1])
        + np.square(Ftx2[:, 0]) + np.square(Ftx2[:, 1])
    )
    return numer / np.maximum(denom, 1e-12)


def estimate_fundamental_inliers(
    pts1: np.ndarray,
    pts2: np.ndarray,
    ransac_reproj_thresh: float,
) -> tuple[np.ndarray | None, np.ndarray]:
    """Return (F or None, boolean inlier mask) via RANSAC/USAC_MAGSAC."""
    if len(pts1) < 8:
        return None, np.zeros(len(pts1), dtype=bool)

    method = cv2.USAC_MAGSAC if hasattr(cv2, "USAC_MAGSAC") else cv2.FM_RANSAC
    F, mask = cv2.findFundamentalMat(
        pts1, pts2, method, ransac_reproj_thresh, 0.999, 10000,
    )
    if F is None or mask is None:
        return None, np.zeros(len(pts1), dtype=bool)
    return F, mask.ravel().astype(bool)
