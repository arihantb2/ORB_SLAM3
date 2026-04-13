"""Stereo image rectification for pinhole camera models."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def rectify_pair(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    params: tuple,
    *,
    return_transforms: bool = False,
) -> tuple[np.ndarray, np.ndarray] | tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """Rectify a stereo pair.

    ``params`` is the tuple returned by :func:`stereo.config.load_stereo_params`.
    If ``return_transforms`` is True, also returns a dict with
    ``r1``, ``r2``, ``P1``, ``P2``, ``image_size``.
    """
    config_size, K1, D1, K2, D2, R, t, _ = params
    h, w = left_bgr.shape[:2]
    image_size = (w, h)

    K1 = np.asarray(K1, dtype=np.float64)
    D1 = np.asarray(D1, dtype=np.float64)
    K2 = np.asarray(K2, dtype=np.float64)
    D2 = np.asarray(D2, dtype=np.float64)
    R  = np.asarray(R,  dtype=np.float64)
    t  = np.asarray(t,  dtype=np.float64)

    if image_size != config_size:
        print(
            f"Warning: config image size {config_size} != input image size {image_size}. "
            "Using input image size for rectification."
        )

    r1, r2, p1, p2, _, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, image_size, R, t,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
    )
    map1x, map1y = cv2.initUndistortRectifyMap(
        K1, D1, r1, p1, image_size, cv2.CV_32FC1
    )
    map2x, map2y = cv2.initUndistortRectifyMap(
        K2, D2, r2, p2, image_size, cv2.CV_32FC1
    )

    rect_left  = cv2.remap(left_bgr,  map1x, map1y, cv2.INTER_LINEAR)
    rect_right = cv2.remap(right_bgr, map2x, map2y, cv2.INTER_LINEAR)

    if not return_transforms:
        return rect_left, rect_right

    transforms = {
        "r1": r1, "r2": r2,
        "p1": p1, "p2": p2,
        "P1": p1, "P2": p2,   # aliases used by diagnose script
        "image_size": image_size,
    }
    return rect_left, rect_right, transforms
