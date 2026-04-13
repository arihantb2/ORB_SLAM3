"""Image loading helpers for stereo scripts.

Assumptions:
- Raw inputs are single-channel Bayer mosaics in BGGR order.
- Most RAW datasets are stored as 8/16-bit TIFF/PNG; the scripts operate on 8-bit
  images for OpenCV feature detectors and consistent visualization.
"""

from __future__ import annotations

import cv2
import numpy as np


def to_u8(img: np.ndarray) -> np.ndarray:
    if img.dtype == np.uint8:
        return img
    if img.dtype == np.uint16:
        return (img >> 8).astype(np.uint8, copy=False)
    return cv2.convertScaleAbs(img)


def demosaic_bggr_to_bgr_u8(bayer_mosaic: np.ndarray) -> np.ndarray:
    if bayer_mosaic is None:
        raise ValueError("Input image is None.")
    if bayer_mosaic.ndim != 2:
        raise ValueError(f"Expected single-channel Bayer mosaic, got shape {bayer_mosaic.shape}.")
    bgr = cv2.cvtColor(bayer_mosaic, cv2.COLOR_BayerBG2BGR)
    return to_u8(bgr)


def load_bayer_bggr_pair_bgr_u8(left_path: str, right_path: str) -> tuple[np.ndarray, np.ndarray]:
    left_raw = cv2.imread(left_path, cv2.IMREAD_UNCHANGED)
    right_raw = cv2.imread(right_path, cv2.IMREAD_UNCHANGED)
    if left_raw is None or right_raw is None:
        raise ValueError("Could not read one or both input images.")
    if left_raw.shape[:2] != right_raw.shape[:2]:
        raise ValueError("Left and right images must have identical dimensions.")
    return demosaic_bggr_to_bgr_u8(left_raw), demosaic_bggr_to_bgr_u8(right_raw)

