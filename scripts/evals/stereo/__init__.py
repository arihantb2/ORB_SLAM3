"""Stereo calibration library — public API.

Sub-modules
-----------
config          YAML loading: load_stereo_params, load_camera_intrinsics
geometry        Epipolar math: fundamental matrix, Sampson distance
matching        Feature detection and descriptor matching
rectify         Stereo image rectification
validation      Rigorous geometric epipolar validation
visualization   OpenCV drawing helpers for debug output
cli             Shared argparse helpers for entry-point scripts
"""

from .config import (
    extrinsic_T_c1_c2_from_static_tf_file,
    load_camera_intrinsics,
    load_stereo_params,
)
from .geometry import (
    estimate_fundamental_inliers,
    fundamental_matrix_from_rtk,
    sampson_distance,
)
from .matching import (
    detect_and_match,
    epipolar_stats,
    matches_to_points,
)
from .io import (
    demosaic_bggr_to_bgr_u8,
    load_bayer_bggr_pair_bgr_u8,
    to_u8,
)
from .rectify import rectify_pair
from .validation import RigorousValidationArgs, rigorous_epipolar_validation
from .visualization import (
    draw_matches_overlay,
    resize_to_width,
    wait_for_windows_or_key,
    window_search_debug,
)

__all__ = [
    # config
    "extrinsic_T_c1_c2_from_static_tf_file",
    "load_camera_intrinsics",
    "load_stereo_params",
    # geometry
    "estimate_fundamental_inliers",
    "fundamental_matrix_from_rtk",
    "sampson_distance",
    # matching
    "detect_and_match",
    "epipolar_stats",
    "matches_to_points",
    # rectify
    "rectify_pair",
    # io
    "to_u8",
    "demosaic_bggr_to_bgr_u8",
    "load_bayer_bggr_pair_bgr_u8",
    # validation
    "RigorousValidationArgs",
    "rigorous_epipolar_validation",
    # visualization
    "draw_matches_overlay",
    "resize_to_width",
    "wait_for_windows_or_key",
    "window_search_debug",
]
