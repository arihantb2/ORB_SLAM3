"""Shared argparse helpers for stereo calibration scripts."""

from __future__ import annotations

import argparse


def add_stereo_config_args(parser: argparse.ArgumentParser) -> None:
    """Add ``--platform-config``, ``--stereo-left-frame``, ``--stereo-right-frame``."""
    parser.add_argument(
        "--platform-config",
        default=None,
        help=(
            "static_tf YAML (same as orb_slam3 --platform-config). "
            "Required when Stereo.T_c1_c2 is absent from --config."
        ),
    )
    parser.add_argument(
        "--stereo-left-frame",
        default="cam_aft",
        help="static_tf frame id for Camera1 / left camera (default: cam_aft).",
    )
    parser.add_argument(
        "--stereo-right-frame",
        default="cam_fwd",
        help="static_tf frame id for Camera2 / right camera (default: cam_fwd).",
    )
