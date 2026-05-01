#!/usr/bin/env python3
"""
Launch orb_slam3_wrapper_main with configurable dataset, config, optional --mono, and optional GDB.
Defaults match run_orbslam.sh (stereo, CHERYL, Solomon Islands 2025 dataset).

ORB-SLAM3 no longer embeds a Pangolin map viewer; orb_slam3_wrapper_main does not
accept --show-viewer. Do not pass it via --extra-args (the binary will reject unknown flags).
"""

import argparse
import shlex
import signal
import subprocess
import sys
import shutil
from pathlib import Path
from datetime import datetime


def _find_lcm_file(dataset_path):
    """Find the single .lcm file in dataset path (first level only), excluding *_short.lcm."""
    path = Path(dataset_path)
    if not path.is_dir():
        return None
    candidates = [
        f for f in path.iterdir()
        if f.suffix == ".lcm" and not f.name.endswith("_short.lcm")
    ]
    if len(candidates) != 1:
        return None
    return path / candidates[0].name


def _default_proc_path(mono, image_filter):
    """Default output path under processed_trajectories: orb_slam3/stereo or orb_slam3/mono[-filter]."""
    if not mono:
        return "orb_slam3/stereo"
    return f"orb_slam3/mono-{image_filter}" if image_filter else "orb_slam3/mono"


def _format_cmd_for_display(cmd):
    """Format command for display: binary on first line, then \\ and indented flag+value pairs."""
    if not cmd:
        return ""
    parts = [cmd[0]]
    i = 1
    while i < len(cmd):
        segment = "\\\n      " + cmd[i]
        i += 1
        while i < len(cmd) and not cmd[i].startswith("-"):
            segment += " " + cmd[i]
            i += 1
        parts.append(segment)
    return " ".join(parts)


def main():
    workspace = Path(__file__).resolve().parents[4]  # runners -> scripts -> ORB_SLAM3 -> src -> ros2_ws
    default_binary = workspace / "build/orbslam3/bin/orb_slam3_wrapper_main"
    camera_calib_path_base = workspace / "src/ORB_SLAM3/config/camera"
    config_path_base = workspace / "src/ORB_SLAM3/config"
    platform_config_base = [workspace / "src/ORB_SLAM3/config/platform"]

    parser = argparse.ArgumentParser(
        description="Run ORB-SLAM3 wrapper (stereo or mono) with configurable dataset and config."
    )
    parser.add_argument(
        "--dataset",
        default="/home/a.lunawat/data/SolomonIslands2025/RAW_DATA/r20250708_032211_SC64_lughughi_40to60",
        help="Path to the dataset directory containing a .lcm file",
    )
    parser.add_argument(
        "--vocab-path",
        default="/home/a.lunawat/data/vocabulary/orb_voc.fbow",
        help="Path to the visual bag of words vocabulary file",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Override output directory (default: auto-derived under PROCESSED_DATA)",
    )
    parser.add_argument(
        "--no-output",
        action="store_true",
        help="Do not write output (no output-dir)",
    )
    parser.add_argument(
        "--output-dir-suffix",
        default="",
        help="Suffix appended to auto-derived output directory name",
    )
    parser.add_argument(
        "--platform",
        default="CHERYL",
        help="Platform identifier (default: CHERYL)",
    )
    parser.add_argument(
        "--platform-config",
        required=True,
        help="Platform configuration YAML filename (looked up under config/platform/)",
    )
    parser.add_argument(
        "--camera-calib-file",
        required=True,
        help="Camera calibration YAML filename (looked up under config/camera/)",
    )
    parser.add_argument(
        "--orbslam3-config-name",
        default="stereo",
        help="Config name without .yaml extension (default: stereo)",
    )
    parser.add_argument(
        "--orbslam3-extractor-type",
        default="gridorb",
        choices=["orb", "brisk", "gridorb"],
        help="Feature extractor type (default: gridorb)",
    )
    parser.add_argument(
        "--mono",
        action="store_true",
        help="Run in monocular mode (default: stereo)",
    )
    parser.add_argument(
        "--image-filter",
        default="",
        help="Image channel filter for monocular mode (ignored in stereo)",
    )
    parser.add_argument(
        "--use-priors",
        action="store_true",
        help="Use navigation priors for tracking (monocular mode only)",
    )
    parser.add_argument(
        "--nav-csv",
        default="",
        help="Path to precomputed navigation trajectory CSV",
    )
    parser.add_argument(
        "--paused",
        action="store_true",
        help="Pause LCM playback at start",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=-1.0,
        help="Playback duration in seconds (default: full log)",
    )
    parser.add_argument(
        "--gdb",
        action="store_true",
        help="Run the binary under gdb (gdb --args ...)",
    )
    parser.add_argument(
        "--gdb-flags",
        default="",
        help="Extra flags passed to gdb (e.g. '-ex run')",
    )
    parser.add_argument(
        "--heaptrack",
        action="store_true",
        help="Run the binary under heaptrack for heap profiling",
    )
    parser.add_argument(
        "--heaptrack-output",
        default="",
        help="heaptrack output file prefix (default: <output-dir>/heaptrack if output-dir is set)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the command and exit without running",
    )
    parser.add_argument(
        "--extra-args",
        default="",
        help="Extra shell-quoted arguments for orb_slam3_wrapper_main",
    )
    args = parser.parse_args()

    dataset_path = Path(args.dataset)
    lcm_path = _find_lcm_file(dataset_path)
    vocab_path = Path(args.vocab_path)

    # Resolve platform config
    platform_config_path = None
    for base in platform_config_base:
        candidate = base / args.platform_config
        if candidate.is_file():
            platform_config_path = candidate.resolve()
            break

    config_name = args.orbslam3_config_name + "_" + args.orbslam3_extractor_type + ".yaml"
    if args.mono:
        config_name = config_name.replace("stereo", "mono")
    config_path = config_path_base / config_name
    camera_calib_path = camera_calib_path_base / args.camera_calib_file

    errors = []
    if not vocab_path.is_file():
        errors.append(f"Vocab file not found: {vocab_path}")
    if lcm_path is None:
        errors.append(
            f"No single non-short .lcm file found in {dataset_path} "
            "(expect exactly one .lcm file, excluding *_short.lcm)"
        )
    elif not lcm_path.is_file():
        errors.append(f"LCM file not found: {lcm_path}")
    if not config_path.is_file():
        errors.append(f"Config file not found: {config_path}")
    if not camera_calib_path.is_file():
        errors.append(f"Camera calibration file not found: {camera_calib_path}")
    if platform_config_path is None:
        errors.append(f"Platform config file not found: {args.platform_config}")
    if not default_binary.is_file():
        errors.append(f"Binary not found: {default_binary} (build the workspace first)")
    if args.heaptrack and args.gdb:
        errors.append("--heaptrack and --gdb are mutually exclusive")
    if args.heaptrack and not shutil.which("heaptrack"):
        errors.append("heaptrack not found on PATH")
    if errors:
        for e in errors:
            print(e, file=sys.stderr)
        sys.exit(1)

    # Build output directory path
    output_dir = None
    if not args.no_output:
        if args.output_dir is not None:
            output_dir = Path(args.output_dir)
        else:
            output_dir = (
                dataset_path / ".." / ".." / "PROCESSED_DATA" / dataset_path.name
                / _default_proc_path(args.mono, args.image_filter)
            ).resolve()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name_parts = [
            timestamp,
            dataset_path.parent.parent.name,
            dataset_path.name,
            args.orbslam3_extractor_type,
        ]
        if args.heaptrack:
            name_parts.append("heaptrack")
        if args.duration > 0:
            name_parts.append(f"{args.duration}sec")
        if args.output_dir_suffix:
            name_parts.append(args.output_dir_suffix)
        output_dir = output_dir / "-".join(name_parts)
        output_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(default_binary),
        "--vocab-file", str(vocab_path),
        "--camera-calib-file", str(camera_calib_path),
        "--config-file", str(config_path),
        "--lcm-log", str(lcm_path),
        "--platform", args.platform,
        "--platform-config", str(platform_config_path),
    ]
    if args.use_priors:
        cmd.append("--use-priors")
    if args.nav_csv:
        cmd.extend(["--nav-csv", args.nav_csv])
    if args.mono:
        cmd.extend(["--monocular", "--image-name-filter", args.image_filter])
    if output_dir is not None:
        cmd.extend(["--output-dir", str(output_dir)])
    if args.extra_args:
        cmd.extend(shlex.split(args.extra_args))
    if args.paused:
        cmd.append("--lcm-replay-paused")
    if args.duration > 0:
        cmd.extend(["--lcm-replay-playback-duration", str(args.duration)])

    if args.heaptrack:
        heaptrack_out = args.heaptrack_output or (
            str(output_dir / "heaptrack") if output_dir is not None else ""
        )
        heaptrack_prefix = ["heaptrack"]
        if heaptrack_out:
            heaptrack_prefix.extend(["--output", heaptrack_out])
        cmd = heaptrack_prefix + cmd

    if args.gdb:
        gdb_prefix = ["gdb", "--args"]
        if args.gdb_flags:
            gdb_prefix.extend(args.gdb_flags.split())
        cmd = gdb_prefix + cmd

    cmd_str = _format_cmd_for_display(cmd)
    print("\n-----------------------------------------------------------\n")
    print("Running Command:\n")
    print(cmd_str)
    print("\n-----------------------------------------------------------\n")

    if not args.no_output and output_dir is not None:
        try:
            shutil.copy2(platform_config_path, output_dir / platform_config_path.name)
            shutil.copy2(camera_calib_path, output_dir / camera_calib_path.name)
            shutil.copy2(config_path, output_dir / config_path.name)
            if args.nav_csv:
                nav_csv_path = Path(args.nav_csv)
                if nav_csv_path.is_file():
                    shutil.copy2(nav_csv_path, output_dir / nav_csv_path.name)
                else:
                    print(f"Warning: nav csv not found (not copied): {nav_csv_path}", file=sys.stderr)
            (output_dir / "command.txt").write_text(cmd_str + "\n")
        except Exception as e:
            print(f"Warning: failed to record config/command to output directory: {e}", file=sys.stderr)

    if args.dry_run:
        return 0

    proc = subprocess.Popen(cmd, cwd=workspace)
    try:
        return proc.wait()
    except KeyboardInterrupt:
        proc.send_signal(signal.SIGINT)
        proc.wait()
        return 130


if __name__ == "__main__":
    sys.exit(main())
