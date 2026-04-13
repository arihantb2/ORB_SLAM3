#!/usr/bin/env python3

import argparse
import os

import matplotlib.pyplot as plt

from trajectory_errors.io import load_csv, load_xml


def parse_args():
    parser = argparse.ArgumentParser(
        description="Load a reference trajectory (.xml or .csv) and display a 3D plot."
    )
    parser.add_argument(
        "--ref",
        required=True,
        help="Reference trajectory path (.xml or .csv).",
    )
    parser.add_argument(
        "--ref-group-id",
        type=int,
        default=0,
        help="XML reference cameras group id (used only when --ref ends with .xml).",
    )
    return parser.parse_args()


def load_reference(path, group_id):
    lower = path.lower()
    if lower.endswith(".csv"):
        return load_csv(path, "Reference trajectory")
    if lower.endswith(".xml"):
        return load_xml(path, "Reference trajectory", group_id=group_id)
    raise ValueError(
        f"Reference trajectory {path} file extension not supported, try .csv or .xml"
    )


def main():
    args = parse_args()
    if not os.path.exists(args.ref):
        print(f"Reference trajectory {args.ref} not found")
        return

    df_ref = load_reference(args.ref, args.ref_group_id)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(df_ref["tx"], df_ref["ty"], df_ref["tz"], color="C1", linewidth=2.0)
    ax.set_title("Reference Trajectory (3D)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
