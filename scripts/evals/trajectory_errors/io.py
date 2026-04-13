import json
import os
import re
from datetime import datetime, timedelta

import pandas as pd
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as R


def load_csv(path, label):
    df = pd.read_csv(path)
    required_cols = ["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f"{label} missing columns: {', '.join(missing)}")
    return df


_LABEL_TIME_RE = re.compile(r"^PR_(\d{8})_(\d{6})_(\d{3})_")


def _timestamp_from_label_local_epoch_seconds(label: str) -> float:
    """
    Parses labels like: PR_20251107_032204_766_AC16
    Returns Unix epoch seconds assuming the label time is in local timezone.
    """
    m = _LABEL_TIME_RE.match(label or "")
    if not m:
        raise ValueError(
            f"Unsupported camera label format for timestamp parsing: {label!r}"
        )
    yyyymmdd, hhmmss, ms = m.group(1), m.group(2), m.group(3)
    dt = datetime.strptime(yyyymmdd + hhmmss, "%Y%m%d%H%M%S") + timedelta(
        milliseconds=int(ms)
    )
    return dt.timestamp()


def _parse_transform_4x4(transform_text: str) -> np.ndarray:
    parts = (transform_text or "").strip().split()
    if len(parts) != 16:
        raise ValueError(f"Expected 16 floats in <transform>, got {len(parts)}")
    vals = np.array([float(x) for x in parts], dtype=float)
    return vals.reshape((4, 4))


def load_xml(path, label, group_id=0):
    """
    Load reference trajectory from a Metashape-style XML.

    Extracts camera poses from camera <transform>. Cameras without a non-empty
    <transform> are skipped. Supports both grouped cameras
    (./chunk/cameras/group[@id='<group_id>']/camera) and flat cameras
    (./chunk/cameras/camera). Timestamps come from camera labels.

    Returns a DataFrame with columns:
      timestamp, tx, ty, tz, qx, qy, qz, qw
    """
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as e:
        raise ValueError(f"{label} XML parse error: {e}") from e

    group = root.find(f".//chunk/cameras/group[@id='{group_id}']")
    if group is not None:
        cameras = group.findall("./camera")
    else:
        cameras = root.findall(".//chunk/cameras/camera")
        if not cameras:
            raise ValueError(
                f"{label} missing cameras group id={group_id} and no flat cameras in XML"
            )

    rows = []
    for cam in cameras:
        cam_label = cam.get("label", "")
        transform_el = cam.find("./transform")
        if transform_el is None or not (transform_el.text or "").strip():
            continue

        timestamp = _timestamp_from_label_local_epoch_seconds(cam_label)
        T = _parse_transform_4x4(transform_el.text)
        Rm = T[:3, :3]
        t = T[:3, 3]
        quat_xyzw = R.from_matrix(Rm).as_quat()

        rows.append(
            {
                "timestamp": float(timestamp),
                "tx": float(t[0]),
                "ty": float(t[1]),
                "tz": float(t[2]),
                "qx": float(quat_xyzw[0]),
                "qy": float(quat_xyzw[1]),
                "qz": float(quat_xyzw[2]),
                "qw": float(quat_xyzw[3]),
            }
        )

    if not rows:
        raise ValueError(
            f"{label} contains no cameras with non-empty <transform> in group id={group_id}"
        )

    df = pd.DataFrame(rows).sort_values("timestamp")
    required_cols = ["timestamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f"{label} missing columns: {', '.join(missing)}")
    return df


def write_json(path, payload):
    """Write payload dict as minified JSON. Creates parent directory if needed."""
    dirname = os.path.dirname(path)
    if dirname:
        os.makedirs(dirname, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, separators=(',', ':'))


def write_trajectory_csv(path, timestamps, positions, quaternions, extra_columns=None):
    """
    Write a trajectory to CSV in the same format as load_csv expects:
    timestamp, tx, ty, tz, qx, qy, qz, qw.
    positions: (N, 3), quaternions: (N, 4) in xyzw order.
    extra_columns: optional dict of {column_name: array} appended after pose columns.
    Creates parent directory if needed.
    """
    dirname = os.path.dirname(path)
    if dirname:
        os.makedirs(dirname, exist_ok=True)
    df = pd.DataFrame(
        {
            "timestamp": timestamps,
            "tx": positions[:, 0],
            "ty": positions[:, 1],
            "tz": positions[:, 2],
            "qx": quaternions[:, 0],
            "qy": quaternions[:, 1],
            "qz": quaternions[:, 2],
            "qw": quaternions[:, 3],
        }
    )
    if extra_columns:
        for col_name, col_data in extra_columns.items():
            df[col_name] = col_data
    df.to_csv(path, index=False)
