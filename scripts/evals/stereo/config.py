"""Camera calibration parameter loading from ORB-SLAM3 YAML configs."""

from __future__ import annotations

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as ScipyRotation


# ---------------------------------------------------------------------------
# Internal YAML helpers
# ---------------------------------------------------------------------------


def _get_required_scalar(fs: cv2.FileStorage, key: str) -> float:
    node = fs.getNode(key)
    if node.empty():
        raise ValueError(f"Missing key in config: {key}")
    return float(node.real())


def _get_optional_scalar(fs: cv2.FileStorage, key: str) -> float | None:
    node = fs.getNode(key)
    if node.empty():
        return None
    return float(node.real())


def _get_required_string(fs: cv2.FileStorage, key: str) -> str:
    node = fs.getNode(key)
    if node.empty():
        raise ValueError(f"Missing key in config: {key}")
    return str(node.string())


def _normalize_camera_type(camera_type: str) -> str:
    return "".join(c for c in camera_type.lower() if c.isalnum())


def _load_pinhole_camera(
    fs: cv2.FileStorage, prefix: str, width: int, height: int
) -> tuple[np.ndarray, np.ndarray]:
    fx = _get_optional_scalar(fs, f"{prefix}.fx")
    fy = _get_optional_scalar(fs, f"{prefix}.fy")
    if fx is not None and fy is not None:
        cx = _get_required_scalar(fs, f"{prefix}.cx")
        cy = _get_required_scalar(fs, f"{prefix}.cy")
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    else:
        f    = _get_required_scalar(fs, f"{prefix}.f")
        cx_off = _get_required_scalar(fs, f"{prefix}.cx")
        cy_off = _get_required_scalar(fs, f"{prefix}.cy")
        b1 = _get_optional_scalar(fs, f"{prefix}.b1") or 0.0
        b2 = _get_optional_scalar(fs, f"{prefix}.b2") or 0.0
        cx = (width * 0.5) + cx_off
        cy = (height * 0.5) + cy_off
        K = np.array([[f + b1, b2, cx], [0.0, f, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

    p1 = _get_optional_scalar(fs, f"{prefix}.p1") or 0.0
    p2 = _get_optional_scalar(fs, f"{prefix}.p2") or 0.0
    k1 = _get_required_scalar(fs, f"{prefix}.k1")
    k2 = _get_required_scalar(fs, f"{prefix}.k2")
    k3 = _get_optional_scalar(fs, f"{prefix}.k3") or 0.0
    k4 = _get_optional_scalar(fs, f"{prefix}.k4")
    k5 = _get_optional_scalar(fs, f"{prefix}.k5")
    k6 = _get_optional_scalar(fs, f"{prefix}.k6")

    if k4 is not None or k5 is not None or k6 is not None:
        D = np.array(
            [k1, k2, p1, p2, k3, (k4 or 0.0), (k5 or 0.0), (k6 or 0.0)],
            dtype=np.float64,
        )
    else:
        D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
    return K, D


def _chain_to_root(
    edges: dict[str, tuple[str, np.ndarray]], root: str, frame: str
) -> np.ndarray:
    """Compose T_root_frame (p_root = T * p_frame), matching static_tf::StaticTfTree."""
    T = np.eye(4, dtype=np.float64)
    current = frame
    max_depth = len(edges) + 1
    depth = 0
    while current != root:
        if current not in edges:
            raise ValueError(
                f"Unknown static_tf frame '{current}' (expected a sensor key under 'sensors:' "
                f"or the root '{root}')."
            )
        parent, Tpc = edges[current]
        T = Tpc @ T
        current = parent
        depth += 1
        if depth > max_depth:
            raise ValueError(
                f"Cycle or unreachable frame '{frame}' toward root '{root}' in static_tf YAML."
            )
    return T


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def extrinsic_T_c1_c2_from_static_tf_file(
    platform_yaml: str,
    stereo_left_frame: str = "cam_aft",
    stereo_right_frame: str = "cam_fwd",
) -> np.ndarray:
    """T_c1_c2 = lookup(left, right) with static_tf conventions (p_left = T * p_right)."""
    try:
        import yaml
    except ImportError as e:
        raise ImportError(
            "extrinsic_T_c1_c2_from_static_tf_file requires PyYAML (pip install pyyaml)."
        ) from e

    with open(platform_yaml, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    root = "body"
    if cfg.get("metadata") and cfg["metadata"].get("reference_frame"):
        root = str(cfg["metadata"]["reference_frame"])

    edges: dict[str, tuple[str, np.ndarray]] = {}
    sensors = cfg.get("sensors") or {}
    for name, s in sensors.items():
        parent = str(s["parent_frame"])
        tnode = s["T_body_sensor"]
        t = np.array(tnode["translation"], dtype=np.float64)
        q = tnode["quaternion"]
        w, x, y, z = (float(q[i]) for i in range(4))
        Rm = ScipyRotation.from_quat([x, y, z, w]).as_matrix()
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = Rm
        T[:3, 3] = t
        edges[str(name)] = (parent, T)

    if stereo_left_frame == stereo_right_frame:
        return np.eye(4, dtype=np.float64)
    Rt = _chain_to_root(edges, root, stereo_left_frame)
    Rs = _chain_to_root(edges, root, stereo_right_frame)
    return np.linalg.inv(Rt) @ Rs


def load_stereo_params(
    config_path: str,
    *,
    platform_config_path: str | None = None,
    stereo_left_frame: str = "cam_aft",
    stereo_right_frame: str = "cam_fwd",
) -> tuple:
    """Load stereo config. Returns ``(image_size, K1, D1, K2, D2, R, t, camera_type)``.

    If ``Stereo.T_c1_c2`` is absent from the YAML, pass ``platform_config_path`` to build
    T_c1_c2 from the static_tf file used by ``orb_slam3_wrapper_main --platform-config``.
    """
    fs = cv2.FileStorage(config_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f"Could not open config file: {config_path}")

    try:
        width = int(_get_required_scalar(fs, "Camera.width"))
        height = int(_get_required_scalar(fs, "Camera.height"))
        camera_type = _get_required_string(fs, "Camera.type")
        camera_type_norm = _normalize_camera_type(camera_type)
        if camera_type_norm in ("pinhole", "metashape"):
            K1, D1 = _load_pinhole_camera(fs, "Camera1", width, height)
            K2, D2 = _load_pinhole_camera(fs, "Camera2", width, height)
        else:
            raise ValueError(
                f"Unsupported Camera.type: {camera_type!r}. "
                "Only pinhole / Metashape models are supported."
            )

        t_node = fs.getNode("Stereo.T_c1_c2")
        if not t_node.empty():
            t_c1_c2 = t_node.mat()
            if t_c1_c2 is None or t_c1_c2.shape != (4, 4):
                raise ValueError("Stereo.T_c1_c2 must be a 4×4 matrix.")
        elif platform_config_path:
            t_c1_c2 = extrinsic_T_c1_c2_from_static_tf_file(
                platform_config_path, stereo_left_frame, stereo_right_frame
            )
        else:
            raise ValueError(
                "Stereo.T_c1_c2 is missing from the camera YAML. Pass platform_config_path= "
                "(static_tf YAML, same as orb_slam3 --platform-config) or restore "
                "Stereo.T_c1_c2 in the file."
            )
    finally:
        fs.release()

    R = t_c1_c2[:3, :3]
    t = t_c1_c2[:3, 3]
    return (width, height), K1, D1, K2, D2, R, t, camera_type


def load_camera_intrinsics(config_path: str) -> tuple[int, int, list[dict]]:
    """Load per-camera intrinsics without requiring extrinsics.

    Returns ``(width, height, cameras)`` where each camera entry is::

        {"name": str, "K": np.ndarray (3×3), "D": np.ndarray}

    Works for both mono (``Camera.*``) and stereo (``Camera1.*`` / ``Camera2.*``) configs.
    Supported models: ``pinhole`` / ``metashape``.
    """
    fs = cv2.FileStorage(config_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f"Could not open config file: {config_path}")

    try:
        width = int(_get_required_scalar(fs, "Camera.width"))
        height = int(_get_required_scalar(fs, "Camera.height"))
        camera_type = _get_required_string(fs, "Camera.type")
        camera_type_norm = _normalize_camera_type(camera_type)

        if camera_type_norm not in ("pinhole", "metashape"):
            raise ValueError(
                f"Unsupported Camera.type: {camera_type!r}. "
                "Only pinhole / Metashape models are supported."
            )

        # Stereo (Camera1.*) vs mono (Camera.*)
        if not fs.getNode("Camera1.f").empty() or not fs.getNode("Camera1.fx").empty():
            K1, D1 = _load_pinhole_camera(fs, "Camera1", width, height)
            K2, D2 = _load_pinhole_camera(fs, "Camera2", width, height)
            cameras = [
                {"name": "Camera 1 (Left)",  "K": K1, "D": D1},
                {"name": "Camera 2 (Right)", "K": K2, "D": D2},
            ]
        else:
            K, D = _load_pinhole_camera(fs, "Camera", width, height)
            cameras = [{"name": "Camera", "K": K, "D": D}]
    finally:
        fs.release()

    return width, height, cameras
