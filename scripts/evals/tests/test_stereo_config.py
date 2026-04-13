"""Tests for stereo/config.py — helpers and load functions."""
import numpy as np
import pytest

from stereo.config import _chain_to_root, _normalize_camera_type


# ---------------------------------------------------------------------------
# _normalize_camera_type
# ---------------------------------------------------------------------------


class TestNormalizeCameraType:
    def test_lowercase(self):
        assert _normalize_camera_type("Pinhole") == "pinhole"

    def test_strips_non_alphanumeric(self):
        assert _normalize_camera_type("pin-hole") == "pinhole"
        assert _normalize_camera_type("Pin.Hole") == "pinhole"

    def test_strips_whitespace(self):
        assert _normalize_camera_type("  pinhole  ") == "pinhole"

    def test_mixed_case_with_digits(self):
        assert _normalize_camera_type("KannalaBrandt8") == "kannalabrandt8"

    def test_metashape(self):
        assert _normalize_camera_type("Metashape") == "metashape"


# ---------------------------------------------------------------------------
# _chain_to_root
# ---------------------------------------------------------------------------


def _T(tx=0.0, ty=0.0, tz=0.0):
    """Pure-translation 4×4 transform."""
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = [tx, ty, tz]
    return T


class TestChainToRoot:
    def test_already_at_root(self):
        """frame == root → identity transform."""
        result = _chain_to_root({}, "body", "body")
        np.testing.assert_allclose(result, np.eye(4))

    def test_single_hop(self):
        T_body_sensor = _T(tx=1.0)
        edges = {"sensor": ("body", T_body_sensor)}
        result = _chain_to_root(edges, "body", "sensor")
        np.testing.assert_allclose(result, T_body_sensor)

    def test_two_hops(self):
        """T_root_c = T_root_b @ T_b_c (chain composition)."""
        T_root_b = _T(tx=1.0)
        T_b_c = _T(ty=2.0)
        edges = {
            "b": ("root", T_root_b),
            "c": ("b",    T_b_c),
        }
        result = _chain_to_root(edges, "root", "c")
        np.testing.assert_allclose(result, T_root_b @ T_b_c)

    def test_three_hops(self):
        T1 = _T(tx=1.0)
        T2 = _T(ty=2.0)
        T3 = _T(tz=3.0)
        edges = {
            "a": ("root", T1),
            "b": ("a",    T2),
            "c": ("b",    T3),
        }
        result = _chain_to_root(edges, "root", "c")
        np.testing.assert_allclose(result, T1 @ T2 @ T3)

    def test_unknown_frame_raises(self):
        with pytest.raises(ValueError, match="Unknown static_tf frame"):
            _chain_to_root({}, "body", "nonexistent_sensor")

    def test_cycle_raises(self):
        T = np.eye(4, dtype=np.float64)
        edges = {"a": ("b", T), "b": ("a", T)}
        with pytest.raises(ValueError, match="Cycle or unreachable"):
            _chain_to_root(edges, "root", "a")


# ---------------------------------------------------------------------------
# load_stereo_params / load_camera_intrinsics with a temp YAML
# ---------------------------------------------------------------------------

# Minimal valid ORB-SLAM3 stereo YAML (OpenCV FileStorage format)
_STEREO_YAML = """%YAML:1.0
---
Camera.type: "pinhole"
Camera.width: 640
Camera.height: 480
Camera1.fx: 500.0
Camera1.fy: 500.0
Camera1.cx: 320.0
Camera1.cy: 240.0
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera2.fx: 480.0
Camera2.fy: 480.0
Camera2.cx: 310.0
Camera2.cy: 235.0
Camera2.k1: -0.1
Camera2.k2:  0.02
Stereo.T_c1_c2: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1., 0., 0., 0.12,
           0., 1., 0., 0.,
           0., 0., 1., 0.,
           0., 0., 0., 1. ]
"""

_MONO_YAML = """%YAML:1.0
---
Camera.type: "pinhole"
Camera.width: 1280
Camera.height: 720
Camera.fx: 800.0
Camera.fy: 800.0
Camera.cx: 640.0
Camera.cy: 360.0
Camera.k1: -0.15
Camera.k2:  0.05
"""


@pytest.fixture()
def stereo_yaml(tmp_path):
    p = tmp_path / "stereo.yaml"
    p.write_text(_STEREO_YAML)
    return str(p)


@pytest.fixture()
def mono_yaml(tmp_path):
    p = tmp_path / "mono.yaml"
    p.write_text(_MONO_YAML)
    return str(p)


class TestLoadStereoParams:
    def test_tuple_length(self, stereo_yaml):
        from stereo.config import load_stereo_params
        result = load_stereo_params(stereo_yaml)
        assert len(result) == 8

    def test_image_size(self, stereo_yaml):
        from stereo.config import load_stereo_params
        (width, height), *_ = load_stereo_params(stereo_yaml)
        assert (width, height) == (640, 480)

    def test_intrinsics_shape(self, stereo_yaml):
        from stereo.config import load_stereo_params
        _, K1, D1, K2, D2, R, t, _ = load_stereo_params(stereo_yaml)
        assert K1.shape == (3, 3)
        assert K2.shape == (3, 3)
        assert D1.ndim == 1 and len(D1) >= 4
        assert D2.ndim == 1 and len(D2) >= 4

    def test_K1_values(self, stereo_yaml):
        from stereo.config import load_stereo_params
        _, K1, _, _, _, _, _, _ = load_stereo_params(stereo_yaml)
        assert K1[0, 0] == pytest.approx(500.0)
        assert K1[0, 2] == pytest.approx(320.0)
        assert K1[1, 2] == pytest.approx(240.0)

    def test_extrinsics(self, stereo_yaml):
        from stereo.config import load_stereo_params
        _, _, _, _, _, R, t, _ = load_stereo_params(stereo_yaml)
        assert R.shape == (3, 3)
        assert t.shape == (3,)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-10)
        assert t[0] == pytest.approx(0.12)

    def test_camera_type_string(self, stereo_yaml):
        from stereo.config import load_stereo_params
        *_, camera_type = load_stereo_params(stereo_yaml)
        assert "pinhole" in camera_type.lower()

    def test_missing_file_raises(self):
        from stereo.config import load_stereo_params
        with pytest.raises(ValueError, match="Could not open"):
            load_stereo_params("/nonexistent/path/config.yaml")


class TestLoadCameraIntrinsics:
    def test_stereo_returns_two_cameras(self, stereo_yaml):
        from stereo.config import load_camera_intrinsics
        width, height, cameras = load_camera_intrinsics(stereo_yaml)
        assert (width, height) == (640, 480)
        assert len(cameras) == 2

    def test_stereo_camera_dict_keys(self, stereo_yaml):
        from stereo.config import load_camera_intrinsics
        _, _, cameras = load_camera_intrinsics(stereo_yaml)
        for cam in cameras:
            assert "K" in cam
            assert "D" in cam
            assert "name" in cam
            assert cam["K"].shape == (3, 3)

    def test_stereo_cam2_distortion(self, stereo_yaml):
        """Camera 2 has k1=-0.1, k2=0.02 in the fixture."""
        from stereo.config import load_camera_intrinsics
        _, _, cameras = load_camera_intrinsics(stereo_yaml)
        D2 = cameras[1]["D"]
        assert D2[0] == pytest.approx(-0.1, abs=1e-6)
        assert D2[1] == pytest.approx(0.02, abs=1e-6)

    def test_mono_returns_one_camera(self, mono_yaml):
        from stereo.config import load_camera_intrinsics
        width, height, cameras = load_camera_intrinsics(mono_yaml)
        assert (width, height) == (1280, 720)
        assert len(cameras) == 1

    def test_mono_K_values(self, mono_yaml):
        from stereo.config import load_camera_intrinsics
        _, _, cameras = load_camera_intrinsics(mono_yaml)
        K = cameras[0]["K"]
        assert K[0, 0] == pytest.approx(800.0)
        assert K[0, 2] == pytest.approx(640.0)
        assert K[1, 2] == pytest.approx(360.0)
