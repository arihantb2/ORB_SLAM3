"""Tests for stereo/geometry.py — fundamental matrix and Sampson distance."""
import numpy as np
import pytest

from stereo.geometry import (
    estimate_fundamental_inliers,
    fundamental_matrix_from_rtk,
    sampson_distance,
)


def _K(fx=500.0, cx=320.0, cy=240.0):
    return np.array([[fx, 0, cx], [0, fx, cy], [0, 0, 1]], dtype=np.float64)


def _horizontal_rig():
    """K1==K2, R=I, pure x-translation (standard horizontal stereo)."""
    K = _K()
    R = np.eye(3, dtype=np.float64)
    t = np.array([0.1, 0.0, 0.0], dtype=np.float64)
    return K, R, t


class TestFundamentalMatrix:
    def test_shape(self):
        K, R, t = _horizontal_rig()
        F = fundamental_matrix_from_rtk(K, K, R, t)
        assert F.shape == (3, 3)

    def test_rank_two(self):
        """A valid F must be rank-2."""
        K, R, t = _horizontal_rig()
        F = fundamental_matrix_from_rtk(K, K, R, t)
        assert np.linalg.matrix_rank(F) == 2

    def test_epipolar_constraint_same_row(self):
        """For pure x-baseline, points on the same image row satisfy x2ᵀ F x1 = 0."""
        K, R, t = _horizontal_rig()
        F = fundamental_matrix_from_rtk(K, K, R, t)

        pts1 = np.array([[100.0, 200.0], [300.0, 150.0], [250.0, 350.0]])
        pts2 = pts1 + [50.0, 0.0]  # same row, different column
        x1 = np.c_[pts1, np.ones(len(pts1))]
        x2 = np.c_[pts2, np.ones(len(pts2))]
        residuals = np.einsum("ni,ij,nj->n", x2, F, x1)
        np.testing.assert_allclose(residuals, 0.0, atol=1e-10)

    def test_violates_constraint_when_rows_differ(self):
        """A point-pair with different y-coordinates should NOT satisfy the constraint."""
        K, R, t = _horizontal_rig()
        F = fundamental_matrix_from_rtk(K, K, R, t)

        x1 = np.array([[100.0, 200.0, 1.0]])
        x2 = np.array([[150.0, 250.0, 1.0]])  # y differs by 50 px
        residual = float(np.einsum("ni,ij,nj->n", x2, F, x1)[0])
        assert abs(residual) > 1e-4

    def test_zero_translation_gives_zero_F(self):
        """Degenerate case: zero baseline → E=0 → F=0."""
        K = _K()
        F = fundamental_matrix_from_rtk(K, K, np.eye(3), np.zeros(3))
        np.testing.assert_allclose(F, 0.0, atol=1e-15)

    def test_different_intrinsics(self):
        """F enforces the epipolar constraint for proper correspondences with K1 ≠ K2."""
        K1 = _K(fx=500.0, cx=320.0, cy=240.0)
        K2 = _K(fx=600.0, cx=310.0, cy=230.0)
        R = np.eye(3, dtype=np.float64)
        b = 0.12
        t = np.array([b, 0.0, 0.0], dtype=np.float64)
        F = fundamental_matrix_from_rtk(K1, K2, R, t)

        # Build true correspondences from normalized image coords.
        # For R=I, t=[b,0,0] and a unit-depth point (x_n, y_n, 1):
        #   cam1 pixel: (fx1*x_n + cx1, fy1*y_n + cy1)
        #   cam2 pixel: (fx2*(x_n+b) + cx2, fy2*y_n + cy2)
        rng = np.random.default_rng(10)
        x_n = rng.uniform(-0.4, 0.4, 5)
        y_n = rng.uniform(-0.3, 0.3, 5)
        pts1 = np.stack([K1[0, 0] * x_n + K1[0, 2], K1[1, 1] * y_n + K1[1, 2]], axis=1)
        pts2 = np.stack([K2[0, 0] * (x_n + b) + K2[0, 2], K2[1, 1] * y_n + K2[1, 2]], axis=1)
        x1 = np.c_[pts1, np.ones(len(pts1))]
        x2 = np.c_[pts2, np.ones(len(pts2))]
        residuals = np.einsum("ni,ij,nj->n", x2, F, x1)
        np.testing.assert_allclose(residuals, 0.0, atol=1e-10)


class TestSampsonDistance:
    def test_output_shape(self):
        K, R, t = _horizontal_rig()
        F = fundamental_matrix_from_rtk(K, K, R, t)
        pts1 = np.random.default_rng(0).uniform(50, 600, (8, 2))
        pts2 = pts1 + [30.0, 0.0]
        d = sampson_distance(F, pts1, pts2)
        assert d.shape == (8,)

    def test_zero_for_epipolar_inliers(self):
        """Points exactly on the epipolar line → Sampson distance ≈ 0."""
        K, R, t = _horizontal_rig()
        F = fundamental_matrix_from_rtk(K, K, R, t)
        pts1 = np.array([[100.0, 200.0], [300.0, 150.0], [200.0, 320.0]])
        pts2 = pts1 + [50.0, 0.0]  # same row → on epipolar line
        d = sampson_distance(F, pts1, pts2)
        np.testing.assert_allclose(d, 0.0, atol=1e-10)

    def test_positive_for_outliers(self):
        """Points off the epipolar line → strictly positive Sampson distance."""
        K, R, t = _horizontal_rig()
        F = fundamental_matrix_from_rtk(K, K, R, t)
        pts1 = np.array([[100.0, 200.0]], dtype=np.float64)
        pts2 = np.array([[150.0, 260.0]], dtype=np.float64)  # 60 px y-error
        d = sampson_distance(F, pts1, pts2)
        assert d[0] > 0.01

    def test_non_negative(self):
        """Sampson distance should always be ≥ 0 (it's a squared quantity)."""
        K, R, t = _horizontal_rig()
        F = fundamental_matrix_from_rtk(K, K, R, t)
        rng = np.random.default_rng(42)
        pts1 = rng.uniform(0, 640, (20, 2))
        pts2 = rng.uniform(0, 640, (20, 2))
        d = sampson_distance(F, pts1, pts2)
        assert np.all(d >= 0)


class TestEstimateFundamentalInliers:
    def test_too_few_points_returns_none(self):
        """< 8 points → must return (None, all-False mask)."""
        rng = np.random.default_rng(0)
        pts = rng.uniform(0, 640, (7, 2))
        F, mask = estimate_fundamental_inliers(pts, pts.copy(), ransac_reproj_thresh=1.0)
        assert F is None
        assert mask.shape == (7,)
        assert not np.any(mask)

    def test_empty_points(self):
        pts = np.zeros((0, 2), dtype=np.float64)
        F, mask = estimate_fundamental_inliers(pts, pts, ransac_reproj_thresh=1.0)
        assert F is None
        assert len(mask) == 0

    def test_returns_valid_F_shape_with_inliers(self):
        """Enough coplanar-free inliers → returns a (3,3) F and boolean mask."""
        rng = np.random.default_rng(1)
        K = _K()
        R = np.eye(3)
        t = np.array([0.15, 0.0, 0.0])
        F_true = fundamental_matrix_from_rtk(K, K, R, t)

        pts1 = rng.uniform(50, 590, (60, 2))
        # Generate clean correspondences on the epipolar line (same row)
        pts2 = pts1 + rng.uniform(5, 80, (60, 1)) * [1.0, 0.0]
        # Add a little noise so points aren't perfectly coplanar
        pts2 += rng.normal(0, 0.5, pts2.shape)

        F, mask = estimate_fundamental_inliers(pts1, pts2, ransac_reproj_thresh=2.0)
        assert F is not None
        assert F.shape == (3, 3)
        assert mask.dtype == bool
        assert mask.shape == (60,)
        assert np.sum(mask) >= 8
