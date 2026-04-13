"""Tests for stereo/matching.py — epipolar_stats and matches_to_points."""
import math

import cv2
import numpy as np
import pytest

from stereo.matching import epipolar_stats, matches_to_points


def _kps(coords):
    """Create a list of cv2.KeyPoint from an (N, 2) array of (x, y)."""
    return [cv2.KeyPoint(float(x), float(y), 1.0) for x, y in coords]


def _matches(pairs):
    """Create DMatch objects from (query_idx, train_idx, distance) triples."""
    return [cv2.DMatch(int(q), int(t), float(d)) for q, t, d in pairs]


class TestEpipolarStats:
    def test_empty_matches_returns_zeros_and_nans(self):
        stats = epipolar_stats([], [], [])
        assert stats["count"] == 0
        assert math.isnan(stats["mean_abs_dy"])
        assert math.isnan(stats["median_abs_dy"])
        assert math.isnan(stats["max_abs_dy"])

    def test_zero_dy_when_same_row(self):
        """Correspondences on the same row → all dy metrics = 0."""
        pts1 = [[100, 200], [300, 150], [250, 350]]
        pts2 = [[150, 200], [350, 150], [300, 350]]
        kp1 = _kps(pts1)
        kp2 = _kps(pts2)
        m = _matches([(0, 0, 0.1), (1, 1, 0.2), (2, 2, 0.3)])
        stats = epipolar_stats(kp1, kp2, m)
        assert stats["count"] == 3
        assert stats["mean_abs_dy"] == pytest.approx(0.0)
        assert stats["median_abs_dy"] == pytest.approx(0.0)
        assert stats["max_abs_dy"] == pytest.approx(0.0)

    def test_known_dy_values(self):
        """dy values are [1, 3, 5] → mean=3, median=3, max=5."""
        # dy = kp1.y - kp2.y
        pts1 = [[100, 201], [200, 203], [300, 205]]
        pts2 = [[150, 200], [250, 200], [350, 200]]  # kp2.y all 200
        kp1 = _kps(pts1)
        kp2 = _kps(pts2)
        m = _matches([(0, 0, 0.1), (1, 1, 0.2), (2, 2, 0.3)])
        stats = epipolar_stats(kp1, kp2, m)
        assert stats["count"] == 3
        assert stats["mean_abs_dy"] == pytest.approx(3.0)
        assert stats["median_abs_dy"] == pytest.approx(3.0)
        assert stats["max_abs_dy"] == pytest.approx(5.0)

    def test_abs_dy_ignores_sign(self):
        """Negative dy should be treated the same as positive."""
        pts1 = [[100, 195]]   # y = 195
        pts2 = [[150, 200]]   # y = 200 → dy = -5, |dy| = 5
        kp1 = _kps(pts1)
        kp2 = _kps(pts2)
        m = _matches([(0, 0, 0.1)])
        stats = epipolar_stats(kp1, kp2, m)
        assert stats["max_abs_dy"] == pytest.approx(5.0)

    def test_single_match(self):
        pts1 = [[100, 204]]
        pts2 = [[120, 200]]
        kp1 = _kps(pts1)
        kp2 = _kps(pts2)
        m = _matches([(0, 0, 0.5)])
        stats = epipolar_stats(kp1, kp2, m)
        assert stats["count"] == 1
        assert stats["mean_abs_dy"] == pytest.approx(4.0)
        assert stats["median_abs_dy"] == pytest.approx(4.0)
        assert stats["max_abs_dy"] == pytest.approx(4.0)


class TestMatchesToPoints:
    def test_extracts_correct_coords(self):
        pts1_coords = [[100.0, 200.0], [300.0, 150.0], [50.0, 400.0]]
        pts2_coords = [[110.0, 200.0], [310.0, 150.0], [60.0, 400.0]]
        kp1 = _kps(pts1_coords)
        kp2 = _kps(pts2_coords)
        m = _matches([(0, 0, 0.1), (1, 1, 0.2), (2, 2, 0.3)])
        p1, p2 = matches_to_points(kp1, kp2, m)
        np.testing.assert_allclose(p1, pts1_coords)
        np.testing.assert_allclose(p2, pts2_coords)

    def test_output_shape(self):
        n = 10
        rng = np.random.default_rng(7)
        coords = rng.uniform(0, 640, (n * 2, 2))
        kp1 = _kps(coords[:n])
        kp2 = _kps(coords[n:])
        m = _matches([(i, i, 0.1) for i in range(n)])
        p1, p2 = matches_to_points(kp1, kp2, m)
        assert p1.shape == (n, 2)
        assert p2.shape == (n, 2)

    def test_respects_match_indices(self):
        """queryIdx / trainIdx pairing should be honoured, not position order."""
        kp1 = _kps([[10, 20], [30, 40], [50, 60]])
        kp2 = _kps([[100, 200], [300, 400], [500, 600]])
        # Match kp1[2] → kp2[0], kp1[0] → kp2[2]
        m = _matches([(2, 0, 0.1), (0, 2, 0.2)])
        p1, p2 = matches_to_points(kp1, kp2, m)
        np.testing.assert_allclose(p1[0], [50, 60])
        np.testing.assert_allclose(p2[0], [100, 200])
        np.testing.assert_allclose(p1[1], [10, 20])
        np.testing.assert_allclose(p2[1], [500, 600])

    def test_float64_dtype(self):
        kp1 = _kps([[1, 2]])
        kp2 = _kps([[3, 4]])
        m = _matches([(0, 0, 0.1)])
        p1, p2 = matches_to_points(kp1, kp2, m)
        assert p1.dtype == np.float64
        assert p2.dtype == np.float64
