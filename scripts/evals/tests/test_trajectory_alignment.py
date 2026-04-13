"""Tests for trajectory_errors/alignment.py — align_umeyama and split_segments_by_gap."""
import numpy as np
import pytest

from trajectory_errors.alignment import align_umeyama, split_segments_by_gap


# ---------------------------------------------------------------------------
# split_segments_by_gap
# ---------------------------------------------------------------------------


class TestSplitSegmentsByGap:
    def test_empty(self):
        assert split_segments_by_gap(np.array([]), 1.0) == []

    def test_single_element(self):
        assert split_segments_by_gap(np.array([0.0]), 1.0) == [(0, 1)]

    def test_no_gap(self):
        t = np.array([0.0, 0.1, 0.2, 0.3])
        assert split_segments_by_gap(t, 0.5) == [(0, 4)]

    def test_one_gap(self):
        t = np.array([0.0, 0.1, 1.5, 1.6])
        segs = split_segments_by_gap(t, 1.0)
        assert segs == [(0, 2), (2, 4)]

    def test_gap_exactly_at_threshold_is_not_split(self):
        """Gap == threshold should NOT be split (condition is strictly greater)."""
        t = np.array([0.0, 1.0, 2.0])
        segs = split_segments_by_gap(t, 1.0)
        assert segs == [(0, 3)]

    def test_gap_just_above_threshold_splits(self):
        t = np.array([0.0, 1.001, 2.002])
        segs = split_segments_by_gap(t, 1.0)
        assert segs == [(0, 1), (1, 2), (2, 3)]

    def test_multiple_gaps(self):
        t = np.array([0.0, 0.1, 2.0, 2.1, 5.0])
        segs = split_segments_by_gap(t, 1.0)
        assert segs == [(0, 2), (2, 4), (4, 5)]

    def test_all_gaps(self):
        """Every consecutive pair exceeds the threshold → one segment per point."""
        t = np.array([0.0, 2.0, 4.0])
        segs = split_segments_by_gap(t, 1.0)
        assert segs == [(0, 1), (1, 2), (2, 3)]

    def test_segments_cover_all_points(self):
        """Union of all segments must equal [0, N)."""
        rng = np.random.default_rng(42)
        t = np.cumsum(rng.exponential(0.5, 50))
        segs = split_segments_by_gap(t, 1.5)
        covered = set()
        for s, e in segs:
            covered.update(range(s, e))
        assert covered == set(range(len(t)))


# ---------------------------------------------------------------------------
# align_umeyama
# ---------------------------------------------------------------------------


class TestAlignUmeyama:
    def test_identity_no_scale(self):
        """model == data → s=1, R=I, t=0."""
        rng = np.random.default_rng(0)
        data = rng.standard_normal((40, 3))
        s, Rot, t = align_umeyama(data, data, with_scale=False)
        assert s == pytest.approx(1.0)
        np.testing.assert_allclose(Rot, np.eye(3), atol=1e-8)
        np.testing.assert_allclose(t, np.zeros(3), atol=1e-8)

    def test_pure_translation_recovered(self):
        """model = data + c → R=I, t=c, s=1."""
        rng = np.random.default_rng(1)
        data = rng.standard_normal((50, 3))
        c = np.array([3.0, -1.5, 2.0])
        model = data + c
        s, Rot, t = align_umeyama(model, data, with_scale=False)
        assert s == pytest.approx(1.0)
        np.testing.assert_allclose(Rot, np.eye(3), atol=1e-6)
        np.testing.assert_allclose(t, c, atol=1e-6)

    def test_scale_and_translation_recovered(self):
        """model = true_s * data + c → recovered s ≈ true_s, t ≈ c."""
        rng = np.random.default_rng(2)
        N = 60
        data = rng.standard_normal((N, 3))
        true_s = 2.5
        c = np.array([1.0, -2.0, 3.0])
        model = true_s * data + c
        s, Rot, t = align_umeyama(model, data, with_scale=True)
        assert s == pytest.approx(true_s, rel=1e-5)
        np.testing.assert_allclose(Rot, np.eye(3), atol=1e-5)
        np.testing.assert_allclose(t, c, atol=1e-5)

    def test_with_scale_false_forces_unit_scale(self):
        rng = np.random.default_rng(3)
        data = rng.standard_normal((30, 3))
        model = 3.0 * data  # scaled
        s, _, _ = align_umeyama(model, data, with_scale=False)
        assert s == 1.0  # forced, not recovered

    def test_rotation_recovered(self):
        """Pure rotation (no translation, no scale) → Rot ≈ true R."""
        from scipy.spatial.transform import Rotation as ScipyR
        rng = np.random.default_rng(4)
        # Zero-mean data
        data = rng.standard_normal((50, 3))
        data -= data.mean(axis=0)
        true_R = ScipyR.from_euler("z", 30, degrees=True).as_matrix()
        model = data @ true_R.T
        s, Rot, t = align_umeyama(model, data, with_scale=False)
        assert s == pytest.approx(1.0)
        np.testing.assert_allclose(Rot, true_R, atol=1e-6)
        np.testing.assert_allclose(t, np.zeros(3), atol=1e-6)

    def test_degenerate_single_point_no_crash(self):
        """Single point (zero variance) must not produce NaN."""
        data = np.array([[1.0, 2.0, 3.0]])
        s, Rot, t = align_umeyama(data, data, with_scale=True)
        assert np.isfinite(s)
        assert np.all(np.isfinite(Rot))
        assert np.all(np.isfinite(t))

    def test_aligned_residuals_near_zero(self):
        """After alignment, s * R @ data + t should match model closely."""
        rng = np.random.default_rng(5)
        data = rng.standard_normal((30, 3))
        true_s = 1.8
        c = np.array([-0.5, 1.0, 2.5])
        model = true_s * data + c
        s, Rot, t = align_umeyama(model, data, with_scale=True)
        aligned = s * (data @ Rot.T) + t
        np.testing.assert_allclose(aligned, model, atol=1e-8)
