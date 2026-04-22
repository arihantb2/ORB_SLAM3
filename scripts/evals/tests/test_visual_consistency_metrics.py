"""Unit tests for visual_consistency/metrics.py.

All tests use synthetic numpy arrays — no real image files required.
Follows the class-based pytest pattern used in test_trajectory_alignment.py.
"""

import numpy as np
import pytest

from visual_consistency.metrics import (
    bhattacharyya_distance,
    energy_of_gradient,
    phase_correlation_psr,
    spectral_centroid,
    ssim,
    zncc,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

RNG = np.random.default_rng(42)


def _random_u8(shape=(64, 64), seed=0) -> np.ndarray:
    return np.random.default_rng(seed).integers(0, 256, shape, dtype=np.uint8)


def _flat(val: int = 128, shape=(64, 64)) -> np.ndarray:
    return np.full(shape, val, dtype=np.uint8)


def _step_edge(shape=(64, 64)) -> np.ndarray:
    """Left half = 0, right half = 255."""
    img = np.zeros(shape, dtype=np.uint8)
    img[:, shape[1] // 2 :] = 255
    return img


def _blurred(img: np.ndarray, sigma: float = 5.0) -> np.ndarray:
    import cv2
    ksize = int(sigma * 6) | 1  # odd kernel
    return cv2.GaussianBlur(img, (ksize, ksize), sigma)


# ---------------------------------------------------------------------------
# TestBhattacharyya
# ---------------------------------------------------------------------------


class TestBhattacharyya:
    def test_identical_images_return_zero(self):
        img = _random_u8(seed=1)
        assert bhattacharyya_distance(img, img) == pytest.approx(0.0, abs=1e-6)

    def test_identical_flat_images_return_zero(self):
        img = _flat(100)
        assert bhattacharyya_distance(img, img) == pytest.approx(0.0, abs=1e-6)

    def test_non_overlapping_distributions_have_high_distance(self):
        """Images with disjoint intensity distributions should have high distance.

        A flat dark image (all bins at 20) vs a flat bright image (all bins at 220)
        have completely non-overlapping histograms → Bhattacharyya distance ≈ 1.
        """
        dark = _flat(20)
        bright = _flat(220)
        d = bhattacharyya_distance(dark, bright)
        assert d > 0.9, f"Expected near-1 distance for disjoint histograms, got {d}"

    def test_returns_float(self):
        img = _random_u8(seed=3)
        assert isinstance(bhattacharyya_distance(img, img), float)

    def test_similar_images_lower_than_different(self):
        base = _random_u8(seed=4)
        # Small additive noise → similar
        noisy = np.clip(base.astype(np.int16) + RNG.integers(-5, 6, base.shape), 0, 255).astype(np.uint8)
        # Uniform image → very different
        uniform = _flat(200)
        d_similar = bhattacharyya_distance(base, noisy)
        d_different = bhattacharyya_distance(base, uniform)
        assert d_similar < d_different


# ---------------------------------------------------------------------------
# TestZNCC
# ---------------------------------------------------------------------------


class TestZNCC:
    def test_identical_images_return_one(self):
        img = _random_u8(seed=5)
        assert zncc(img, img) == pytest.approx(1.0, abs=1e-6)

    def test_zero_std_returns_zero_no_crash(self):
        """Blank image has σ=0 — must return 0.0 without dividing by zero."""
        blank = _flat(0)
        random = _random_u8(seed=6)
        assert zncc(blank, random) == 0.0
        assert zncc(random, blank) == 0.0
        assert zncc(blank, blank) == 0.0

    def test_range_is_bounded(self):
        a = _random_u8(seed=7)
        b = _random_u8(seed=8)
        val = zncc(a, b)
        assert -1.0 <= val <= 1.0

    def test_anti_correlated_is_negative(self):
        """image vs inverted version should give negative ZNCC."""
        img = _random_u8(seed=9)
        inv = (255 - img.astype(np.int16)).astype(np.uint8)
        assert zncc(img, inv) < 0.0

    def test_returns_float(self):
        img = _random_u8(seed=10)
        assert isinstance(zncc(img, img), float)


# ---------------------------------------------------------------------------
# TestSSIM
# ---------------------------------------------------------------------------


class TestSSIM:
    def test_identical_images_return_one(self):
        img = _random_u8(seed=11)
        assert ssim(img, img) == pytest.approx(1.0, abs=1e-5)

    def test_value_in_valid_range(self):
        a = _random_u8(seed=12)
        b = _random_u8(seed=13)
        val = ssim(a, b)
        assert -1.0 <= val <= 1.0

    def test_similar_pair_higher_than_random_pair(self):
        base = _random_u8(seed=14)
        slightly_noisy = np.clip(
            base.astype(np.int16) + np.random.default_rng(99).integers(-3, 4, base.shape),
            0, 255,
        ).astype(np.uint8)
        completely_different = _random_u8(seed=15)
        assert ssim(base, slightly_noisy) > ssim(base, completely_different)

    def test_returns_float(self):
        img = _random_u8(seed=16)
        assert isinstance(ssim(img, img), float)


# ---------------------------------------------------------------------------
# TestEoG
# ---------------------------------------------------------------------------


class TestEoG:
    def test_flat_image_returns_zero(self):
        img = _flat(128)
        assert energy_of_gradient(img) == pytest.approx(0.0, abs=1e-8)

    def test_step_edge_is_positive(self):
        img = _step_edge()
        assert energy_of_gradient(img) > 0.0

    def test_higher_contrast_edge_gives_higher_eog(self):
        """Doubling the edge contrast should raise EoG."""
        # Low contrast: 100 | 155
        low = np.zeros((64, 64), dtype=np.uint8)
        low[:, 32:] = 55
        # High contrast: 0 | 255
        high = _step_edge()
        assert energy_of_gradient(high) > energy_of_gradient(low)

    def test_random_image_is_positive(self):
        img = _random_u8(seed=17)
        assert energy_of_gradient(img) > 0.0

    def test_normalised_by_pixel_count(self):
        """Normalisation keeps EoG in a consistent range across image sizes.

        Two random images of different sizes drawn from the same distribution
        should have similar EoG values (within a factor of 2), confirming that
        dividing by N_pixels prevents scale-dependent blow-up.
        """
        small = np.random.default_rng(99).integers(0, 256, (32, 32), dtype=np.uint8)
        large = np.random.default_rng(100).integers(0, 256, (128, 128), dtype=np.uint8)
        eog_small = energy_of_gradient(small)
        eog_large = energy_of_gradient(large)
        ratio = eog_small / eog_large
        assert 0.5 < ratio < 2.0, (
            f"EoG values differ too much: small={eog_small:.1f} large={eog_large:.1f} "
            f"ratio={ratio:.2f}"
        )

    def test_returns_float(self):
        img = _random_u8(seed=18)
        assert isinstance(energy_of_gradient(img), float)


# ---------------------------------------------------------------------------
# TestPhasePSR
# ---------------------------------------------------------------------------


class TestPhasePSR:
    def test_identical_images_positive_response(self):
        img = _random_u8(seed=19)
        r = phase_correlation_psr(img, img)
        assert r > 0.0

    def test_random_pair_lower_than_identical(self):
        img = _random_u8(seed=20)
        other = _random_u8(seed=21)
        r_same = phase_correlation_psr(img, img)
        r_diff = phase_correlation_psr(img, other)
        assert r_same > r_diff, (
            f"Expected identical>different, got identical={r_same:.4f} diff={r_diff:.4f}"
        )

    def test_returns_float(self):
        img = _random_u8(seed=22)
        assert isinstance(phase_correlation_psr(img, img), float)

    def test_non_negative(self):
        img = _random_u8(seed=23)
        other = _random_u8(seed=24)
        assert phase_correlation_psr(img, other) >= 0.0


# ---------------------------------------------------------------------------
# TestSpectralCentroid
# ---------------------------------------------------------------------------


class TestSpectralCentroid:
    def test_blurred_lower_than_sharp(self):
        """Blurring removes high frequencies → centroid moves toward DC."""
        sharp = _random_u8(shape=(128, 128), seed=25)
        blurred = _blurred(sharp, sigma=8.0)
        sc_sharp = spectral_centroid(sharp)
        sc_blurred = spectral_centroid(blurred)
        assert sc_sharp > sc_blurred, (
            f"Expected sharp ({sc_sharp:.2f}) > blurred ({sc_blurred:.2f})"
        )

    def test_flat_image_returns_zero(self):
        """Uniform image has all energy at DC → centroid at 0."""
        img = _flat(128)
        assert spectral_centroid(img) == pytest.approx(0.0, abs=1e-6)

    def test_random_image_positive(self):
        img = _random_u8(seed=26)
        assert spectral_centroid(img) > 0.0

    def test_returns_float(self):
        img = _random_u8(seed=27)
        assert isinstance(spectral_centroid(img), float)

    def test_more_blurred_lower_centroid(self):
        """Increasing blur monotonically decreases spectral centroid."""
        base = _random_u8(shape=(128, 128), seed=28)
        sc_vals = [spectral_centroid(_blurred(base, sigma=s)) for s in (1, 3, 6, 10)]
        # Each step should lower the centroid
        for i in range(len(sc_vals) - 1):
            assert sc_vals[i] > sc_vals[i + 1], (
                f"sigma={[1,3,6,10][i]} → {sc_vals[i]:.2f} should be > "
                f"sigma={[1,3,6,10][i+1]} → {sc_vals[i+1]:.2f}"
            )
