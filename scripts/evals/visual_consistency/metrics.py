"""Six visual consistency metrics for sequential grayscale image pairs.

All public functions accept uint8 (H, W) numpy arrays and return a single float.
They are pure functions with no side effects.
"""

import cv2
import numpy as np
from skimage.metrics import structural_similarity as _ssim_fn


def bhattacharyya_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Bhattacharyya distance between normalised 256-bin grayscale histograms.

    Range: [0, ∞). Lower = more similar intensity distributions.
    High values indicate strobe flicker or sudden exposure jumps.
    """
    hist_a = cv2.calcHist([a], [0], None, [256], [0, 256])
    hist_b = cv2.calcHist([b], [0], None, [256], [0, 256])
    cv2.normalize(hist_a, hist_a, alpha=1.0, norm_type=cv2.NORM_L1)
    cv2.normalize(hist_b, hist_b, alpha=1.0, norm_type=cv2.NORM_L1)
    return float(cv2.compareHist(hist_a, hist_b, cv2.HISTCMP_BHATTACHARYYA))


def zncc(a: np.ndarray, b: np.ndarray) -> float:
    """Zero-mean normalised cross-correlation (ZNCC) between two frames.

    Measures texture similarity regardless of global brightness change —
    critical for evaluating feature consistency under varying strobe intensity.

    Range: [-1, 1]. 1 = identical texture. Returns 0.0 when either frame
    has near-zero variance (e.g. blank or saturated image) to avoid division
    by zero.
    """
    a_f = a.astype(np.float64)
    b_f = b.astype(np.float64)
    sigma_a = a_f.std()
    sigma_b = b_f.std()
    if sigma_a < 1e-8 or sigma_b < 1e-8:
        return 0.0
    a_zm = a_f - a_f.mean()
    b_zm = b_f - b_f.mean()
    return float(np.sum(a_zm * b_zm) / (a_f.size * sigma_a * sigma_b))


def ssim(a: np.ndarray, b: np.ndarray) -> float:
    """Structural Similarity Index (SSIM) between two frames.

    Perception-based comparison of luminance, contrast, and structure.
    Drops in SSIM indicate transient noise (marine snow) or motion blur.

    Range: [-1, 1]. Values near 1 indicate high perceptual similarity.
    """
    return float(
        _ssim_fn(
            a.astype(np.float64) / 255.0,
            b.astype(np.float64) / 255.0,
            data_range=1.0,
        )
    )


def energy_of_gradient(img: np.ndarray) -> float:
    """Mean squared gradient energy (Sobel x+y), normalised by pixel count.

    EoG = Σ(Gx² + Gy²) / N_pixels

    Range: [0, ∞). Near-zero values suggest a turbid silt cloud is obscuring
    the seafloor rather than sharp bottom texture being illuminated.
    """
    gx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=3)
    return float(np.sum(gx ** 2 + gy ** 2) / img.size)


def phase_correlation_psr(a: np.ndarray, b: np.ndarray) -> float:
    """Phase-correlation peak magnitude (PSR) between two frames.

    cv2.phaseCorrelate returns the peak of the normalised cross-power
    spectrum.  Higher = stronger geometric alignment between frames.
    A weak peak indicates significant visual decorrelation or backscatter.

    Inputs are cast to float64 as required by cv2.phaseCorrelate.
    """
    _, response = cv2.phaseCorrelate(
        a.astype(np.float64),
        b.astype(np.float64),
    )
    return float(response)


def spectral_centroid(img: np.ndarray) -> float:
    """Weighted mean radial frequency of the 2D power spectrum.

    Spectral centroid = Σ(r · P(r)) / Σ(P(r))
    where r = √(u² + v²) is the radial distance from DC in the shifted FFT,
    and P(r) is the power at that frequency bin.

    Higher values = more high-frequency content (sharper image).
    A drift toward lower values signals blur or loss of fine seafloor detail.
    Result is in pixel units (radial distance in the FFT plane).
    """
    f_shifted = np.fft.fftshift(np.fft.fft2(img.astype(np.float64)))
    power = np.abs(f_shifted) ** 2
    total_power = power.sum()
    if total_power < 1e-12:
        return 0.0
    H, W = img.shape
    cy, cx = H // 2, W // 2
    y_idx, x_idx = np.mgrid[0:H, 0:W]
    r = np.sqrt((y_idx - cy) ** 2 + (x_idx - cx) ** 2)
    return float(np.sum(r * power) / total_power)
