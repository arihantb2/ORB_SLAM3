# visual_consistency

Pre-flight health check for AUV image sequences captured under strobe illumination.
Quantifies photometric, structural, and frequency-domain stability across a time series
so you can diagnose *why* a VO run fails before committing to the full pipeline.

## CLI quick start

```bash
# All images in a directory, save outputs
vis-check images-AC/ -o results/

# TIF raw files only, limit to first 200 frames
vis-check i20251003_212212/ --ext tif --max-images 200 -o results/

# Preview first 50 frames interactively
vis-check images-AC/ --max-images 50 --show
```

## Outputs

| File | Contents |
|------|----------|
| `consistency_metrics.csv` | Per-pair values for all six metrics |
| `consistency_dashboard.png` | 3×2 time-series plot, one panel per metric |
| `strobe_stability_heatmap.png` | Intensity distribution heatmap + delta analysis |
| `analysis_summary.json` | Per-metric mean, std, variance, consistency score |
| `visual_consistency_console.txt` | Copy of stdout (only written with `-o`) |

---

## Metrics

Each metric is computed on every consecutive frame pair (i, i+1).
Energy of Gradient and Spectral Centroid are single-frame metrics stored
alongside the pair they belong to (computed on frame i).

### A. Photometric Consistency

#### 1. Bhattacharyya Distance
**Range:** [0, ∞) — lower is better.

Measures the "distance" between the normalised 256-bin intensity histograms
of two consecutive frames.

```
B(H₁, H₂) = √(1 − Σ √(H₁(k) · H₂(k)))
```

| Value | Interpretation |
|-------|---------------|
| ≈ 0 | Identical intensity distributions — stable strobe |
| 0.1 – 0.3 | Mild illumination variation — acceptable |
| > 0.3 | Large photometric shift — strobe flicker or exposure jump |

**Watch for:** Sudden spikes (single misfired strobe pulse) or a slow drift
(gradual exposure creep or sediment plume obscuring the seafloor).

---

#### 2. Zero-Mean Normalised Cross-Correlation (ZNCC)
**Range:** [−1, 1] — higher is better.

Measures texture similarity *after* subtracting the mean intensity from each
frame. Unlike raw correlation, ZNCC is insensitive to global brightness
changes — a frame that is twice as bright but otherwise identical will give
ZNCC = 1.

```
ZNCC(A, B) = Σ[(Aᵢ − μ_A)(Bᵢ − μ_B)] / (N · σ_A · σ_B)
```

| Value | Interpretation |
|-------|---------------|
| > 0.8 | Highly consistent texture — good feature repeatability |
| 0.3 – 0.8 | Moderate variation — acceptable scene motion |
| < 0.3 | Low texture consistency — marine snow, blur, or large AUV motion |
| ≈ 0 | One or both frames are near-uniform (saturated or blank) |

**Watch for:** Sustained low values indicate the strobe is not illuminating
useful texture (silt cloud, over-exposure). Sudden drops mark individual bad
frames.

---

### B. Structural Consistency

#### 3. Structural Similarity Index (SSIM)
**Range:** [−1, 1] — higher is better; typical good frames are 0 – 1.

Perception-based comparison of luminance, contrast, and local structure.
Computed on float images normalised to [0, 1].

| Value | Interpretation |
|-------|---------------|
| > 0.7 | High structural similarity — stable scene |
| 0.3 – 0.7 | Moderate change — AUV motion or minor disturbance |
| < 0.3 | Large structural change — motion blur, marine snow, scene change |

**Watch for:** Drops in SSIM that do *not* coincide with drops in ZNCC
suggest the scene content is changing (AUV turning, seabed topology change)
rather than an illumination problem.

---

#### 4. Energy of Gradient (EoG)
**Range:** [0, ∞) — higher means sharper.

Measures image sharpness by summing squared Sobel gradient magnitudes,
normalised by pixel count so the value is resolution-independent.

```
EoG = Σ(Gₓ² + G_y²) / N_pixels
```

| Value | Interpretation |
|-------|---------------|
| High, stable | Sharp, well-illuminated seafloor texture — ideal |
| Sudden drop | Strobe illuminating a turbid silt cloud instead of the seafloor |
| Gradual decline | AUV ascending (scene becoming more uniform) or lens fouling |

**Watch for:** EoG drops that align with Bhattacharyya spikes are the
clearest signature of a strobe misfire followed by a blown-exposure frame.

---

### C. Frequency Domain

#### 5. Phase Correlation PSR
**Range:** [0, ∞) — higher means stronger alignment.

The peak magnitude of the normalised cross-power spectrum between two frames
(`cv2.phaseCorrelate`). Quantifies how well the two frames are geometrically
aligned in the frequency domain.

```
PSR = peak(|F(A) · conj(F(B))| / |F(A) · conj(F(B))|)
```

| Value | Interpretation |
|-------|---------------|
| High, stable | Consistent geometric overlap — good for feature tracking |
| Low | Visual decorrelation from backscatter, large inter-frame motion |
| Near zero | Frames are effectively uncorrelated — tracking likely to fail |

**Watch for:** Consistently low PSR even when ZNCC and SSIM are moderate
suggests high-frequency backscatter noise is dominating the spectrum.

---

#### 6. Spectral Centroid
**Range:** [0, ∞) px (radial frequency units) — higher means more detail.

The power-weighted mean radial distance from DC in the 2D FFT power spectrum.
Higher values indicate richer high-frequency content (fine seafloor texture);
lower values indicate a blurred or featureless image.

```
SC = Σ(r · P(r)) / Σ(P(r))   where r = √(u² + v²)
```

| Value | Interpretation |
|-------|---------------|
| Stable, relatively high | Consistent fine detail — good for ORB feature extraction |
| Gradual decrease | Scene becoming blurrier (altitude gain or turbidity increase) |
| Sudden drop | Single blurred frame (motion blur or out-of-focus strobe) |

**Watch for:** Spectral Centroid is the most sensitive indicator of subtle
focus drift. Correlate with EoG for a complete sharpness picture — EoG
responds to edge intensity, SC responds to high-frequency spatial content.

---

## Consistency Score

Each metric in `analysis_summary.json` includes a `consistency_score` in [0, 1]:

```
consistency_score = 1 / (1 + CV)   where CV = std / |mean|
```

A score of **1.0** means zero variance (perfectly consistent sequence).
Scores below **0.5** indicate the metric is highly variable relative to its mean
and the sequence warrants closer inspection.

---

## Strobe Stability Heatmap

`strobe_stability_heatmap.png` contains two panels:

**Top — Intensity Distribution over Time**
Each column is the normalised 256-bin histogram of one frame, rendered as a
`plasma` heatmap. Overlaid are the rolling P₂₅/P₅₀/P₇₅ lines:

- **Smooth, nearly horizontal bands** → photometrically stable sequence.
- **Sharp vertical deviations** → strobe jitter (individual misfires).
- **Slow vertical drift** → gradual exposure change or scene depth change.

**Bottom — Δ Histogram (Frame-to-Frame Rate of Change)**
Each column is `hist[i+1] − hist[i]` per intensity bin. The `RdBu_r`
diverging colormap is centred at zero:

- **Red** (positive) → that intensity bin gained pixel mass between frames.
- **Blue** (negative) → that intensity bin lost pixel mass between frames.
- **Thin, faint columns** → quiet transitions (slow AUV motion).
- **Broad red/blue bands** → sudden illumination shift (strobe misfire).

---

## Module layout

```
visual_consistency/
├── loader.py     Image loading — PNG (uint8) and TIF (Bayer BGGR16 → uint8 grayscale).
│                 iter_images()      single-frame generator
│                 iter_image_pairs() consecutive-pair generator (used by analysis)
├── metrics.py    Six pure metric functions (no I/O, no side effects).
├── analysis.py   run_analysis() — orchestrates the pair loop, returns a DataFrame.
└── reporting.py  plot_dashboard()         3×2 time-series dashboard
                  plot_strobe_heatmap()    intensity heatmap + delta analysis
                  compute_summary()        per-metric statistics dict
                  write_summary_json()     JSON output (delegates to trajectory_evals.io)
```
