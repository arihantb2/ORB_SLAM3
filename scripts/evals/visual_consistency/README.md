# visual_consistency

Pre-flight health check for AUV image sequences captured under strobe illumination.
Provides two analysis modes:

- **Temporal** (`vis-check`) — photometric, structural, and frequency-domain stability
  across consecutive frames from a single camera. Diagnose why a VO run fails before
  committing to the full pipeline.
- **Stereo** (`stereo-check`) — same photometric and structural metrics applied across
  synchronised left/right camera pairs. Diagnose rig issues (exposure mismatch,
  one-sided lens fouling, sync gaps) without requiring extrinsic calibration.

---

## CLI quick start

### Temporal (single camera)

```bash
# All images in a directory, save outputs
vis-check images-AC/ -o results/

# TIF raw files only, limit to first 200 frames
vis-check i20251003_212212/ --ext tif --max-images 200 -o results/

# Preview first 50 frames interactively
vis-check images-AC/ --max-images 50 --show
```

### Stereo (left + right cameras)

```bash
# Compare left and right image directories
stereo-check images-left/ images-right/ -o results/

# TIF raw files, first 200 pairs
stereo-check images-left/ images-right/ --ext tif --max-images 200 -o results/
```

---

## Outputs

### Temporal (`vis-check`)

| File | Contents |
|------|----------|
| `consistency_metrics.csv` | Per-pair values for all six metrics |
| `consistency_dashboard.png` | 3×2 time-series plot, one panel per metric |
| `strobe_stability_heatmap.png` | Intensity distribution heatmap + delta analysis |
| `analysis_summary.json` | Per-metric mean, std, variance, consistency score |
| `visual_consistency_console.txt` | Copy of stdout (only written with `-o`) |

### Stereo (`stereo-check`)

| File | Contents |
|------|----------|
| `stereo_consistency_metrics.csv` | Per-pair values for the four pair-wise metrics |
| `stereo_consistency_dashboard.png` | 2×2 plot, one panel per metric |
| `stereo_analysis_summary.json` | Per-metric mean, std, variance, consistency score |
| `stereo_consistency_console.txt` | Copy of stdout (only written with `-o`) |

---

## Metrics

### Temporal vs stereo coverage

| Metric | Temporal | Stereo | Notes |
|--------|----------|--------|-------|
| Bhattacharyya Distance | ✓ | ✓ | pair-wise |
| ZNCC | ✓ | ✓ | pair-wise |
| SSIM | ✓ | ✓ | pair-wise |
| Phase Correlation PSR | ✓ | ✓ | pair-wise |
| Energy of Gradient | ✓ | — | single-frame; tracks sharpness over time |
| Spectral Centroid | ✓ | — | single-frame; tracks focus drift over time |

EoG and Spectral Centroid are omitted from stereo analysis because they describe
a single frame's sharpness, not the relationship between left and right.

---

### A. Photometric Consistency

#### 1. Bhattacharyya Distance
**Range:** [0, ∞) — lower is better.

Measures the "distance" between the normalised 256-bin intensity histograms
of two frames.

```
B(H₁, H₂) = √(1 − Σ √(H₁(k) · H₂(k)))
```

| Value | Temporal interpretation | Stereo interpretation |
|-------|------------------------|-----------------------|
| ≈ 0 | Stable strobe — identical distributions | Left/right well-matched exposures |
| 0.1 – 0.3 | Mild illumination variation | Minor camera-to-camera gain difference |
| > 0.3 | Strobe flicker or exposure jump | Significant exposure mismatch between cameras |

**Watch for (temporal):** Sudden spikes (misfired strobe pulse) or slow drift
(exposure creep or sediment plume obscuring the seafloor).

**Watch for (stereo):** Sustained high values indicate a persistent exposure
or gain imbalance between the two cameras that will affect stereo matching quality.

---

#### 2. Zero-Mean Normalised Cross-Correlation (ZNCC)
**Range:** [−1, 1] — higher is better.

Measures texture similarity after subtracting the mean intensity from each
frame. Insensitive to global brightness differences — a frame that is twice
as bright but otherwise identical gives ZNCC = 1.

```
ZNCC(A, B) = Σ[(Aᵢ − μ_A)(Bᵢ − μ_B)] / (N · σ_A · σ_B)
```

| Value | Interpretation |
|-------|---------------|
| > 0.8 | Highly consistent texture — good feature repeatability |
| 0.3 – 0.8 | Moderate variation — acceptable scene motion or viewpoint change |
| < 0.3 | Low texture consistency — marine snow, blur, or large baseline |
| ≈ 0 | One or both frames are near-uniform (saturated or blank) |

**Watch for (temporal):** Sustained low values indicate the strobe is not
illuminating useful texture. Sudden drops mark individual bad frames.

**Watch for (stereo):** Low ZNCC alongside low SSIM suggests the stereo pair
is not observing the same scene content — possible sync issue or field-of-view
mismatch.

---

### B. Structural Consistency

#### 3. Structural Similarity Index (SSIM)
**Range:** [−1, 1] — higher is better; typical good frames are 0 – 1.

Perception-based comparison of luminance, contrast, and local structure.
Computed on float images normalised to [0, 1].

| Value | Interpretation |
|-------|---------------|
| > 0.7 | High structural similarity — stable scene |
| 0.3 – 0.7 | Moderate change — AUV motion, viewpoint shift, or minor disturbance |
| < 0.3 | Large structural change — motion blur, marine snow, or scene change |

**Watch for (temporal):** Drops in SSIM that do *not* coincide with drops in
ZNCC suggest scene content is changing (AUV turning, seabed topology) rather
than an illumination problem.

**Watch for (stereo):** Low SSIM with moderate ZNCC indicates structural
differences not explained by brightness — could be rectification error or
one camera being out of focus.

---

#### 4. Energy of Gradient (EoG)
**Range:** [0, ∞) — higher means sharper.  **Temporal only.**

Measures image sharpness by summing squared Sobel gradient magnitudes,
normalised by pixel count so the value is resolution-independent.
Computed on the earlier frame of each consecutive pair.

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

| Value | Temporal interpretation | Stereo interpretation |
|-------|------------------------|-----------------------|
| High, stable | Consistent geometric overlap — good for feature tracking | Strong spatial correlation between views |
| Low | Backscatter noise or large inter-frame motion | Rectification error or large stereo baseline at close range |
| Near zero | Frames effectively uncorrelated — tracking likely to fail | Views are decorrelated — stereo matching will fail |

---

#### 6. Spectral Centroid
**Range:** [0, ∞) px (radial frequency units) — higher means more detail.  **Temporal only.**

The power-weighted mean radial distance from DC in the 2D FFT power spectrum.
Higher values indicate richer high-frequency content (fine seafloor texture);
lower values indicate a blurred or featureless image.
Computed on the earlier frame of each consecutive pair.

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

Each metric in the JSON summary includes a `consistency_score` in [0, 1]:

```
consistency_score = 1 / (1 + CV)   where CV = std / |mean|
```

A score of **1.0** means zero variance (perfectly consistent sequence).
Scores below **0.5** indicate the metric is highly variable relative to its mean
and the sequence warrants closer inspection.

---

## Strobe Stability Heatmap

`strobe_stability_heatmap.png` is produced by `vis-check` only (temporal mode).
It contains two panels:

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
│                 iter_images()         single-frame generator
│                 iter_image_pairs()    consecutive-pair generator (temporal analysis)
│                 load_image_grayscale_u8()  single image loader (stereo analysis)
├── metrics.py    Six pure metric functions (no I/O, no side effects).
├── analysis.py   run_analysis()        temporal pair loop → DataFrame (6 metrics)
│                 run_stereo_analysis() stereo pair loop  → DataFrame (4 metrics)
└── reporting.py  plot_dashboard()         time-series dashboard (auto-sized grid)
                  plot_strobe_heatmap()    intensity heatmap + delta (temporal only)
                  compute_summary()        per-metric statistics dict
                  write_summary_json()     JSON output (delegates to trajectory_evals.io)
                  STEREO_METRIC_DEFS       metric list for stereo callers
```
