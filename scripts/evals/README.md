# ORB-SLAM3 Evaluation Suite

Offline evaluation tools for AUV runs built on ORB-SLAM3.  Three independent
subsystems share this directory: trajectory evaluation, stereo calibration
diagnostics, and visual consistency analysis.

Activate the environment before running anything:

```bash
source ~/.pyenv/bin/activate
```

---

## CLI entry points

All commands below are installed in editable mode — changes to source files
take effect immediately with no reinstall.

### `traj-eval` — align and score a single run

```
traj-eval --platform-config <yaml> --test-frame <frame> --ref-frame <frame>
          [--dir <run_dir>] [--test <csv>] [--ref <csv|xml>]
          [--delta-t <s>] [--no-scale] [--alignment-on central|trajectory_start]
          [--segment-gap-seconds <s>] [--min-segment-samples <n>]
          [--output-dir <dir>] [--show]
```

Aligns an ORB-SLAM3 trajectory against a reference (CSV or Metashape XML) using
Umeyama similarity, then computes APE and RPE.

**Required flags:**
- `--platform-config` — static_tf platform YAML (same as the `--platform-config`
  passed to `orb_slam3_wrapper_main`). Used to look up the static transform
  between `--test-frame` and `--ref-frame`.
- `--test-frame` — TF frame the test trajectory is expressed in
  (e.g. `cam_aft`).
- `--ref-frame` — TF frame the reference trajectory is expressed in
  (e.g. `vehicle`).

**Auto-discovery via `--dir`:** if `--test`/`--ref` are not supplied, the script
looks for `<dir>/trajectory_frames.csv` and `<dir>/trajectory_nav.csv`.

**Outputs written to `--output-dir`:**

| File | Contents |
|------|----------|
| `trajectory_aligned.csv` | Per-pose aligned positions + APE/RPE columns |
| `trajectory_errors_console.txt` | Copy of stdout |
| `ape_errors.png` | APE translation time-series + components |
| `rpe_errors.png` | RPE translation time-series + components |
| `trajectory_2d_planes.png` | XY scatter coloured by APE |
| `drift_vs_distance.png` | APE vs cumulative distance |
| `scale_drift.png` | RPE-derived scale ratio over time |
| `ape_cdf.png` | APE cumulative distribution |

---

### `traj-inspect` — visualise one or more pose files

```
traj-inspect <pose_file.csv|.xml> [<pose_file2> ...] --labels "L1" "L2"
             [-o <output_dir>] [--show]
```

Generates motion distribution and trajectory visualizations without needing a
reference or alignment. Accepts any file format supported by `load_pose_file`.

**Outputs written to `-o`:**

| File | Contents |
|------|----------|
| `trajectory_2d.png` | Best-fit 2D projection (PCA) |
| `motion_distribution.png` | 6-panel distribution (translation + rotation) |
| `motion_timeseries.png` | Speed and angular-rate profiles |
| `rate_distribution.png` | 6-panel velocity/angular-rate distributions |

---

### `traj-compare` — cross-run statistics

```
traj-compare run1/trajectory_aligned.csv run2/trajectory_aligned.csv
             [--labels "Run A" "Run B"] [-o <output_dir>] [--show]
```

Accepts `trajectory_aligned.csv` files produced by `traj-eval`, or directories
that contain one. Produces side-by-side violin plots, CDF overlay, and a summary
statistics table.

**Outputs written to `-o`:**

| File | Contents |
|------|----------|
| `ape_comparison.png` | Violin/box comparison of APE translation |
| `ape_cdf.png` | CDF overlay for all runs |
| `rpe_comparison.png` | Violin/box comparison of RPE translation |
| `summary.csv` | Per-run statistics (RMSE, mean, median, max) |

---

### `build-inspector` — generate frame inspector JSON

```
build-inspector <frame_stats.csv> [--trajectory trajectory_aligned.csv]
                [-o <output.json>]
```

Converts an ORB-SLAM3 frame-stats CSV to a JSON file loadable by
`frame_inspector.html`. Open `frame_inspector.html` in a browser and use the
file picker to load the generated JSON.

When `--trajectory` is supplied, per-pose APE/RPE columns are merged in and the
3D Trajectory tab is enabled in the dashboard.

---

### `vis-check` — temporal visual consistency

```
vis-check <image_dir> [-o <output_dir>] [--ext tif|png] [--max-images N] [--show]
```

Photometric, structural, and frequency-domain analysis across consecutive
frames from a single camera. Pre-flight health check for VO runs.

See [`visual_consistency/README.md`](visual_consistency/README.md) for full
metric documentation.

**Outputs written to `-o`:**

| File | Contents |
|------|----------|
| `consistency_metrics.csv` | Per-pair values for all six metrics |
| `consistency_dashboard.png` | 3×2 time-series plot |
| `strobe_stability_heatmap.png` | Intensity distribution heatmap + delta |
| `analysis_summary.json` | Per-metric mean, std, variance, consistency score |
| `visual_consistency_console.txt` | Copy of stdout |

---

### `stereo-check` — stereo pair visual consistency

```
stereo-check <left_dir> <right_dir> [-o <output_dir>] [--ext tif|png]
             [--max-images N]
```

Same photometric and structural metrics applied across synchronised left/right
pairs. Diagnoses exposure mismatch, one-sided lens fouling, and sync gaps.

See [`visual_consistency/README.md`](visual_consistency/README.md) for full
metric documentation.

**Outputs written to `-o`:**

| File | Contents |
|------|----------|
| `stereo_consistency_metrics.csv` | Per-pair values for four pair-wise metrics |
| `stereo_consistency_dashboard.png` | 2×2 plot |
| `stereo_analysis_summary.json` | Per-metric statistics |
| `stereo_consistency_console.txt` | Copy of stdout |

---

## Scripts (not installed as CLI commands)

| Script | Purpose |
|--------|---------|
| `stereo_diagnose.py` | Diagnose stereo calibration from an image pair — epipolar error, reprojection RMS, pre-rect vs rectified comparison. Requires `--config`, `--left`, `--right`. |
| `stereo_debug.py` | Visual debug for stereo matching — epipolar guidelines overlay, optional disparity map, optional rigorous validation. |
| `cam_intrinsics.py` | Visualise distortion warp from a YAML as a grid deformation. No images required. |

---

## Packages

| Package | Purpose | README |
|---------|---------|--------|
| `trajectory_evals/` | Pose loading, Umeyama alignment, APE/RPE computation, plotting | [README](trajectory_evals/README.md) |
| `stereo/` | Stereo config loading, epipolar geometry, feature matching, rectification, validation | [README](stereo/README.md) |
| `visual_consistency/` | Image loading, photometric/structural/frequency-domain metrics, dashboard reporting | [README](visual_consistency/README.md) |

### Shared utilities

- `plot_style.py` — call `plot_style.apply_paper_style()` at the top of any
  script that generates figures (seaborn-paper rcParams, 300 DPI save).
- `frame_inspector.html` — standalone browser dashboard; load a JSON produced
  by `build-inspector` via the file picker.

---

## Tests

```bash
# All working tests
python -m pytest tests/test_trajectory_alignment.py tests/test_build_inspector.py \
    tests/test_stereo_geometry.py tests/test_stereo_matching.py -q

# Single test file
python -m pytest tests/test_trajectory_alignment.py -q
```

Always specify test files explicitly — listing `tests/` as a directory triggers
a ROS pytest plugin conflict. `tests/test_stereo_config.py` is currently broken.
