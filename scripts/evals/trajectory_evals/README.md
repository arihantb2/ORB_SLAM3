# trajectory_evals

Library for loading, aligning, and computing errors on 6-DOF trajectories.
Consumed by the installed CLI entry points (`traj-eval`, `traj-inspect`,
`traj-compare`, `build-inspector`).

---

## Module layout

```
trajectory_evals/
├── io.py          All pose file loaders and writers.
│                  load_pose_file()        dispatcher — sniffs extension + columns
│                  load_csv()              standard trajectory CSV
│                  load_xml()              Metashape-style camera XML
│                  load_online_nav_csv()   vehicle_online_nav.csv format
│                  write_trajectory_csv()  write aligned trajectory CSV
│                  write_json()            write minified JSON (used by reporting)
├── alignment.py   Umeyama alignment and APE/RPE computation.
│                  align_umeyama()         scale+rotation+translation alignment
│                  compute_errors()        per-pose APE and RPE over segments
│                  split_segments_by_gap() split timestamp array on time gaps
└── plotting.py    All trajectory error figures.
                   plot_errors()           consumed only by traj-eval
```

---

## Pose format

All loaders return a `DataFrame` with these columns:

| Column | Type | Description |
|--------|------|-------------|
| `timestamp` | `float` | Unix epoch seconds |
| `tx`, `ty`, `tz` | `float` | Translation in metres |
| `qx`, `qy`, `qz`, `qw` | `float` | Quaternion in **scipy xyzw order** |

---

## `io.py` — loaders

### `load_pose_file(path, label, group_id=0)`

Dispatcher. Chooses loader based on extension and column sniff:

| File | Detected by | Loader |
|------|-------------|--------|
| `.xml` | extension | `load_xml()` |
| `.csv` with `{time, heading, depth, filename}` | column sniff | `load_online_nav_csv()` |
| `.csv` (all others) | extension | `load_csv()` |

### `load_csv(path, label)`

Requires columns `timestamp, tx, ty, tz, qx, qy, qz, qw`.

### `load_xml(path, label, group_id=0)`

Metashape-style camera XML. Supports grouped (`group[@id=N]`) and flat camera
layouts. Timestamps parsed from camera label format `PR_YYYYMMDD_HHMMSS_mmm_*`
as local epoch seconds. Cameras without a `<transform>` are skipped.

### `load_online_nav_csv(path, label)`

vehicle_online_nav.csv format:

| Source column | Mapping |
|---------------|---------|
| `time` (µs int) | → `timestamp` (Unix seconds float) |
| `x`, `y` | → `tx`, `ty` |
| `depth` (NED z-down, positive) | → `tz` |
| `roll`, `pitch`, `heading` (degrees) | → quaternion via Euler ZYX intrinsic |

Only rows where `filename` contains `"AC"` are kept (one camera per timestamp).

Heading is a signed compass bearing: 0 = North, positive = clockwise (East),
range [−180, 180]. No ENU/NED frame flip is applied here — use `--ref-frame`
and `--test-frame` in `traj-eval` to reconcile frames.

### `write_trajectory_csv(path, timestamps, positions, quaternions, extra_columns=None)`

Writes a trajectory in the standard CSV format. `positions` is `(N, 3)`,
`quaternions` is `(N, 4)` xyzw. `extra_columns` is an optional
`{name: array}` dict appended after the pose columns.

---

## `alignment.py` — Umeyama + error computation

### `align_umeyama(model, data, with_scale=True)`

Closed-form similarity transform that maps `data` onto `model`.

```
s, R, t = align_umeyama(p_ref, p_est, with_scale=True)
p_aligned = s * (p_est @ R.T) + t
```

Returns `(s, R, t)` where `R` is a `(3, 3)` rotation matrix, `t` is `(3,)`.
Handles the reflection degenerate case. Falls back to `s=1.0` when the data
segment has zero variance (single-sample segments).

### `compute_errors(df_est, df_ref, delta_t, tolerance, with_scale, segment_gap_seconds, min_segment_samples=1)`

Main alignment and error pipeline:

1. Clips estimated trajectory to the temporal overlap with the reference.
2. Linearly interpolates reference positions and slerps reference rotations to
   estimated timestamps.
3. Splits the trajectory on gaps > `segment_gap_seconds` (default 0.6 s);
   drops segments shorter than `min_segment_samples`.
4. Runs Umeyama per segment, applies it to positions and rotations.
5. Computes APE (per-pose position and rotation error against interpolated ref).
6. Computes RPE at time horizon `delta_t` with `tolerance` matching window.

Returns a dict. Key fields:

| Key | Shape | Description |
|-----|-------|-------------|
| `t_est_valid` | `(N,)` | Timestamps of kept poses |
| `p_est_aligned` | `(N, 3)` | Aligned estimated positions |
| `q_est_aligned` | `(N, 4)` | Aligned estimated quaternions (xyzw) |
| `p_ref_interp` | `(N, 3)` | Reference positions interpolated to `t_est_valid` |
| `ape_trans` | `(N,)` | Per-pose translation APE (metres) |
| `ape_rot_deg` | `(N,)` | Per-pose rotation APE (degrees) |
| `ape_vec` | `(N, 3)` | Signed APE vector (ref − est) |
| `rpe_trans` | `(M,)` | RPE translation errors (metres) |
| `rpe_rot_deg` | `(M,)` | RPE rotation errors (degrees) |
| `rpe_times` | `(M,)` | Timestamps of RPE anchor poses |
| `rpe_scale` | `(M,)` | Per-pair scale ratio (gt / est) |
| `segments` | list of `(start, end)` | Segment index ranges into compacted arrays |
| `scale` | `float` | Mean Umeyama scale across segments |
| `ref_dist` | `(N,)` | Cumulative reference path distance (metres) |

---

## `plotting.py` — figures

`plot_errors(results, alignment_on, output_dir=None, show=True)` — consumed
only by `traj-eval`. Saves six PNG files:

| File | Contents |
|------|----------|
| `ape_errors.png` | APE translation time-series + component breakdown |
| `rpe_errors.png` | RPE translation time-series + component breakdown |
| `trajectory_2d_planes.png` | XY scatter coloured by APE |
| `drift_vs_distance.png` | APE vs cumulative reference distance with % annotation |
| `scale_drift.png` | RPE-derived scale ratio over time |
| `ape_cdf.png` | APE cumulative distribution with P50/P75/P90/P95 markers |

Call `plot_style.apply_paper_style()` before any figure that needs consistent
seaborn-paper rcParams (handled inside `plot_errors`).
