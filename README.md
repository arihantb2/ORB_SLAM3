# ORB-SLAM3 (Fork)

Heavily modified fork of [UZ-SLAMLab/ORB\_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), packaged as a CMake library for ROS 2. Designed for AUV stereo/monocular visual odometry via LCM log replay.

**Camera models:** `PinHole`, `Rectified`, `Metashape`  
**Sensor modes:** `MONOCULAR`, `STEREO`

---

## Divergence from Upstream

### Changes and additions

**GTSAM replaces g2o as the backend solver.**
All bundle adjustment and pose graph optimization uses GTSAM factor graphs. g2o has been removed entirely.

**fbow replaces DBoW2 as the default BoW backend.**
fbow is faster to query and supports float descriptors (required for SIFT vocabularies). DBoW2 is retained as a fallback. Configure via `Vocabulary.type` in the algorithm YAML — see [Config Files](#config-files).

**Navigation pose priors in monocular initialization and local bundle adjustment.**
External navigation poses (e.g. from DVL/INS) can be injected as absolute pose constraints or odometric between-factors into the local BA factor graph. Configured per-run via `--nav-csv` and per-config via `Optimizer.LocalBundleAdjustment.*` in the algorithm YAML. This substantially improves monocular scale observability in low-texture underwater scenes.

**Pluggable feature extractor API supporting OpenCV ORB and BRISK.**
The extractor is selected at runtime via `FeatureExtractor.type`. Three options are available: `GridORB` (custom FAST dual-threshold + quadtree distribution, default), `ORB` (cv::ORB per pyramid level), and `BRISK` (BRISK with native scale space). All share a common descriptor interface; the rest of the pipeline is extractor-agnostic.

**Metashape pinhole camera model.**
Supports calibrations exported directly from Agisoft Metashape (`Camera.type: "Metashape"` in the camera calibration YAML), in addition to the standard `PinHole` and `Rectified` models.

**Map memory management overhaul and upstream leak fixes.**
Map points and keyframes use explicit ownership semantics with deferred deletion. Several upstream memory leaks — including dangling map point references and unreleased keyframe observations — have been resolved.

### Removed

| Feature | Notes |
|---------|-------|
| Fisheye / KannalaBrandt8 lens model | Not required for AUV cameras |
| IMU / inertial tracking | Replaced by external navigation pose priors |
| Loop closing and loop closure thread | Not needed for visual odometry use case |
| Map merging | Removed along with multi-map support |
| Multi-map Atlas | System maintains exactly one map at all times |
| `RECENTLY_LOST` relocalization | Tracking loss triggers an immediate map reset |
| Dataset example executables (`Examples/`) | Superseded by the ROS 2 wrapper |
| Python bindings | Removed |

---

## Dependencies

- C++17, CMake ≥ 3.10
- OpenCV ≥ 4.2, Eigen3 ≥ 3.3.7, Sophus, DBoW2, fbow, GTSAM
- ROS 2 (rclcpp, tf2, nav_msgs, sensor_msgs, visualization_msgs)
- `lcm_log_player`, `acfr_lcm_types`, `static_tf` (internal packages)

---

## Building

```bash
wsman build -p orbslam3 --deps
```

The build extracts `Vocabulary/ORBvoc.txt` from its `.tar.gz` automatically.

---

## Repository Layout

```
config/
  camera/          # Per-platform camera calibration YAMLs
  platform/        # Per-platform sensor frame YAMLs
  stereo_orb.yaml  # Algorithm configs (stereo_{orb,gridorb,brisk}.yaml, mono_*)
orb_slam3_wrapper/ # ROS 2 node that drives the library (see below)
scripts/runners/   # Python launch helper
src/ include/      # ORB-SLAM3 library
Vocabulary/        # ORBvoc.txt / fbow vocabulary files
```

---

## The Wrapper (`orb_slam3_wrapper`)

`orb_slam3_wrapper_main` is the ROS 2 node that wires ORB-SLAM3 to LCM log data. It:

1. Reads a camera calibration YAML (`config/camera/`) and injects it into `ORB_SLAM3::System` — the library itself has no file-based camera loading.
2. Replays a `.lcm` log file, feeding stereo or monocular frames to the tracker.
3. Publishes pose, map points, and TF over ROS 2 topics.
4. Writes trajectory CSVs to the output directory (see **Output Files** below).

**ROS topics published:**

| Topic | Type | Description |
|-------|------|-------------|
| `/orb_slam3/tracking/pose` | `geometry_msgs/PoseStamped` | Per-frame camera pose |
| `/orb_slam3/tracking/odometry` | `nav_msgs/Odometry` | Per-frame odometry |
| `/orb_slam3/tracking/keyframe_path` | `nav_msgs/Path` | All keyframe poses |
| `/orb_slam3/tracking/map_points/inlier_all` | `sensor_msgs/PointCloud2` | All inlier map points |
| `/orb_slam3/tracking/map_points/local_inliers` | `sensor_msgs/PointCloud2` | Local map inliers |
| `/orb_slam3/tracking/map_points/local_outliers` | `sensor_msgs/PointCloud2` | Local map outliers |
| `/orb_slam3/tracking/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Tracking health |
| `/orb_slam3/tracking/debug/left` | `sensor_msgs/Image` + `CameraInfo` | Debug images (if enabled) |

**Key flags:**

| Flag | Description |
|------|-------------|
| `--vocab-file` | Path to vocabulary (`.fbow` or `ORBvoc.txt`) |
| `--camera-calib-file` | Camera calibration YAML |
| `--config-file` | Algorithm config YAML (tracking / feature params) |
| `--lcm-file` | Path to `.lcm` log |
| `--mono` | Monocular mode (default: stereo) |
| `--use-priors` | Navigation priors for monocular tracking |
| `--synchronous-local-mapping` | Single-threaded tracking+mapping (debug) |
| `--verbose` | ORB-SLAM3 console output |

---

## Running via the Python Launcher

The launcher at `scripts/runners/run_orbslam.py` resolves paths, picks the right config, and calls `orb_slam3_wrapper_main`. Activate the Python env first:

```bash
source ~/.pyenv/bin/activate
```

**Minimal stereo run:**

```bash
python scripts/runners/run_orbslam.py \
  --dataset /path/to/dataset \
  --vocab-path /path/to/orb_voc.fbow \
  --platform-config seeker_cheryl_samoa.yaml \
  --camera-calib-file cheryl_solomon2025/stereo.yaml
```

**Monocular run:**

```bash
python scripts/runners/run_orbslam.py \
  --dataset /path/to/dataset \
  --vocab-path /path/to/orb_voc.fbow \
  --platform-config seeker_cheryl_samoa.yaml \
  --camera-calib-file cheryl_solomon2025/mono.yaml \
  --mono --image-filter left
```

**Feature extractor** (`--orbslam3-extractor-type`): `gridorb` (default), `orb`, `brisk`.

Use `--dry-run` to print the full command without executing. Use `--gdb` to attach a debugger.

---

## Output Files

When `--output-dir` is set (or auto-derived by the launcher), the wrapper writes:

| File | Contents |
|------|----------|
| `trajectory_frames.csv` | Per-frame pose: `timestamp, tx, ty, tz, qw, qx, qy, qz` in `vo_map → vo_camera` |
| `trajectory_keyframes.csv` | Keyframe-only pose log |
| `trajectory_nav.csv` | Navigation prior poses (if `--nav-csv` supplied) |
| `trajectory_frame_stats.csv` | Per-frame tracking stats (inlier counts, tracking state, etc.) |
| `orbslam3.log` | Full ORB-SLAM3 console log |
| `command.txt` | The exact command used to produce this run |

The launcher auto-derives the output path as `<dataset>/../PROCESSED_DATA/<dataset_name>/orb_slam3/stereo` (or `mono`). Override with `--output-dir` or suppress with `--no-output`.

---

## Config Files

Algorithm YAMLs (`config/stereo_gridorb.yaml`, etc.) control tracking and feature extraction only — no camera parameters. Key sections:

- `FeatureExtractor.*` — extractor type and parameters
- `Tracking.*` — keyframe insertion, motion model, local map thresholds
- `LocalMapping.*` — BA frequency, map point culling, triangulation
- `Vocabulary.*` — BoW backend (`dbow2` or `fbow`) and path override

Camera calibration lives separately in `config/camera/`.

---

## Image Format Note

AUV images are Bayer BGGR16 (`uint16`). The wrapper demosaics via `COLOR_BayerBG2BGR` before passing frames to the tracker — do not pass raw Bayer frames directly.

---

## Using the Library Directly

```cpp
#include <CameraModels/CameraCalibrationInput.h>
#include <ORB_SLAM3/System.h>

ORB_SLAM3::CameraCalibrationInput calib = /* built via CreatePinholeCamera / CreateMetashapeCamera */;
ORB_SLAM3::System slam("Vocabulary/ORBvoc.txt", "config/stereo_gridorb.yaml",
                       ORB_SLAM3::System::STEREO, calib);

Sophus::SE3f pose = slam.TrackStereo(imgLeft, imgRight, timestamp);
slam.Shutdown();
```

```cmake
find_package(ORB_SLAM3 REQUIRED)
target_link_libraries(my_target ORB_SLAM3::ORB_SLAM3)
```

---

## Evaluation Tools

`scripts/evals/` contains offline trajectory evaluation and diagnostics tools. Activate the env first (`source ~/.pyenv/bin/activate`), then see [`scripts/evals/README.md`](scripts/evals/README.md) for full usage. Key entry points:

- `traj-eval` — align and score a trajectory against a reference (CSV or Metashape XML)
- `traj-compare` — compare multiple runs side-by-side
- `stereo-diagnose` — stereo calibration and rectification diagnostics

---

## Further Reading

- [`docs/TrackingResultRichData.md`](docs/TrackingResultRichData.md) — all fields in `TrackingResult` (per-frame output of `Track()`)
- [`docs/LocalMappingRichData.md`](docs/LocalMappingRichData.md) — `LocalMappingResult` callback schema for monitoring the mapping thread

---

## Credits

This repo is a heavily modified fork of [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). Original work by Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel, and Juan D. Tardós at the University of Zaragoza.

If you use this in academic work, cite the original paper:
> Campos et al., "ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM", IEEE Transactions on Robotics, 2021. [[arXiv]](https://arxiv.org/abs/2007.11898)

## License

[GPLv3](LICENSE).
