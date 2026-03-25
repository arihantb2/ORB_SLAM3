# ORB-SLAM3 (Fork)

A stripped-down fork of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3/) packaged as a CMake library for use in a ROS 2 workspace.

**Removed from upstream:**

- RGB-D sensor support
- Fisheye / KannalaBrandt8 lens model
- `RECENTLY_LOST` relocalization — tracking loss resets the map immediately
- Loop closing / loop closure
- Map merging
- Multi-map Atlas (system always maintains exactly one map)
- Dataset example executables (`Examples/`)
- Python bindings

**Camera models supported:** `PinHole`, `Rectified`, `Metashape`  
**Sensor modes supported:** `MONOCULAR`, `STEREO`

---

## Prerequisites

- C++17, CMake ≥ 3.10
- OpenCV ≥ 4.2, Eigen3 ≥ 3.3.7, DBoW2, fbow, GTSAM
- `Vocabulary/ORBvoc.txt` (or its `.tar.gz` — CMake extracts it automatically)

---

## Building

**Standalone:**

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

**ROS 2 / colcon:**

```bash
colcon build --packages-select orbslam3
```

Downstream packages:

```cmake
find_package(ORB_SLAM3 REQUIRED)
target_link_libraries(my_target ORB_SLAM3::ORB_SLAM3)
```

---

## Usage

ORB-SLAM3 does not load camera parameters from file. Calibration is **injected** at construction: the caller (e.g. the `orb_slam3_wrapper`) loads a camera calibration file, builds one or two `GeometricCamera` objects via `CreatePinholeCamera` / `CreateMetashapeCamera`, fills a `CameraCalibrationInput`, and passes it into `System`.

```cpp
#include <CameraModels/CameraCalibrationInput.h>
#include <ORB_SLAM3/System.h>

// Caller loads camera calib (e.g. from YAML), builds cameras with
// CreateMetashapeCamera / CreatePinholeCamera, fills calib.
ORB_SLAM3::CameraCalibrationInput calib = /* ... */;

ORB_SLAM3::System slam("Vocabulary/ORBvoc.txt", "config/stereo.yaml", ORB_SLAM3::System::STEREO, calib);

Sophus::SE3f pose = slam.TrackStereo(imgLeft, imgRight, timestamp);
// or: slam.TrackMonocular(img, timestamp);

slam.Shutdown();
```

**Config files:** Algorithm YAML files are **algorithm-only** (tracking, features, local mapping). Camera calibration is injected via `CameraCalibrationInput`. Tracking parameters and log messages are documented in [`TrackingLogs.md`](TrackingLogs.md).

---

## Vocabulary backend (DBoW2 vs fbow)

ORB-SLAM3 can switch the Bag-of-Words (BoW) backend via the algorithm config YAML passed to `System`.

Example snippet for using `fbow`:

```yaml
Vocabulary.type: "fbow"
Vocabulary.path: "src/ORB_SLAM3/Vocabulary/orb_mur.fbow"
```

Notes:

- If `Vocabulary.type` / `Vocabulary.path` are omitted, the system defaults to `dbow2` and uses the `--vocab-file` argument provided by the wrapper.
- Descriptor type must match the vocabulary:
  - `ORB` / `GridORB` (binary descriptors, `CV_8UC1`) expects an `fbow` vocabulary trained on binary descriptors.
  - `SIFT` (float descriptors, `CV_32FC1`) requires an `fbow` vocabulary trained on float descriptors. If it doesn’t match, BoW-based reference-keyframe tracking falls back to non-BoW matching for runtime stability.

---

## License

[GPLv3](LICENSE). See [Dependencies.md](Dependencies.md) for third-party licenses.

Original work by Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel, Juan D. Tardós et al. — cite the [ORB-SLAM3 paper](https://arxiv.org/abs/2007.11898) if you use this in academic work.
