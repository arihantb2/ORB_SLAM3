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
**Sensor modes supported:** `MONOCULAR`, `STEREO`, `IMU_MONOCULAR`, `IMU_STEREO`

---

## Prerequisites

- C++17, CMake ≥ 3.10
- OpenCV ≥ 4.2, Eigen3 ≥ 3.3.7, Pangolin, DBoW2, g2o
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

ORB_SLAM3::System slam("Vocabulary/ORBvoc.txt", "config/stereo.yaml",
                       ORB_SLAM3::System::STEREO, calib, /*bUseViewer=*/true);

Sophus::SE3f pose = slam.TrackStereo(imgLeft, imgRight, timestamp);
// or: slam.TrackMonocular(img, timestamp);

slam.Shutdown();
```

**Config files:** `config/mono.yaml` and `config/stereo.yaml` are **algorithm-only** (tracking, ORB, local mapping, viewer). Camera calibration lives in `config/camera_calib_mono.yaml` and `config/camera_calib_stereo.yaml` and is loaded by the wrapper. Tracking parameters and log messages are documented in [`TrackingLogs.md`](TrackingLogs.md).

---

## License

[GPLv3](LICENSE). See [Dependencies.md](Dependencies.md) for third-party licenses.

Original work by Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel, Juan D. Tardós et al. — cite the [ORB-SLAM3 paper](https://arxiv.org/abs/2007.11898) if you use this in academic work.
