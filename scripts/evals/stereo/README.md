# stereo

Reusable library for stereo calibration diagnostics: loading ORB-SLAM3 YAML
configs, epipolar geometry, feature matching, rectification, and geometric
validation.

Not a CLI — consumed by the entry-point scripts in the parent directory
(`stereo_diagnose.py`, `stereo_debug.py`, `cam_intrinsics.py`).

---

## Module layout

```
stereo/
├── config.py       Load ORB-SLAM3 YAML calibration files.
│                   load_stereo_params()      → (image_size, K1, D1, K2, D2, R, t, camera_type)
│                   load_camera_intrinsics()  → (width, height, cameras[])
│                   extrinsic_T_c1_c2_from_static_tf_file()
├── geometry.py     Epipolar math (pure, no I/O).
│                   fundamental_matrix_from_rtk()   → F (3×3)
│                   sampson_distance()               → distances (N,)
│                   estimate_fundamental_inliers()   → (F or None, inlier_mask)
├── matching.py     Feature detection and descriptor matching.
│                   detect_and_match()       → (kp1, kp2, matches)
│                   matches_to_points()      → (pts1, pts2) shape (N, 2)
│                   epipolar_stats()         → {count, mean_abs_dy, median_abs_dy, max_abs_dy}
├── rectify.py      Stereo rectification for pinhole models.
│                   rectify_pair()           → (rect_left, rect_right[, transforms])
├── validation.py   Quantitative pass/fail epipolar validation.
│                   rigorous_epipolar_validation()  → (metrics dict, inlier_matches)
│                   RigorousValidationArgs          dataclass for thresholds
├── visualization.py  OpenCV-based debug visualizations.
│                   draw_matches_overlay()   → canvas (BGR)
│                   window_search_debug()    → (disp_vis_bgr, valid_ratio, low_texture_ratio)
│                   resize_to_width()
│                   wait_for_windows_or_key()
└── io.py           Image loading for Bayer BGGR raw inputs.
                    load_bayer_bggr_pair_bgr_u8()  → (left_bgr, right_bgr) uint8
                    demosaic_bggr_to_bgr_u8()
                    to_u8()
```

---

## Key types and contracts

### `load_stereo_params(config_path, *, platform_config_path=None, ...)`

Parses an ORB-SLAM3 stereo YAML. Returns:

```
(image_size, K1, D1, K2, D2, R, t, camera_type)
```

| Field | Type | Description |
|-------|------|-------------|
| `image_size` | `(width, height)` | From `Camera.width` / `Camera.height` |
| `K1`, `K2` | `np.ndarray (3×3)` | Camera matrices, left and right |
| `D1`, `D2` | `np.ndarray (5 or 8,)` | Distortion coefficients (OpenCV order) |
| `R` | `np.ndarray (3×3)` | Rotation from right to left camera |
| `t` | `np.ndarray (3,)` | Translation from right to left camera (metres) |
| `camera_type` | `str` | Raw value of `Camera.type` (e.g. `"pinhole"`) |

If `Stereo.T_c1_c2` is absent from the YAML, pass `platform_config_path` to
derive it from a `static_tf` platform YAML (`static_tf` must be installed).

Supported `Camera.type` values: `pinhole`, `Metashape` (treated identically).

### `load_camera_intrinsics(config_path)`

Loads per-camera intrinsics without requiring extrinsics. Returns:

```
(width, height, cameras)
```

Each entry of `cameras` is:
```python
{"name": str, "K": np.ndarray (3×3), "D": np.ndarray}
```

Works for both mono (`Camera.*`) and stereo (`Camera1.*` / `Camera2.*`) YAMLs.

### `rectify_pair(left_bgr, right_bgr, params, *, return_transforms=False)`

`params` is the tuple from `load_stereo_params`. Returns `(rect_left, rect_right)`.
With `return_transforms=True`, also returns a dict with `r1`, `r2`, `P1`, `P2`,
`image_size` — needed by `rigorous_epipolar_validation`.

### `rigorous_epipolar_validation(kp1, kp2, matches, rect_transforms, args)`

Full geometric validation after rectification. Returns `(metrics, inlier_matches)`.

Metrics dict keys:

| Key | Description |
|-----|-------------|
| `success` | `True` if all thresholds pass |
| `reason` | `"PASS"` or semicolon-joined failure reasons |
| `candidate_count` | Matches before RANSAC |
| `inlier_count` | RANSAC inliers |
| `inlier_ratio` | Inliers / candidates |
| `mean/median/p95/max_abs_dy` | Vertical pixel disparity in rectified space |
| `mean/median/p95/max_abs_dy_norm` | Same, normalised by image height |
| `mean/median/p95_sampson` | Sampson distance under the estimated F |

Thresholds are set via `RigorousValidationArgs` (frozen dataclass).

### `detect_and_match(left_gray, right_gray, detector_name, ratio_test)`

`detector_name` is `"sift"` or `"orb"`. Lowe ratio test applied.
Returns `(kp1, kp2, matches)` sorted by descriptor distance.

### `load_bayer_bggr_pair_bgr_u8(left_path, right_path)`

Reads a left/right pair from disk. Handles 8-bit and 16-bit TIF/PNG.
Bayer BGGR demosaiced to BGR uint8 via `COLOR_BayerBG2BGR`.
Raises `ValueError` if dimensions differ.

---

## Typical usage pattern

```python
from stereo.config import load_stereo_params
from stereo.io import load_bayer_bggr_pair_bgr_u8
from stereo.matching import detect_and_match
from stereo.rectify import rectify_pair
from stereo.validation import RigorousValidationArgs, rigorous_epipolar_validation

params = load_stereo_params("stereo.yaml")
left_bgr, right_bgr = load_bayer_bggr_pair_bgr_u8("left.tif", "right.tif")
rect_left, rect_right, transforms = rectify_pair(left_bgr, right_bgr, params, return_transforms=True)

import cv2
left_gray  = cv2.cvtColor(rect_left,  cv2.COLOR_BGR2GRAY)
right_gray = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)

from stereo.matching import detect_and_match
kp1, kp2, matches = detect_and_match(left_gray, right_gray, "sift", ratio_test=0.75)

args = RigorousValidationArgs(
    ransac_reproj_thresh=1.0, min_inliers=20, min_inlier_ratio=0.3,
    max_median_dy=1.5, max_p95_dy=3.0,
    max_median_dy_norm=None, max_p95_dy_norm=None,
    max_median_sampson=5.0,
)
metrics, inlier_matches = rigorous_epipolar_validation(kp1, kp2, matches, transforms, args)
print(metrics["reason"])  # "PASS" or failure details
```
