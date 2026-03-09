% ORB-SLAM3: TrackingResult Rich Data

## Overview

`TrackingResult` is the per-frame output of `Tracking::Track()` and is returned by:

- `Tracking::GrabImageStereo(const cv::Mat& imageLeft, const cv::Mat& imageRight, ...)`
- `Tracking::GrabImageMonocular(const cv::Mat& image, ...)`

The original fields (success flags, match counts, simple keypoint vectors, final pose) are preserved for backward compatibility. This document describes the **richer introspection data** added on top, intended for client libraries that want to:

- Visualize detected features, matches, and map points.
- Inspect inlier/outlier decisions of the pose optimizers.
- Understand which stereo keypoints are candidates for future map points.
- Work offline with self-contained tracking snapshots, including the input images.

Unless stated otherwise, all of the fields below live in the `ORB_SLAM3::TrackingResult` struct defined in `include/Tracking.h`.

---

## Structs and Fields

### FrameKeypointData (`TrackingResult::keypoint_data`)

**Purpose:** snapshot of all ORB keypoints detected in the current frame, plus stereo metadata (if available).

Fields:

- `std::vector<cv::KeyPoint> left_keypoints`
  - Copy of `Frame::mvKeysUn`.
  - Undistorted keypoints in the **left** image (or the single image in monocular).
- `std::vector<cv::KeyPoint> right_keypoints`
  - Copy of `Frame::mvKeysRight`.
  - Non-empty only for stereo sensors.
- `std::vector<float> stereo_right_u`
  - Copy of `Frame::mvuRight`.
  - Same length as `left_keypoints` (and `Frame::N`).
  - `stereo_right_u[i] >= 0` ⇒ a valid right-image match for left keypoint `i`.
  - `stereo_right_u[i] < 0` ⇒ no valid stereo match.
- `std::vector<float> stereo_depth`
  - Copy of `Frame::mvDepth`.
  - `stereo_depth[i] > 0` ⇒ valid depth in metres; derived from stereo disparity.
  - `stereo_depth[i] <= 0` ⇒ no valid stereo depth.
- `std::vector<std::pair<int, cv::KeyPoint>> stereo_match_pairs`
  - Convenience list for visualisation.
  - Each entry: `(left_index, right_kp)`, where:
    - `left_index` is an index into `left_keypoints`.
    - `right_kp` is a synthetic right-image `cv::KeyPoint` whose `pt.x` is `stereo_right_u[left_index]` and `pt.y` is copied from the left keypoint.

**Population and behaviour:**

- Filled **once per frame**, at the start of `Tracking::Track()`, from the current `Frame`:
  - Runs for **all** sensor types; for monocular sensors the stereo-specific fields follow the `Frame` convention (typically all depths and right-u values are `-1` / non-positive).
- For frames with **no detected keypoints**, all vectors are empty.

**Edge cases:**

- Monocular:
  - `right_keypoints` is empty.
  - `stereo_right_u` and `stereo_depth` still have length `N`, but values should be treated as “no stereo” (usually `-1` or non-positive).
  - `stereo_match_pairs` is empty.
- Stereo, but with tracking failure:
  - `keypoint_data` is **always** filled if `mCurrentFrame` was constructed; it does not depend on successful pose estimation.

---

### MatchedKeypoint

**Purpose:** describe a **single feature match** between the current frame and a source frame (either the last frame or the reference keyframe), with keypoint indices, geometry, and inlier/outlier status.

Fields:

- `int current_kp_idx`
  - Index into `Frame::mvKeysUn` of the **current** frame.
  - `-1` means “not set”; clients should guard against negative values.
- `int source_kp_idx`
  - Index into `mvKeysUn` of the **source** frame:
    - For motion-model matches: index into `mLastFrame.mvKeysUn`.
    - For ref-keyframe matches: index into `mpReferenceKF->mvKeysUn`.
  - `-1` if the source index could not be resolved (for example, when the map point has no observation in the reference keyframe).
- `cv::KeyPoint current_kp`
  - Undistorted current-frame keypoint (`mvKeysUn[current_kp_idx]`).
- `cv::KeyPoint source_kp`
  - Undistorted keypoint in the source frame/KF.
  - May be left default-constructed if `source_kp_idx == -1`.
- `bool is_inlier`
  - `false` by default when the match is first created.
  - Set to `true` if the match survives the corresponding pose-optimization outlier rejection.

**Where it appears:**

- `MotionModelTrackingResult::frame_matches`
  - All current ↔ last-frame matches *before* pose optimization.
- `MotionModelTrackingResult::frame_matches_optimized`
  - Subset of `frame_matches` with `is_inlier == true` *after* `DiscardOutliersAndCountInliers`.
- `RefKeyFrameTrackingResult::kf_matches`
  - All current ↔ reference-keyframe matches from `SearchByBoW`, before optimization.
- `RefKeyFrameTrackingResult::kf_matches_optimized`
  - Subset of `kf_matches` with `is_inlier == true` after outlier rejection.

**Relationship to legacy fields:**

- For each `MatchedKeypoint m` we also populate:
  - Pre-optimization:
    - `keypoints_matches.push_back({m.current_kp, m.source_kp});`
  - Post-optimization (inliers only):
    - `keypoints_inliers_optimized.push_back(m.current_kp);`
    - `keypoints_matches_optimized.push_back({m.current_kp, m.source_kp});`
  - Outliers:
    - `keypoints_outliers_optimized.push_back(m.current_kp);`

Existing client code that only uses the legacy vectors will see consistent behaviour.

**Edge cases:**

- `source_kp_idx == -1`:
  - Occurs when the map point has no observation for the chosen source frame/KF.
  - In this case `source_kp` is a default `cv::KeyPoint`; clients should check the index before using it.
- `is_inlier` is **only meaningful** in the `*_matches_optimized` variants; in the base vectors it is left at the default `false`.

---

### MapPointObservation

**Purpose:** describe a single **map point observation** in the current frame, including 2D keypoint and 3D position in both world and camera frames.

Fields:

- `int keypoint_idx`
  - Index into `Frame::mvKeysUn` of the observing keypoint.
- `cv::KeyPoint keypoint`
  - Copy of `mvKeysUn[keypoint_idx]`.
- `Eigen::Vector3f pos_world`
  - `MapPoint::GetWorldPos()` at the time of observation.
  - Expressed in the **world** frame.
- `Eigen::Vector3f pos_camera`
  - Point in the **camera** frame: `pos_camera = Tcw * pos_world`, where `Tcw = mCurrentFrame.GetPose()`.
- `unsigned long map_point_id`
  - `MapPoint::mnId` — unique across the map.
- `bool is_inlier`
  - `true` if the observation was treated as an inlier for the current pose.
  - `false` if it was considered an outlier (e.g. large reprojection error).

**Where it appears:**

- `LocalMapTrackingResult::inlier_observations`
  - Filled in `TrackLocalMap()` for every inlier observation, alongside:
    - `keypoints_inliers` (legacy).
- `LocalMapTrackingResult::outlier_observations`
  - Outlier observations in the local-map phase, alongside:
    - `keypoints_outliers` (legacy).
- `TrackingResult::all_tracked_map_points`
  - Filled at the end of `Track()`, after `UpdateAfterTracking()`:
  - Contains **all** inlier map points observed in the frame after the complete pipeline (motion model / ref-KF + local map).

**Edge cases:**

- If pose estimation fails and `tracking_result.success == false`, `local_map_result.success` may also be `false`, and both the `LocalMapTrackingResult` observation vectors and `all_tracked_map_points` may be empty.
- For frames where `mCurrentFrame.isSet() == false` (rare, mostly error paths), `all_tracked_map_points` is left empty.

---

### NewMapPointCandidate (`TrackingResult::new_map_point_candidates`)

**Purpose:** describe keypoints that have valid stereo depth but are **not yet associated** to any map point. These are the exact inputs `LocalMapping` will use to create new `MapPoint` instances if the frame is promoted to a keyframe.

Fields:

- `int keypoint_idx`
  - Index into `Frame::mvKeysUn` of the left keypoint.
- `cv::KeyPoint left_kp`
  - Copy of `mvKeysUn[keypoint_idx]`.
- `cv::KeyPoint right_kp`
  - Synthetic right-image keypoint, with:
    - `pt.x = mvuRight[keypoint_idx]`
    - `pt.y` copied from `left_kp.pt.y`.
- `float depth`
  - Copy of `mvDepth[keypoint_idx]`, in metres.
  - `depth > 0` ⇒ valid stereo depth.
- `Eigen::Vector3f pos_world`
  - 3D point in world coordinates, from `Frame::UnprojectStereo(keypoint_idx, x3D)`.
- `Eigen::Vector3f pos_camera`
  - `Tcw * pos_world`, same convention as `MapPointObservation`.

**Population and behaviour:**

- Populated in `Track()` after `UpdateAfterTracking()` and before LOST handling, but **only** for stereo sensors:
  - `mSensor == System::STEREO || mSensor == System::IMU_STEREO`.
- Inclusion criteria:
  - `mvDepth[i] > 0.f` (valid stereo depth).
  - `mvpMapPoints[i] == nullptr` (no existing map point tracked at this keypoint).

**Edge cases:**

- Monocular (and IMU-monocular) sensors:
  - `new_map_point_candidates` is always empty.
- If `UnprojectStereo(i, x3D)` fails:
  - `pos_world` and `pos_camera` are left at their default `Eigen::Vector3f` values (zero-initialised).
  - Clients that care about 3D geometry should gate on `depth > 0` **and** optionally check `pos_camera.z() > 0`.

---

### Images (`TrackingResult::image_left`, `TrackingResult::image_right`)

**Purpose:** make each `TrackingResult` self-contained for offline analysis by including the actual grayscale input images used for feature extraction.

Fields:

- `cv::Mat image_left`
  - Always populated:
    - Stereo: left rectified grayscale image.
    - Monocular: (optionally undistorted) grayscale image, as passed into `GrabImageMonocular`.
- `cv::Mat image_right`
  - Stereo: right rectified grayscale image.
  - Monocular: default-constructed and empty (`image_right.empty() == true`).

**Population and behaviour:**

- In `GrabImageStereo()`:
  - `TrackingResult result = Track();`
  - `result.image_left = imageLeft.clone();`
  - `result.image_right = imageRight.clone();`
- In `GrabImageMonocular()`:
  - `TrackingResult result = Track();`
  - `result.image_left = image.clone();`
- `.clone()` is used so the `TrackingResult` owns its image data; callers may keep it beyond the scope of `GrabImage`*.

**Edge cases:**

- Early-return paths in `Track()` that produce a default `TrackingResult` (e.g. IMU errors, timestamp jumps) still pass through `GrabImage`*, so **images are always attached** for any call that returns to user code.

---

## Success Flags and When Data Is Valid

### Top-level `TrackingResult::success`

- `success == true` means:
  - Pose estimation succeeded (`TrackWithMotionModel` and/or `TrackReferenceKeyFrameWithBoW`), **and**
  - `TrackLocalMap()` accepted the result (per its inlier thresholds and IMU state).
- For many client use cases, this is the primary gate:
  - If you want only frames with a “good” pose, check `tracking_result.success` first.

### Sub-result `success` flags

- `motion_model_result.success`
  - True when motion-model tracking produced a valid pose and inlier count for the sensor mode.
  - May be `true` even when `tracking_result.success == false` if local-map tracking or thresholds failed later.
- `ref_key_frame_result.success`
  - True when ref-keyframe tracking (primary or fallback) produced an acceptable pose.
- `local_map_result.success`
  - True when `TrackLocalMap()` accepted the frame (inlier counts above its thresholds).

### LOST state and debug data

- In the main `Track()` function:
  - `UpdateAfterTracking(tracking_result.success);`
  - `all_tracked_map_points` and `new_map_point_candidates` are filled **before** checking `mState == LOST`.
- This means:
  - Even when the tracker declares the camera LOST on this frame, the returned `TrackingResult` still includes:
    - All inlier map points it was using for the last pose.
    - All stereo keypoints that could have been turned into new map points.
  - This is especially useful for **post-mortem visualisation** and debugging.
- After the LOST frame is returned, `ResetActiveMap()` is called and the next frame starts from a fresh map.

---

## Typical Usage Patterns

### Visualising keypoints and matches

- Use `image_left` (and `image_right` for stereo) as the background.
- Draw:
  - All detected keypoints from `keypoint_data.left_keypoints`.
  - Motion-model matches:
    - `motion_model_result.frame_matches` for pre-optimization.
    - `motion_model_result.frame_matches_optimized` for final inliers.
  - Ref-keyframe matches:
    - `ref_key_frame_result.kf_matches` / `kf_matches_optimized`.
- For each `MatchedKeypoint m`:
  - Current-frame pixel: `m.current_kp.pt`.
  - Source-frame pixel:
    - Motion model: `m.source_kp.pt` in the **last frame**.
    - Ref keyframe: `m.source_kp.pt` in the **reference keyframe**.

### Visualising tracked map points

- Use `all_tracked_map_points`:
  - Each entry provides `pos_world`, `pos_camera`, and `map_point_id`.
  - `pos_world` lets you plot the map in global coordinates.
  - `pos_camera` (with `z > 0`) can be projected into the image plane with the camera intrinsics if needed.
- For local-map-specific views (e.g. showing which points were available for local BA), use:
  - `local_map_result.inlier_observations`
  - `local_map_result.outlier_observations`

### Inspecting new map-point candidates

- For stereo frames near a keyframe insertion:
  - Inspect `new_map_point_candidates` to see:
    - Where the system could create new map points.
    - The distribution of depths and viewing angles.
  - Useful for:
    - Tuning stereo matching or depth thresholds.
    - Visualising the “potential” map that LocalMapping will create.

---

## Edge Cases Summary

- **Monocular sensors:**
  - `right_keypoints`, `stereo_match_pairs`, and `new_map_point_candidates` are empty.
  - `stereo_right_u` / `stereo_depth` are present but encode “no stereo” (non-positive values).
- **No features in frame:**
  - `keypoint_data` vectors are empty.
  - All match/vector fields remain empty.
- **Initialization / NOT_INITIALIZED:**
  - `keypoint_data` is still populated (if the frame exists).
  - Match and map-point vectors are generally empty; use with caution.
- **LOST state:**
  - On the LOST frame:
    - `all_tracked_map_points` and `new_map_point_candidates` are still populated (if `mCurrentFrame.isSet()`).
  - On subsequent frames (after reset):
    - These fields reflect the new map; do not attempt to join IDs across reset events without additional logic.
- **IMU vs visual-only modes:**
  - `*_result.success` thresholds differ between IMU and visual-only sensors.
  - For consistent behaviour across sensors, prefer gating on `TrackingResult::success` and treating sub-result fields as diagnostics.

---

## Backward Compatibility

- All existing `TrackingResult` fields have been preserved.
- Legacy vectors (`keypoints_matches`, `keypoints_inliers_optimized`, `keypoints_outliers_optimized`, etc.) are still populated exactly as before, with richer structs layered alongside them.
- Client libraries that only read the original fields do not need to change.
- New clients should:
  - Use the rich structs (`FrameKeypointData`, `MatchedKeypoint`, `MapPointObservation`, `NewMapPointCandidate`) for introspection.
  - Use the legacy vectors only when interacting with older code that expects them.

