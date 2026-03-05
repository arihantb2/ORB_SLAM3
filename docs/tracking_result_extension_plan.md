# Plan: Extend TrackingResult with Rich Keypoint & Map Point Data

## Context

The existing `TrackingResult` struct (and its sub-structs `MotionModelTrackingResult`,
`RefKeyFrameTrackingResult`, `LocalMapTrackingResult`) already has the scaffolding for
keypoint match data, but the relevant fields are declared with TODO comments and never
populated. This plan fills those gaps and adds four new categories of data needed to
build a visual debugging/introspection tool for ORB-SLAM3.

The four things to capture:
1. Detected keypoints in the input image (with stereo match pairs for stereo pinhole)
2. Frame-to-frame matches from motion model tracking and reference KF tracking
3. Matches from local map tracking (inlier/outlier map point observations)
4. New map point candidates with 3D positions in camera frame

---

## Critical Files

| File | Role |
|------|------|
| `include/Tracking.h` (lines 52–136) | All result struct definitions — primary edit target |
| `src/Tracking.cc` | Tracking pipeline — populate fields inline |
| `include/Frame.h` | `mvKeysUn`, `mvKeysRight`, `mvuRight`, `mvDepth`, `mvpMapPoints`, `mvbOutlier`, `N`, `Nleft`, `UnprojectStereo()` |
| `include/MapPoint.h` | `GetWorldPos()`, `GetObservations()`, `mnId`, `mnFirstFrame` |
| `include/KeyFrame.h` | `mvKeysUn`, `mvKeysRight` — used when resolving ref-KF keypoint indices |

---

## New Helper Structs (add to `include/Tracking.h`, before existing structs)

### 1. `MatchedKeypoint` — a keypoint match with indices for cross-referencing

```cpp
struct MatchedKeypoint
{
    int current_kp_idx = -1;    // Index in current Frame's keypoint vector
    int source_kp_idx  = -1;    // Index in source (last Frame / ref KF); -1 if unknown
    cv::KeyPoint current_kp;    // Keypoint in current frame (undistorted)
    cv::KeyPoint source_kp;     // Keypoint in source frame/KF (undistorted)
    bool is_inlier = false;     // Set after pose optimization
};
```

### 2. `MapPointObservation` — a tracked map point associated with a keypoint

```cpp
struct MapPointObservation
{
    int keypoint_idx = -1;              // Index in current frame
    cv::KeyPoint keypoint;              // Keypoint in current frame (undistorted)
    Eigen::Vector3f pos_world;          // MapPoint world position
    Eigen::Vector3f pos_camera;         // World position transformed to camera frame
    unsigned long map_point_id = 0;     // MapPoint::mnId for cross-referencing
    bool is_inlier = true;              // Inlier flag after pose optimization
};
```

### 3. `FrameKeypointData` — all detected keypoints + stereo matching results

```cpp
struct FrameKeypointData
{
    // Mono: undistorted keypoints (mvKeysUn)
    // Stereo pinhole: left keypoints are in rectified/undistorted image (mvKeysUn)
    std::vector<cv::KeyPoint> left_keypoints;   // mvKeysUn  (all sensors)
    std::vector<cv::KeyPoint> right_keypoints;  // mvKeysRight (stereo only, else empty)

    // Stereo matching results (stereo only; empty for mono):
    // stereo_right_u[i] = right-image u-coordinate for left keypoint i (-1 = no match)
    // stereo_depth[i]   = depth in meters (-1 = no match)
    std::vector<float> stereo_right_u;  // mCurrentFrame.mvuRight
    std::vector<float> stereo_depth;    // mCurrentFrame.mvDepth

    // Convenience: matched (left_kp_idx, reconstructed right-image KeyPoint) pairs
    // right KeyPoint has pt = (mvuRight[i], mvKeysUn[i].pt.y), same octave as left
    std::vector<std::pair<int, cv::KeyPoint>> stereo_match_pairs;
};
```

### 4. `NewMapPointCandidate` — stereo keypoint with depth, not yet a tracked map point

```cpp
// Captures keypoints that have valid stereo depth but are NOT currently matched to any
// map point. These are the keypoints that will become new map points if this frame is
// promoted to a KeyFrame (LocalMapping triangulates/assigns them asynchronously).
struct NewMapPointCandidate
{
    int keypoint_idx = -1;          // Index in current frame
    cv::KeyPoint left_kp;           // Left image keypoint (undistorted)
    cv::KeyPoint right_kp;          // Right image keypoint (reconstructed from mvuRight)
    float depth = -1.f;             // Stereo depth in meters
    Eigen::Vector3f pos_world;      // 3D position in world frame (from UnprojectStereo)
    Eigen::Vector3f pos_camera;     // 3D position in current camera frame (Tcw * pos_world)
};
```

---

## Changes to Existing Result Structs

### `MotionModelTrackingResult` — add after existing fields

```cpp
// Rich match data with indices (replaces the unpopulated keypoints_matches fields)
std::vector<MatchedKeypoint> frame_matches;           // Before pose optimization
std::vector<MatchedKeypoint> frame_matches_optimized; // After (is_inlier set)
```

The existing `keypoints_matches`, `keypoints_inliers_optimized`, `keypoints_outliers_optimized`,
`keypoints_matches_optimized` fields will also be **populated** (implementing existing TODOs)
for backwards compatibility.

### `RefKeyFrameTrackingResult` — add after existing fields

```cpp
std::vector<MatchedKeypoint> kf_matches;              // Before pose optimization
std::vector<MatchedKeypoint> kf_matches_optimized;    // After (is_inlier set)
```

Existing TODO fields also populated.

### `LocalMapTrackingResult` — add after existing fields

```cpp
std::vector<MapPointObservation> inlier_observations;  // Non-outlier map points
std::vector<MapPointObservation> outlier_observations; // Outlier map points
// Note: pose field (already declared) will now be populated inline
```

Existing `keypoints_inliers`, `keypoints_outliers`, `keypoints_matches` fields populated.

### `TrackingResult` — add after existing fields

```cpp
// (1) Raw keypoint detections + stereo match pairs
FrameKeypointData keypoint_data;

// (3) All map points visible in this frame after full tracking (superset of local_map_result)
std::vector<MapPointObservation> all_tracked_map_points;

// (4) Stereo keypoints with depth but no existing map point assignment.
//     These are candidates that LocalMapping will turn into map points asynchronously.
//     For monocular, this vector is always empty.
std::vector<NewMapPointCandidate> new_map_point_candidates;
```

---

## Implementation: Where to Populate Each Field

### A. `FrameKeypointData keypoint_data` → in `Track()` (src/Tracking.cc ~line 540)

Populate **at the very start of `Track()`**, before any tracking attempt, directly from
`mCurrentFrame` (which is fully set up by the time `Track()` is called):

```cpp
TrackingResult tracking_result;

// --- Populate keypoint_data ---
tracking_result.keypoint_data.left_keypoints  = mCurrentFrame.mvKeysUn;
tracking_result.keypoint_data.right_keypoints = mCurrentFrame.mvKeysRight; // empty for mono
tracking_result.keypoint_data.stereo_right_u  = mCurrentFrame.mvuRight;
tracking_result.keypoint_data.stereo_depth    = mCurrentFrame.mvDepth;

for (int i = 0; i < mCurrentFrame.N; i++) {
    if (mCurrentFrame.mvuRight[i] >= 0) {
        cv::KeyPoint right_kp = mCurrentFrame.mvKeysUn[i];
        right_kp.pt.x = mCurrentFrame.mvuRight[i];
        tracking_result.keypoint_data.stereo_match_pairs.push_back({i, right_kp});
    }
}
```

`mvuRight` is initialized to -1 for all keypoints in mono (Frame constructor), so this
is safe to run unconditionally regardless of sensor type.

---

### B. Motion model matches → in `TrackWithMotionModel()` (src/Tracking.cc ~line 1566)

**After `SearchByProjection(mCurrentFrame, mLastFrame, th, ...)` and before the retry block:**

Build a reverse lookup (MapPoint* → last-frame keypoint index) then iterate current frame:

```cpp
// Build reverse map: MapPoint → index in last frame
std::unordered_map<MapPoint*, int> lastFramePointIdx;
for (int j = 0; j < mLastFrame.N; j++) {
    if (mLastFrame.mvpMapPoints[j])
        lastFramePointIdx[mLastFrame.mvpMapPoints[j]] = j;
}

for (int i = 0; i < mCurrentFrame.N; i++) {
    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
    if (!pMP) continue;
    MatchedKeypoint m;
    m.current_kp_idx = i;
    m.current_kp     = mCurrentFrame.mvKeysUn[i];
    auto it = lastFramePointIdx.find(pMP);
    if (it != lastFramePointIdx.end()) {
        m.source_kp_idx = it->second;
        m.source_kp     = mLastFrame.mvKeysUn[it->second];
    }
    result.frame_matches.push_back(m);
    result.keypoints_matches.push_back({m.current_kp, m.source_kp}); // existing field
}
```

**After `DiscardOutliersAndCountInliers()`** (the outlier pass removes NULLed entries, so
we need to capture before AND after):

```cpp
for (int i = 0; i < mCurrentFrame.N; i++) {
    // After DiscardOutliersAndCountInliers, outlier entries are NULLed out
    // We captured pre-discard state in frame_matches; now record inlier/outlier status
    // by checking the original list against what remains
}
// Simpler: iterate frame_matches and check if mCurrentFrame.mvpMapPoints[m.current_kp_idx] != null
for (auto& m : result.frame_matches) {
    m.is_inlier = (mCurrentFrame.mvpMapPoints[m.current_kp_idx] != nullptr);
    if (m.is_inlier) {
        result.frame_matches_optimized.push_back(m);
        result.keypoints_inliers_optimized.push_back(m.current_kp);
        result.keypoints_matches_optimized.push_back({m.current_kp, m.source_kp});
    } else {
        result.keypoints_outliers_optimized.push_back(m.current_kp);
    }
}
```

This block goes **after** the existing `DiscardOutliersAndCountInliers()` call
(~line 1619) and before the success check.

**Note on retry:** `frame_matches` is populated from whichever search (initial or retry)
produces the final `mCurrentFrame.mvpMapPoints`. If retry is triggered, the initial
`mvpMapPoints` is cleared and rebuilt, so populate `frame_matches` only once, after
the final search.

---

### C. Ref KF matches → in `TrackReferenceKeyFrameWithBoW()` (src/Tracking.cc ~line 1468)

**Replace the existing empty loop with:**

```cpp
for (int i = 0; i < (int)vpMapPointMatches.size(); i++) {
    MapPoint* pMP = vpMapPointMatches[i];
    if (!pMP || pMP->isBad()) continue;

    MatchedKeypoint m;
    m.current_kp_idx = i;
    m.current_kp     = mCurrentFrame.mvKeysUn[i];

    // Resolve ref-KF keypoint via MapPoint observations
    auto obs = pMP->GetObservations();
    auto it  = obs.find(mpReferenceKF);
    if (it != obs.end()) {
        int refIdx    = std::get<0>(it->second); // left-image keypoint index in ref KF
        m.source_kp_idx = refIdx;
        m.source_kp     = mpReferenceKF->mvKeysUn[refIdx];
    }
    result.kf_matches.push_back(m);
    result.keypoints_matches.push_back({m.current_kp, m.source_kp}); // existing field
}
```

**Replace the existing empty post-optimization loop** (~line 1493):

```cpp
for (auto& m : result.kf_matches) {
    m.is_inlier = (mCurrentFrame.mvpMapPoints[m.current_kp_idx] != nullptr);
    if (m.is_inlier) {
        result.kf_matches_optimized.push_back(m);
        result.keypoints_inliers_optimized.push_back(m.current_kp);
        result.keypoints_matches_optimized.push_back({m.current_kp, m.source_kp});
    } else {
        result.keypoints_outliers_optimized.push_back(m.current_kp);
    }
}
```

---

### D. Local map observations → in `TrackLocalMap()` (src/Tracking.cc ~line 1687)

**Replace / augment the existing inlier-counting loop** (~line 1687–1704):

```cpp
Sophus::SE3f Tcw = mCurrentFrame.GetPose();

for (int i = 0; i < mCurrentFrame.N; i++) {
    if (!mCurrentFrame.mvpMapPoints[i]) continue;

    MapPointObservation obs;
    obs.keypoint_idx  = i;
    obs.keypoint      = mCurrentFrame.mvKeysUn[i];
    obs.pos_world     = mCurrentFrame.mvpMapPoints[i]->GetWorldPos();
    obs.pos_camera    = Tcw * obs.pos_world;
    obs.map_point_id  = mCurrentFrame.mvpMapPoints[i]->mnId;
    obs.is_inlier     = !mCurrentFrame.mvbOutlier[i];

    if (obs.is_inlier) {
        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
            mnMatchesInliers++;
        result.inlier_observations.push_back(obs);
        result.keypoints_inliers.push_back(obs.keypoint);   // existing field
    } else {
        if (mSensor == System::STEREO)
            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        result.outlier_observations.push_back(obs);
        result.keypoints_outliers.push_back(obs.keypoint);  // existing field
    }
}
result.pose = mCurrentFrame.GetPose().inverse();  // populate existing pose field
```

---

### E. `all_tracked_map_points` and `new_map_point_candidates` → end of `Track()` (~line 905)

**After `UpdateAfterTracking()` and just before `return tracking_result`:**

```cpp
// (3) Collect all remaining inlier map point observations for this frame
if (tracking_result.success) {
    Sophus::SE3f Tcw = mCurrentFrame.GetPose();
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (!mCurrentFrame.mvpMapPoints[i] || mCurrentFrame.mvbOutlier[i]) continue;
        MapPointObservation obs;
        obs.keypoint_idx = i;
        obs.keypoint     = mCurrentFrame.mvKeysUn[i];
        obs.pos_world    = mCurrentFrame.mvpMapPoints[i]->GetWorldPos();
        obs.pos_camera   = Tcw * obs.pos_world;
        obs.map_point_id = mCurrentFrame.mvpMapPoints[i]->mnId;
        obs.is_inlier    = true;
        tracking_result.all_tracked_map_points.push_back(obs);
    }
}

// (4) Stereo keypoints with valid depth but unmatched (candidates for new map points)
if (mSensor == System::STEREO || mSensor == System::IMU_STEREO) {
    Sophus::SE3f Tcw = mCurrentFrame.GetPose();
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvDepth[i] <= 0) continue;
        if (mCurrentFrame.mvpMapPoints[i])  continue; // already tracked
        NewMapPointCandidate c;
        c.keypoint_idx = i;
        c.left_kp      = mCurrentFrame.mvKeysUn[i];
        c.depth        = mCurrentFrame.mvDepth[i];
        // Reconstruct right keypoint from mvuRight
        c.right_kp     = mCurrentFrame.mvKeysUn[i];
        c.right_kp.pt.x = mCurrentFrame.mvuRight[i];
        // 3D position
        Eigen::Vector3f x3D;
        if (mCurrentFrame.UnprojectStereo(i, x3D)) {
            c.pos_world  = x3D;                // UnprojectStereo returns world coords
            c.pos_camera = Tcw * x3D;
        }
        tracking_result.new_map_point_candidates.push_back(c);
    }
}
```

---

## LocalMapping Async Limitation

`LocalMapping::CreateNewMapPoints()` triangulates new map points in a separate thread
**after** the tracking result has already been returned to the caller. Therefore,
triangulated map points from two-view matching **cannot** be captured synchronously in
`TrackingResult`.

**What we do capture instead (sufficient for visual debugging):**
- `new_map_point_candidates`: stereo keypoints with valid depth that have no existing
  map point — these are the exact keypoints LocalMapping will promote to map points
  when it processes the next KeyFrame.
- `all_tracked_map_points`: all currently visible/tracked map points with their 3D
  positions, which includes any map points that were previously created and are now
  being tracked.

**Future extension** (out of scope for this PR): Add a thread-safe callback/queue in
`LocalMapping` where `CreateNewMapPoints()` posts `(frame_id, kp_idx, MapPoint*)` tuples.
The caller can query these after the fact keyed by frame ID.

---

## Header Organization (`include/Tracking.h`)

Insert new structs **before** `MotionModelTrackingResult` (line 52) in this order:
1. `MatchedKeypoint`
2. `MapPointObservation`
3. `FrameKeypointData`
4. `NewMapPointCandidate`

Required additional includes at top of `Tracking.h`:
```cpp
#include <Eigen/Core>  // already present transitively via Sophus, but make explicit
#include <unordered_map>  // for reverse-lookup map in TrackWithMotionModel
```

---

## Verification

1. **Build**: `cmake --build build -j$(nproc)` — should compile with no new warnings.
2. **Stereo smoke test**: Run a stereo example dataset; check that:
   - `keypoint_data.left_keypoints.size() == mCurrentFrame.N`
   - `keypoint_data.stereo_match_pairs` is non-empty and each `right_kp.pt.x >= 0`
   - `motion_model_result.frame_matches` non-empty after 2nd frame
   - `local_map_result.inlier_observations` non-empty and `pos_camera.z() > 0`
   - `new_map_point_candidates` non-empty for frames that become keyframes
3. **Mono smoke test**: Check `right_keypoints` and `stereo_*` fields are empty.
4. **Existing fields**: Confirm `num_matches`, `num_matches_optimized` unchanged.
5. **Cross-check**: For a match in `frame_matches`, verify `current_kp_idx` refers to
   the same 2D position shown in `all_tracked_map_points[*].keypoint`.
