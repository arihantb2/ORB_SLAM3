# Plan: Extend TrackingResult with Rich Keypoint & Map Point Data

## Goal

Extend the existing `TrackingResult` struct hierarchy in ORB-SLAM3 to expose five new
categories of data useful for building a visual debugging and introspection tool:

1. **Detected keypoints** in the input image (undistorted), plus stereo match pairs for
   stereo-pinhole mode (left-right correspondences with depth).
2. **Frame-to-frame matches** from motion model tracking and reference keyframe tracking,
   with keypoint indices so both endpoints can be looked up, plus inlier/outlier status
   after pose optimization.
3. **Local map tracking matches**: every map point observation (inlier or outlier) after
   `TrackLocalMap()`, with 3D world position and 3D camera-frame position.
4. **New map point candidates**: for stereo mode, every keypoint that has valid stereo
   depth but is currently unmatched to any map point. These are the keypoints that
   `LocalMapping` will triangulate/assign when the frame is promoted to a KeyFrame.
5. **The input images themselves** (rectified/undistorted grayscale) so that a
   `TrackingResult` is fully self-contained for offline visualisation without needing to
   keep a copy of the original frame.

All data is populated **inline** at the point of computation. Existing fields are
preserved for backward compatibility; new richer fields are added alongside them.

---

## File Index

| File | What changes |
|------|-------------|
| `include/Tracking.h` | Add 4 new helper structs; add new fields to 4 existing structs |
| `src/Tracking.cc` | Populate all new (and existing TODO) fields at exact call sites |

Read-only references (no changes):

| File | Why referenced |
|------|---------------|
| `include/Frame.h` | `mvKeysUn`, `mvKeysRight`, `mvuRight`, `mvDepth`, `mvpMapPoints`, `mvbOutlier`, `N`, `UnprojectStereo()` |
| `include/MapPoint.h` | `GetWorldPos()` → `Eigen::Vector3f`; `GetObservations()` → `std::map<KeyFrame*, std::tuple<int,int>>`; `mnId`; `isBad()` |
| `include/KeyFrame.h` | `mvKeysUn` — used to resolve reference-KF keypoint coordinates |

---

## Part 1 — `include/Tracking.h`: New Helper Structs and New Fields

### 1a. Add `#include <unordered_map>` to the existing include block

Current includes (lines 22–29):
```cpp
#include <list>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>
```

Add `#include <unordered_map>` after `#include <mutex>`:
```cpp
#include <list>
#include <mutex>
#include <unordered_map>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>
```

`<unordered_map>` is needed for the reverse-lookup in `TrackWithMotionModel()`.

---

### 1b. Insert 4 new helper structs before `struct MotionModelTrackingResult` (currently line 52)

Insert the following block immediately before the line:
```cpp
struct MotionModelTrackingResult
```

```cpp
// ---------------------------------------------------------------------------
// Helper structs for rich tracking introspection data
// ---------------------------------------------------------------------------

// A single feature match between two images, with keypoint indices for
// cross-referencing and an inlier flag set after pose optimization.
struct MatchedKeypoint
{
    int current_kp_idx = -1;  // Index into current Frame's mvKeysUn
    int source_kp_idx  = -1;  // Index into source Frame's/KF's mvKeysUn; -1 if unknown
    cv::KeyPoint current_kp;  // Keypoint in current frame (undistorted)
    cv::KeyPoint source_kp;   // Keypoint in source frame/KF (undistorted)
    bool is_inlier = false;   // True if this match survives pose optimization
};

// A map point that is currently observed in the current frame, with its
// 3D position expressed in both world and camera frames.
struct MapPointObservation
{
    int keypoint_idx = -1;           // Index into current Frame's mvKeysUn
    cv::KeyPoint keypoint;           // Keypoint in current frame (undistorted)
    Eigen::Vector3f pos_world;       // MapPoint world position (GetWorldPos())
    Eigen::Vector3f pos_camera;      // pos_world transformed to camera frame: Tcw * pos_world
    unsigned long map_point_id = 0;  // MapPoint::mnId — unique across the map
    bool is_inlier = true;           // False if marked as outlier by pose optimization
};

// All ORB keypoints detected in the current frame, plus stereo matching
// results for stereo-pinhole mode.
struct FrameKeypointData
{
    // Left/mono keypoints in the undistorted image plane (Frame::mvKeysUn).
    // For stereo-pinhole the image is already rectified, so these are also
    // undistorted. For monocular the standard undistortion is applied.
    std::vector<cv::KeyPoint> left_keypoints;

    // Right-image keypoints (Frame::mvKeysRight). Empty for monocular.
    std::vector<cv::KeyPoint> right_keypoints;

    // Per-left-keypoint stereo matching results (stereo only, else empty).
    // Indexed the same as left_keypoints (i.e. index i corresponds to left_keypoints[i]).
    // stereo_right_u[i] = u-coordinate of the match on the right image (-1 = no match).
    // stereo_depth[i]   = depth in metres derived from the stereo baseline (-1 = no match).
    // Source: Frame::mvuRight and Frame::mvDepth respectively.
    std::vector<float> stereo_right_u;
    std::vector<float> stereo_depth;

    // Convenience list of matched stereo pairs for visualisation.
    // Each entry is (left_kp_index, reconstructed right-image KeyPoint).
    // The right KeyPoint has pt.x = stereo_right_u[i], pt.y = left_keypoints[i].pt.y,
    // and the same octave/response/size as the left keypoint.
    std::vector<std::pair<int, cv::KeyPoint>> stereo_match_pairs;
};

// A keypoint that has valid stereo depth but is NOT currently matched to any
// tracked map point. If this frame is promoted to a KeyFrame, LocalMapping
// will create a new MapPoint for each of these. 3D positions are computed
// inline via Frame::UnprojectStereo(). Empty for monocular.
struct NewMapPointCandidate
{
    int keypoint_idx = -1;       // Index into current Frame's mvKeysUn
    cv::KeyPoint left_kp;        // Left undistorted keypoint
    cv::KeyPoint right_kp;       // Reconstructed right keypoint (pt.x = mvuRight[i])
    float depth = -1.f;          // Stereo depth in metres (Frame::mvDepth[i])
    Eigen::Vector3f pos_world;   // 3D world position (from UnprojectStereo)
    Eigen::Vector3f pos_camera;  // 3D camera-frame position: Tcw * pos_world
};

// ---------------------------------------------------------------------------
```

---

### 1c. Add new fields to `MotionModelTrackingResult`

Current struct ends at (approx. line 79):
```cpp
    // Optimized Pose
    Sophus::SE3f pose;
};
```

Add new fields immediately before the closing `};`:
```cpp
    // Rich match data with keypoint indices (supersedes the pair-based fields above,
    // which are also populated for backward compatibility).
    // frame_matches: all current↔last-frame matches after the final SearchByProjection,
    //   before pose optimization. is_inlier is false at this point.
    // frame_matches_optimized: subset where is_inlier == true after DiscardOutliersAndCountInliers.
    std::vector<MatchedKeypoint> frame_matches;
    std::vector<MatchedKeypoint> frame_matches_optimized;

    // Optimized Pose
    Sophus::SE3f pose;
};
```

---

### 1d. Add new fields to `RefKeyFrameTrackingResult`

Same pattern — add before the closing `};` of that struct:
```cpp
    // Rich match data with keypoint indices.
    // kf_matches: all current↔ref-KF matches from SearchByBoW, before optimization.
    // kf_matches_optimized: inlier subset after DiscardOutliersAndCountInliers.
    std::vector<MatchedKeypoint> kf_matches;
    std::vector<MatchedKeypoint> kf_matches_optimized;

    // Optimized Pose
    Sophus::SE3f pose;
};
```

---

### 1e. Add new fields to `LocalMapTrackingResult`

Add before the closing `};` of `LocalMapTrackingResult`:
```cpp
    // Full map point observations split by inlier/outlier status.
    // Populated inside TrackLocalMap() during the inlier-counting loop.
    std::vector<MapPointObservation> inlier_observations;
    std::vector<MapPointObservation> outlier_observations;

    // Pose
    Sophus::SE3f pose;
};
```

Note: `pose` is already declared in the existing struct; do not duplicate it. Add only the two observation vectors before the existing `pose` line.

---

### 1f. Add new fields to `TrackingResult`

Add before the closing `};` of `TrackingResult`:
```cpp
    // (1) All ORB keypoints detected in this frame, plus stereo matching results.
    //     Populated at the start of Track() from mCurrentFrame, before any tracking.
    FrameKeypointData keypoint_data;

    // (3) All map points that are inlier observations in this frame after the full
    //     tracking pipeline (TrackLocalMap + pose optimization). Superset of
    //     local_map_result.inlier_observations because it is populated at the very
    //     end of Track() after UpdateAfterTracking().
    std::vector<MapPointObservation> all_tracked_map_points;

    // (4) Stereo keypoints with valid depth that are NOT matched to any existing map
    //     point after tracking. These are candidates that LocalMapping will turn into
    //     new MapPoints. Always empty for monocular.
    std::vector<NewMapPointCandidate> new_map_point_candidates;

    Sophus::SE3f pose;
};
```

Note: `pose` is already declared. Add the three new fields before it.

---

## Part 2 — `src/Tracking.cc`: Populate Fields Inline

### Touch point A — `Track()`, line 821: populate `keypoint_data`

Current code at line 821:
```cpp
    TrackingResult tracking_result;
    if (mState == NOT_INITIALIZED)
```

Replace with:
```cpp
    TrackingResult tracking_result;

    // Populate keypoint_data from mCurrentFrame immediately after frame construction.
    // mvuRight is initialised to -1 for every keypoint in mono (Frame constructor),
    // so this block is safe to run unconditionally for all sensor types.
    tracking_result.keypoint_data.left_keypoints  = mCurrentFrame.mvKeysUn;
    tracking_result.keypoint_data.right_keypoints = mCurrentFrame.mvKeysRight;
    tracking_result.keypoint_data.stereo_right_u  = mCurrentFrame.mvuRight;
    tracking_result.keypoint_data.stereo_depth    = mCurrentFrame.mvDepth;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvuRight[i] >= 0.f)
        {
            cv::KeyPoint right_kp = mCurrentFrame.mvKeysUn[i];
            right_kp.pt.x = mCurrentFrame.mvuRight[i];
            tracking_result.keypoint_data.stereo_match_pairs.push_back({i, right_kp});
        }
    }

    if (mState == NOT_INITIALIZED)
```

---

### Touch point B — `TrackWithMotionModel()`: populate `frame_matches` and fill existing TODO fields

#### B1 — Replace the empty TODO loop at line 1594–1597

Current code:
```cpp
    for (const auto& mapPoint : mCurrentFrame.mvpMapPoints)
    {
        // TODO: Populate result.keypoints_matches
    }
```

This loop appears **after** the retry block (lines 1577–1592) and **before** the
`nmatches < mMotionModelMinRetryMatches` check (line 1599).

Replace the entire loop with:
```cpp
    // Build a reverse lookup: MapPoint* -> index in mLastFrame.mvpMapPoints.
    // Used to find the last-frame keypoint that corresponds to each current-frame match.
    std::unordered_map<MapPoint*, int> lastFramePointIdx;
    lastFramePointIdx.reserve(mLastFrame.N);
    for (int j = 0; j < mLastFrame.N; j++)
    {
        if (mLastFrame.mvpMapPoints[j])
            lastFramePointIdx[mLastFrame.mvpMapPoints[j]] = j;
    }

    // Populate frame_matches (rich) and keypoints_matches (legacy pair vector).
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (!pMP) continue;

        MatchedKeypoint m;
        m.current_kp_idx = i;
        m.current_kp     = mCurrentFrame.mvKeysUn[i];
        auto it = lastFramePointIdx.find(pMP);
        if (it != lastFramePointIdx.end())
        {
            m.source_kp_idx = it->second;
            m.source_kp     = mLastFrame.mvKeysUn[it->second];
        }
        result.frame_matches.push_back(m);
        result.keypoints_matches.push_back({m.current_kp, m.source_kp}); // legacy field
    }
```

#### B2 — Replace the empty TODO loop at lines 1623–1628

Current code (immediately after `DiscardOutliersAndCountInliers` on line 1619):
```cpp
    for (const auto& mapPoint : mCurrentFrame.mvpMapPoints)
    {
        // TODO: Populate result.keypoints_inliers_optimized
        // TODO: Populate result.keypoints_outliers_optimized
        // TODO: Populate result.keypoints_matches_optimized
    }
```

Replace with:
```cpp
    // After DiscardOutliersAndCountInliers(), outlier entries in mvpMapPoints are
    // set to nullptr. Iterate frame_matches (captured before optimization) to
    // determine inlier/outlier status and populate optimized fields.
    for (auto& m : result.frame_matches)
    {
        m.is_inlier = (mCurrentFrame.mvpMapPoints[m.current_kp_idx] != nullptr);
        if (m.is_inlier)
        {
            result.frame_matches_optimized.push_back(m);
            result.keypoints_inliers_optimized.push_back(m.current_kp);
            result.keypoints_matches_optimized.push_back({m.current_kp, m.source_kp});
        }
        else
        {
            result.keypoints_outliers_optimized.push_back(m.current_kp);
        }
    }
```

**Important:** `frame_matches` is populated after the final `SearchByProjection` call
(either the initial search or the retry search, whichever runs last). Because the retry
path clears `mCurrentFrame.mvpMapPoints` with `fill(..., nullptr)` before re-running the
search, `frame_matches` will always reflect the final match set, not an intermediate one.
No special retry handling is needed.

---

### Touch point C — `TrackReferenceKeyFrameWithBoW()`: populate `kf_matches` and fill existing TODO fields

#### C1 — Replace the empty TODO loop at lines 1468–1471

Current code (immediately after `result.num_matches = nmatches;` on line 1466):
```cpp
    for (const auto& mapPoint : vpMapPointMatches)
    {
        // TODO: Populate result.keypoints_matches
    }
```

Replace with:
```cpp
    // vpMapPointMatches[i] is the MapPoint matched to current-frame keypoint i.
    // Resolve the reference-KF keypoint via the MapPoint's observation list.
    // GetObservations() returns std::map<KeyFrame*, std::tuple<int,int>> where
    // std::get<0>(value) is the left-image keypoint index in that KeyFrame.
    for (int i = 0; i < static_cast<int>(vpMapPointMatches.size()); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if (!pMP || pMP->isBad()) continue;

        MatchedKeypoint m;
        m.current_kp_idx = i;
        m.current_kp     = mCurrentFrame.mvKeysUn[i];

        auto obs = pMP->GetObservations();
        auto it  = obs.find(mpReferenceKF);
        if (it != obs.end())
        {
            int refIdx      = std::get<0>(it->second); // left-image kp index in ref KF
            m.source_kp_idx = refIdx;
            m.source_kp     = mpReferenceKF->mvKeysUn[refIdx];
        }
        result.kf_matches.push_back(m);
        result.keypoints_matches.push_back({m.current_kp, m.source_kp}); // legacy field
    }
```

#### C2 — Replace the empty TODO loop at lines 1493–1498

Current code (immediately after `DiscardOutliersAndCountInliers` on line 1491):
```cpp
    for (const auto& mapPoint : mCurrentFrame.mvpMapPoints)
    {
        // TODO: Populate result.keypoints_inliers_optimized
        // TODO: Populate result.keypoints_outliers_optimized
        // TODO: Populate result.keypoints_matches_optimized
    }
```

Replace with:
```cpp
    // After DiscardOutliersAndCountInliers(), outlier mvpMapPoints entries are nullptr.
    for (auto& m : result.kf_matches)
    {
        m.is_inlier = (mCurrentFrame.mvpMapPoints[m.current_kp_idx] != nullptr);
        if (m.is_inlier)
        {
            result.kf_matches_optimized.push_back(m);
            result.keypoints_inliers_optimized.push_back(m.current_kp);
            result.keypoints_matches_optimized.push_back({m.current_kp, m.source_kp});
        }
        else
        {
            result.keypoints_outliers_optimized.push_back(m.current_kp);
        }
    }
```

---

### Touch point D — `TrackLocalMap()`: replace the inlier-counting loop (lines 1687–1704)

Current code:
```cpp
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                {
                    mnMatchesInliers++;
                }
            }
            else if (mSensor == System::STEREO)
            {
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            }
        }
    }
```

Replace with:
```cpp
    mnMatchesInliers = 0;

    // Update MapPoints Statistics and populate observation vectors.
    const Sophus::SE3f Tcw = mCurrentFrame.GetPose();
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (!mCurrentFrame.mvpMapPoints[i]) continue;

        MapPointObservation obs;
        obs.keypoint_idx = i;
        obs.keypoint     = mCurrentFrame.mvKeysUn[i];
        obs.pos_world    = mCurrentFrame.mvpMapPoints[i]->GetWorldPos();
        obs.pos_camera   = Tcw * obs.pos_world;
        obs.map_point_id = mCurrentFrame.mvpMapPoints[i]->mnId;
        obs.is_inlier    = !mCurrentFrame.mvbOutlier[i];

        if (!mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
            if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                mnMatchesInliers++;
            result.inlier_observations.push_back(obs);
            result.keypoints_inliers.push_back(obs.keypoint);   // legacy field
        }
        else
        {
            if (mSensor == System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            result.outlier_observations.push_back(obs);
            result.keypoints_outliers.push_back(obs.keypoint);  // legacy field
        }
    }

    // Populate the pose field (was previously unpopulated).
    result.pose = mCurrentFrame.GetPose().inverse();
```

The `result.num_matches = mnMatchesInliers;` line at line 1706 remains unchanged after
this block.

---

### Touch point E — `Track()`, line 907: populate `all_tracked_map_points` and `new_map_point_candidates`

Current code at lines 907–922:
```cpp
        UpdateAfterTracking(tracking_result.success);

        // Reset if the camera get lost
        if (mState == LOST)
        {
            mpSystem->ResetActiveMap();
            return tracking_result;
        }

        if (!mCurrentFrame.mpReferenceKF)
        {
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
        mLastFrame = Frame(mCurrentFrame);

        tracking_result.pose = mCurrentFrame.GetPose().inverse();
    }
```

Insert **after** `UpdateAfterTracking(tracking_result.success);` and **before** the
`if (mState == LOST)` check, so these blocks also run on frames that become LOST (the
data is still useful for debugging):

```cpp
        UpdateAfterTracking(tracking_result.success);

        // Populate all_tracked_map_points: every inlier map point visible in this frame
        // after the complete tracking pipeline.
        if (mCurrentFrame.isSet())
        {
            const Sophus::SE3f Tcw = mCurrentFrame.GetPose();
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
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

        // Populate new_map_point_candidates: stereo keypoints with valid depth that
        // are NOT currently tracked as map points. LocalMapping will create new
        // MapPoints from these when this frame becomes a KeyFrame.
        if ((mSensor == System::STEREO || mSensor == System::IMU_STEREO) && mCurrentFrame.isSet())
        {
            const Sophus::SE3f Tcw = mCurrentFrame.GetPose();
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                if (mCurrentFrame.mvDepth[i] <= 0.f)  continue; // no stereo depth
                if (mCurrentFrame.mvpMapPoints[i])     continue; // already tracked
                NewMapPointCandidate c;
                c.keypoint_idx  = i;
                c.left_kp       = mCurrentFrame.mvKeysUn[i];
                c.depth         = mCurrentFrame.mvDepth[i];
                c.right_kp      = mCurrentFrame.mvKeysUn[i];
                c.right_kp.pt.x = mCurrentFrame.mvuRight[i]; // reconstruct right kp
                Eigen::Vector3f x3D;
                if (mCurrentFrame.UnprojectStereo(i, x3D))
                {
                    c.pos_world  = x3D;
                    c.pos_camera = Tcw * x3D;
                }
                tracking_result.new_map_point_candidates.push_back(c);
            }
        }

        // Reset if the camera get lost
        if (mState == LOST)
        {
            mpSystem->ResetActiveMap();
            return tracking_result;
        }
        ...
```

---

## Part 3 — Known Limitation: LocalMapping is Asynchronous

`LocalMapping::CreateNewMapPoints()` runs in a **separate thread** and triangulates new
map points **after** `TrackingResult` has been returned to the caller. It is therefore
not possible to synchronously populate newly triangulated map points in `TrackingResult`.

**What is captured instead:**
- `new_map_point_candidates` — the exact set of stereo keypoints that LocalMapping will
  promote to MapPoints, with their stereo-derived 3D positions.
- `all_tracked_map_points` — all map points that were **already in the map** and observed
  in this frame, including any points created in previous frames that are now tracked.

**Future extension** (out of scope): Add a thread-safe queue in `LocalMapping` where
`CreateNewMapPoints()` posts `(frame_id, keypoint_idx, MapPoint*)` tuples after
triangulation. A caller can query by frame ID after the fact.

---

## Part 4 — Coordinate Conventions

| Term | Meaning |
|------|---------|
| `Tcw` | `mCurrentFrame.GetPose()` — transforms a point **from world frame to camera frame** |
| `pos_camera = Tcw * pos_world` | Correct: world-frame point → camera-frame point |
| `UnprojectStereo(i, x3D)` | Returns 3D point in **world frame** (not camera frame). Internally it computes camera-frame coordinates then applies `Twc`. |
| `result.pose = GetPose().inverse()` | Stores `Twc`, i.e. the camera pose **in world frame** (position + orientation of camera). This is the convention already used by existing result struct `pose` fields. |

---

## Part 5 — Images in TrackingResult

### 5a. Why images are not stored in Frame or Tracking

ORB-SLAM3 does **not** retain images after feature extraction. The `Frame` class stores
only feature data (keypoints, descriptors, depths). Images flow through as local variables
in `System.cc` → `Tracking::GrabImageStereo/GrabImageMonocular()` and are discarded.

The images passed into `GrabImageStereo` and `GrabImageMonocular` are already the
processed version the consumer cares about:
- **Stereo pinhole**: rectified via `cv::remap()` in `System.cc` using `M1l/M2l/M1r/M2r`
  maps from settings, then resized to `mImageScale` if needed.
- **Monocular**: undistorted via `cv::undistort()` if `settings->needToUndistort()`, or
  the raw image if the camera model handles undistortion intrinsically.

Both are always single-channel (grayscale). The Tracking layer asserts this on entry.

### 5b. New fields in `TrackingResult` (`include/Tracking.h`)

Add before the closing `};` of `TrackingResult`, alongside the other new fields:

```cpp
    // (5) Input images as received by the tracking layer — already grayscale, rectified
    //     (stereo pinhole) or undistorted (monocular), and rescaled to mImageScale.
    //     Together with the keypoint and map-point data above, these make TrackingResult
    //     fully self-contained for offline visualisation.
    //
    //     image_left  — left image (stereo) or the single image (monocular). Never empty.
    //     image_right — right image (stereo only). Empty (default-constructed cv::Mat)
    //                   for monocular.
    //
    //     Both images are cloned (own their data); the originals are temporaries and go
    //     out of scope after GrabImage* returns.
    cv::Mat image_left;
    cv::Mat image_right;
```

`cv::Mat` is already included via `<opencv2/core/core.hpp>` which is present in
`Tracking.h`. No new `#include` is needed for this field.

### 5c. Populate in `GrabImageStereo()` (`src/Tracking.cc`, around line 253)

Current code (lines 253–258):
```cpp
    TrackingResult result = Track();
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    return result;
```

Replace with:
```cpp
    TrackingResult result = Track();

    // Attach the rectified images so the result is self-contained.
    // .clone() is mandatory: imageLeft/imageRight are const refs to temporaries
    // in System.cc that go out of scope immediately after this function returns.
    result.image_left  = imageLeft.clone();
    result.image_right = imageRight.clone();

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    return result;
```

### 5d. Populate in `GrabImageMonocular()` (`src/Tracking.cc`, around line 313)

Current code (lines 313–318):
```cpp
    TrackingResult result = Track();
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    return result;
```

Replace with:
```cpp
    TrackingResult result = Track();

    // Attach the (undistorted) image so the result is self-contained.
    // image_right is left default-constructed (empty) for monocular.
    result.image_left = image.clone();

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    return result;
```

### 5e. Early-return paths (`Track()` returns `{}`)

`Track()` has five early-return paths that return a default-constructed `TrackingResult`
(lines 772, 788, 807, 855, 913). These are exceptional cases (bad IMU, map reset,
timestamp jump, LOST state). The caller is `GrabImageStereo` or `GrabImageMonocular`,
which will still attach the images **after** `Track()` returns, so all early-return paths
are automatically covered — no special handling needed.

---

## Part 6 — Verification Checklist

After implementing, verify the following (can be done with a simple print loop in any
stereo or mono example):

| Check | Expected result |
|-------|----------------|
| `keypoint_data.left_keypoints.size()` | Equals `mCurrentFrame.N` |
| `keypoint_data.right_keypoints.size()` | Equals `mCurrentFrame.N` (stereo) or 0 (mono) |
| `keypoint_data.stereo_right_u.size()` | Equals `mCurrentFrame.N` |
| `keypoint_data.stereo_match_pairs` non-empty (stereo) | Yes; each `right_kp.pt.x >= 0` |
| `keypoint_data.stereo_right_u` all -1 (mono) | Yes |
| `motion_model_result.frame_matches` non-empty (frame ≥ 2) | Yes |
| `frame_matches_optimized.size() <= frame_matches.size()` | Yes (optimizer removes outliers) |
| `ref_key_frame_result.kf_matches` non-empty | Yes when used as primary or fallback |
| `local_map_result.inlier_observations` non-empty on success | Yes |
| `local_map_result.inlier_observations[i].pos_camera.z() > 0` | Yes (forward-facing) |
| `all_tracked_map_points.size()` ≈ `mnMatchesInliers` | Yes |
| `new_map_point_candidates` non-empty (stereo, near keyframe) | Yes |
| `image_left.empty()` | Always false — populated before every return |
| `image_right.empty()` | False for stereo, true for monocular |
| `image_left.channels()` | Always 1 (grayscale) |
| `image_left.size()` matches the resolution used for feature extraction | Yes |
| Build produces no new compiler warnings | Yes |
| Legacy fields (`num_matches`, `keypoints_inliers_optimized`, etc.) unchanged in size/values | Yes |
