# Design: Local Mapping Thread Callbacks

## 1. Motivation

The tracking thread already exposes a rich `TrackingResult` struct returned
synchronously to the caller of `System::TrackStereo` / `TrackMonocular`.
External clients can inspect every sub-step of the tracking pipeline without
reaching into library internals.

The local mapping thread operates **asynchronously** on its own OS thread, so
the same return-value pattern cannot be applied.  Instead, this design
introduces:

1. A **`LocalMappingResult`** struct — a fully self-contained snapshot of one
   `RunLoop()` iteration, mirroring the structure of `TrackingResult`.
2. A **callback registration** interface on `System` that lets the client
   register a `std::function` which is invoked from the mapping thread at the
   end of every iteration.

The callback-based design matches the asynchronous nature of the thread.  The
client is responsible for any cross-thread synchronisation it needs (e.g. a
lock-free queue to move data to its own thread).

---

## 2. Data Design

### 2.1 Map-point transfer philosophy

* **Newly created map points** — sent as a small value struct
  (`NewMappingMapPoint`) containing the world-space position and the
  map-point ID.  The client accumulates these to maintain its own map.
* **Removed / culled map points** — sent as a `vector<unsigned long>` of IDs
  only.  The client drops those IDs from its own map.  This covers both points
  that are marked bad (SetBadFlag) and keyframes that are culled.

This keeps the result struct free of raw pointers into library-internal
structures and avoids copying heavy data (descriptors, observation maps, etc.)
that most clients do not need.

### 2.2 Struct hierarchy

```
LocalMappingResult
├── context                    (keyframe identity, queue depth, iteration counter)
├── ProcessNewKeyFrameResult
├── MapPointCullingResult
├── CreateNewMapPointsResult
├── SearchInNeighborsResult
├── LocalBundleAdjustmentResult
├── KeyFrameCullingResult
└── timing                     (per-stage and total wall-clock durations in ms)
```

---

## 3. Proposed Header (`include/LocalMappingResult.h`)

```cpp
#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ORB_SLAM3
{

// ---------------------------------------------------------------------------
// Shared value types
// ---------------------------------------------------------------------------

/// Lightweight representation of a map point that was created or survived
/// culling in this iteration.  The client accumulates these to maintain its
/// own sparse map.
struct NewMappingMapPoint
{
    unsigned long id = 0;           ///< MapPoint::mnId — unique in the map
    Eigen::Vector3f pos_world;      ///< World-frame 3-D position
    unsigned long first_kf_id = 0;  ///< mnId of the KeyFrame that created it
};

// ---------------------------------------------------------------------------
// Per-stage result structs
// ---------------------------------------------------------------------------

/// Results from ProcessNewKeyFrame().
///
/// This stage pops the next KeyFrame from the queue, computes its BoW
/// descriptor, associates existing MapPoints, and inserts the frame into the
/// Atlas.
struct ProcessNewKeyFrameResult
{
    /// KeyFrame::mnId of the frame that was processed.
    unsigned long keyframe_id = 0;

    /// Frame::mnFrameId — the sequential tracking frame index.
    unsigned long frame_id = 0;

    /// Sensor timestamp of the processed KeyFrame (seconds).
    double timestamp = 0.0;

    /// World-to-camera pose of the processed KeyFrame at the time of
    /// insertion (before any LBA refinement this iteration).
    Sophus::SE3f pose;

    /// Number of map point slots in the KeyFrame's observation vector
    /// (mpCurrentKeyFrame->GetMapPointMatches().size()).  This is the total
    /// number of feature keypoints that *could* be associated with a map point.
    int num_kf_map_point_slots = 0;

    /// Number of those slots that hold a valid, non-bad MapPoint after the
    /// association loop in ProcessNewKeyFrame().
    int num_associated_map_points = 0;

    /// Number of stereo MapPoints (already in mlpRecentAddedMapPoints from the
    /// tracking thread) that were registered in this call.  These are points
    /// that were created by Tracking's stereo initialisation and promoted to
    /// the "recent added" list for later culling.
    int num_stereo_map_points_registered = 0;

    /// Remaining depth of the new-keyframe queue *after* this frame was popped.
    int queue_size_after = 0;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

/// Results from MapPointCulling().
///
/// This stage walks mlpRecentAddedMapPoints and removes unreliable points.
/// Points may be silently removed (already bad) or explicitly marked bad
/// (SetBadFlag) if they have too few observations or a low found-ratio.
/// Points that have simply aged out of the "recent" window are dropped from
/// the list without being marked bad.
struct MapPointCullingResult
{
    /// Total size of mlpRecentAddedMapPoints *before* culling.
    int num_recent_map_points_before = 0;

    /// Points already flagged bad by another thread — removed from list only.
    int num_culled_already_bad = 0;

    /// Points culled because GetFoundRatio() < threshold — SetBadFlag() called.
    int num_culled_low_found_ratio = 0;

    /// Points culled because they have too few observations after the
    /// minimum-KF-age window has elapsed — SetBadFlag() called.
    int num_culled_too_few_observations = 0;

    /// Points removed from the "recent" list because they have aged beyond
    /// mMPCullingMaxKFAgeInRecent without another culling trigger.
    /// SetBadFlag() is NOT called for these; they graduate to the main map.
    int num_graduated = 0;

    /// Remaining size of mlpRecentAddedMapPoints *after* culling.
    int num_recent_map_points_after = 0;

    /// IDs of map points on which SetBadFlag() was called during this stage.
    /// The client should remove these from its own map.
    /// (Points that were already bad or merely graduated are excluded.)
    std::vector<unsigned long> culled_map_point_ids;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

/// Results from CreateNewMapPoints().
///
/// This stage searches for feature matches across neighbouring KeyFrames
/// using the epipolar constraint, triangulates valid pairs, and inserts new
/// MapPoints into the Atlas.
struct CreateNewMapPointsResult
{
    /// Number of neighbour KeyFrames considered (from GetBestCovisibilityKeyFrames).
    int num_neighbour_kfs = 0;

    /// Total epipolar matches found across all neighbour pairs before
    /// triangulation checks.
    int num_epipolar_matches = 0;

    /// Number of stereo-unproject attempts (bPointStereo path).
    int num_stereo_unproject_attempts = 0;

    /// Number of successfully created MapPoints this stage.
    int num_created = 0;

    /// Subset of num_created that came from stereo unprojection rather than
    /// multi-view triangulation.
    int num_created_from_stereo = 0;

    /// Whether the loop was aborted early because a new KeyFrame arrived
    /// (CheckNewKeyFrames() returned true mid-iteration).
    bool aborted_early = false;

    /// The newly created MapPoints.  The client should merge these into its
    /// own sparse map.
    std::vector<NewMappingMapPoint> new_map_points;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

/// Results from SearchInNeighbors().
///
/// This stage fuses duplicate MapPoints between the current KeyFrame and its
/// neighbours (bidirectional projection search).
struct SearchInNeighborsResult
{
    /// Number of first-level neighbour KeyFrames added as fusion targets.
    int num_first_level_neighbours = 0;

    /// Number of second-level (covisible-of-covisible) KeyFrames also added.
    int num_second_level_neighbours = 0;

    /// Total fusion target KeyFrames (first + second level, deduplicated).
    int num_target_kfs = 0;

    /// Whether the stage exited early because mbAbortBA was set (a new
    /// KeyFrame arrived).
    bool aborted_early = false;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

/// Results from LocalBundleAdjustment (LBA).
///
/// LBA is throttled by mOptimizeEveryTSeconds so it may be skipped entirely.
/// When it runs it jointly optimises the current KeyFrame and its local
/// neighbourhood.
struct LocalBundleAdjustmentResult
{
    /// True if LBA was skipped this iteration (throttle or too few KFs).
    bool skipped = false;

    /// Reason LBA was skipped (empty string when skipped == false).
    /// Possible values: "throttled", "too_few_keyframes", "stop_requested",
    ///                  "new_kf_arrived".
    std::string skip_reason;

    /// Number of KeyFrames held fixed during LBA (anchors / fixed nodes).
    int num_fixed_kfs = 0;

    /// Number of KeyFrames whose poses were optimised.
    int num_optimised_kfs = 0;

    /// Number of MapPoints included in the optimisation.
    int num_map_points = 0;

    /// Number of reprojection edges in the g2o graph.
    int num_edges = 0;

    /// Wall-clock duration of this stage (milliseconds).  Zero when skipped.
    double duration_ms = 0.0;
};

/// Results from KeyFrameCulling().
///
/// A KeyFrame is redundant — and set bad — if >= mKeyFrameCullingRedundantRatio
/// of its close MapPoints are observed by at least mKeyFrameCullingMinObsInOthers
/// other KeyFrames at an equal or finer scale.
struct KeyFrameCullingResult
{
    /// Number of local KeyFrames examined (may be capped by
    /// mKeyFrameCullingMaxKeyframesToCheck or an early-exit condition).
    int num_kfs_checked = 0;

    /// Number of KeyFrames marked bad (SetBadFlag) in this call.
    int num_kfs_culled = 0;

    /// Whether the loop exited early due to mbAbortBA or the max-check cap.
    bool aborted_early = false;

    /// mnId values of KeyFrames that were marked bad.
    /// The client should treat observations from these frames as no longer
    /// authoritative (though the MapPoints themselves may still be valid).
    std::vector<unsigned long> culled_keyframe_ids;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

// ---------------------------------------------------------------------------
// Top-level result
// ---------------------------------------------------------------------------

/// Complete result of one RunLoop() iteration of the LocalMapping thread.
///
/// Produced at the end of every iteration in which a KeyFrame was actually
/// processed (i.e. CheckNewKeyFrames() returned true at entry).  The callback
/// is NOT invoked on idle iterations where the thread finds an empty queue and
/// goes back to sleep.
///
/// The struct is fully self-contained: no raw pointers into library internals.
/// All map-point identity is communicated via numeric IDs so the client can
/// maintain its own lightweight map without holding library object references.
struct LocalMappingResult
{
    // -----------------------------------------------------------------------
    // Context
    // -----------------------------------------------------------------------

    /// Monotonically increasing counter, starting at 1, incremented every time
    /// a KeyFrame is processed (regardless of which sub-stages run).
    uint64_t iteration = 0;

    /// KeyFrame::mnId of the frame processed this iteration.
    unsigned long keyframe_id = 0;

    /// Frame::mnFrameId of the frame processed this iteration.
    unsigned long frame_id = 0;

    /// Sensor timestamp of the processed KeyFrame (seconds).
    double timestamp = 0.0;

    // -----------------------------------------------------------------------
    // Per-stage results
    // -----------------------------------------------------------------------

    ProcessNewKeyFrameResult process_new_keyframe;
    MapPointCullingResult map_point_culling;
    CreateNewMapPointsResult create_new_map_points;

    /// SearchInNeighbors is skipped when a new KeyFrame is waiting in the
    /// queue (CheckNewKeyFrames() == true after CreateNewMapPoints).
    /// Check search_in_neighbors.aborted_early or inspect the skip flag below.
    bool search_in_neighbors_skipped = false;
    SearchInNeighborsResult search_in_neighbors;

    LocalBundleAdjustmentResult lba;
    KeyFrameCullingResult keyframe_culling;

    // -----------------------------------------------------------------------
    // Convenience map-point delta summary
    // -----------------------------------------------------------------------

    /// Union of all newly created map points this iteration
    /// (from CreateNewMapPoints; duplicates CreateNewMapPointsResult::new_map_points
    /// at the top level for ergonomic access).
    std::vector<NewMappingMapPoint> added_map_points;

    /// Union of all map-point IDs removed this iteration.
    /// Combines culled IDs from MapPointCulling and (if applicable) any points
    /// invalidated by KeyFrameCulling.  The client should drop these IDs from
    /// its own map.
    std::vector<unsigned long> removed_map_point_ids;

    // -----------------------------------------------------------------------
    // Timing summary
    // -----------------------------------------------------------------------

    /// Total wall-clock duration of the RunLoop() body for this iteration
    /// (milliseconds).  Equals the sum of all stage duration_ms fields plus
    /// negligible bookkeeping overhead.
    double total_duration_ms = 0.0;
};

// ---------------------------------------------------------------------------
// Callback type
// ---------------------------------------------------------------------------

/// Signature of the callback that the client registers via
/// System::SetLocalMappingCallback().
///
/// The callback is invoked **from the LocalMapping thread**.  Implementations
/// must be thread-safe and should return quickly.  If heavy work is needed,
/// copy the result and hand it off to a separate thread.
using LocalMappingCallback = std::function<void(const LocalMappingResult&)>;

}  // namespace ORB_SLAM3
```

---

## 4. Changes to `LocalMapping`

### 4.1 New members (`LocalMapping.h`)

```cpp
#include "LocalMappingResult.h"
#include <functional>
#include <atomic>

// Inside class LocalMapping:
public:
    /// Register a callback to be invoked at the end of every RunLoop()
    /// iteration that processes a KeyFrame.  Pass nullptr to clear.
    /// Thread-safe: may be called from any thread.
    void SetCallback(LocalMappingCallback cb);

private:
    LocalMappingCallback mCallback;       // guarded by mMutexCallback
    std::mutex           mMutexCallback;
    std::atomic<uint64_t> mIterationCounter{0};
```

### 4.2 `SetCallback` implementation (`LocalMapping.cc`)

```cpp
void LocalMapping::SetCallback(LocalMappingCallback cb)
{
    std::unique_lock<std::mutex> lock(mMutexCallback);
    mCallback = std::move(cb);
}
```

### 4.3 Instrumented `RunLoop()` (`LocalMapping.cc`)

The existing `RunLoop()` is modified to:

1. Time each sub-stage with `std::chrono::steady_clock`.
2. Populate the appropriate result sub-struct inside each `Process*` /
   `MapPoint*` / `Create*` / `Search*` / `KeyFrame*` method (or inline for
   LBA).
3. Assemble the `LocalMappingResult` at the end of the iteration.
4. Invoke the callback under lock (to guard against concurrent `SetCallback`).

High-level sketch:

```cpp
bool LocalMapping::RunLoop()
{
    SetAcceptKeyFrames(false);

    if (CheckNewKeyFrames())
    {
        LocalMappingResult result;
        result.iteration = ++mIterationCounter;
        const auto loop_start = std::chrono::steady_clock::now();

        // --- ProcessNewKeyFrame ---
        {
            const auto t0 = std::chrono::steady_clock::now();
            result.process_new_keyframe = ProcessNewKeyFrame();   // returns sub-result
            result.process_new_keyframe.duration_ms = elapsed_ms(t0);
        }

        // Fill top-level context from the sub-result (avoids duplication).
        result.keyframe_id = result.process_new_keyframe.keyframe_id;
        result.frame_id    = result.process_new_keyframe.frame_id;
        result.timestamp   = result.process_new_keyframe.timestamp;

        // --- MapPointCulling ---
        {
            const auto t0 = std::chrono::steady_clock::now();
            result.map_point_culling = MapPointCulling();
            result.map_point_culling.duration_ms = elapsed_ms(t0);
        }

        // --- CreateNewMapPoints ---
        {
            const auto t0 = std::chrono::steady_clock::now();
            result.create_new_map_points = CreateNewMapPoints();
            result.create_new_map_points.duration_ms = elapsed_ms(t0);
        }

        mbAbortBA = false;

        // --- SearchInNeighbors (conditional) ---
        if (!CheckNewKeyFrames())
        {
            const auto t0 = std::chrono::steady_clock::now();
            result.search_in_neighbors = SearchInNeighbors();
            result.search_in_neighbors.duration_ms = elapsed_ms(t0);
        }
        else
        {
            result.search_in_neighbors_skipped = true;
        }

        // --- LBA ---
        {
            const auto t0 = std::chrono::steady_clock::now();
            result.lba = MaybeRunLBA();   // encapsulates throttle logic + KeyFrameCulling
            result.lba.duration_ms = elapsed_ms(t0);
        }

        // --- Assemble convenience deltas ---
        result.added_map_points    = result.create_new_map_points.new_map_points;
        result.removed_map_point_ids = result.map_point_culling.culled_map_point_ids;
        // (KeyFrameCulling does not directly invalidate MapPoints, but callers
        //  should be aware those KF observations are gone.)

        result.total_duration_ms = elapsed_ms(loop_start);

        // --- Fire callback ---
        {
            std::unique_lock<std::mutex> lock(mMutexCallback);
            if (mCallback)
                mCallback(result);
        }
    }
    // ... rest of RunLoop (Stop/Reset/Finish handling) unchanged
}
```

> **Note:** The existing sub-stage methods currently return `void`.  They will
> be refactored to return their respective result structs.  The internal logic
> of each method is **not** changed; only the final counters and collected IDs
> are written into the struct before returning.

---

## 5. Changes to `System`

### 5.1 New public method (`System.h`)

```cpp
#include "LocalMappingResult.h"

// Inside class System:
public:
    /// Register a callback to be invoked from the LocalMapping thread at the
    /// end of every iteration that processes a KeyFrame.
    /// Pass nullptr to clear an existing callback.
    void SetLocalMappingCallback(LocalMappingCallback cb);
```

### 5.2 Implementation (`System.cc`)

```cpp
void System::SetLocalMappingCallback(LocalMappingCallback cb)
{
    mpLocalMapper->SetCallback(std::move(cb));
}
```

Delegation is intentionally thin — `System` is the public API boundary and
`LocalMapping` owns the callback storage and mutex.

---

## 6. Thread-safety model

| Concern | Mitigation |
|---------|-----------|
| Callback registration from user thread while mapping thread reads it | `mMutexCallback` held both in `SetCallback` and when invoking the callback. |
| Callback runs on the mapping thread; client accesses its own data | Client responsibility.  The design document recommends a lock-free queue or a mutex-protected buffer in the client. |
| `LocalMappingResult` contains only value types and `std::vector` | No shared ownership; the result is safe to move into a client-side queue. |
| Callback blocks the mapping thread | Client should return quickly.  Document states this explicitly. |

---

## 7. Timing instrumentation

A small inline helper avoids repetition:

```cpp
// In LocalMapping.cc (file-local)
static double elapsed_ms(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
               .count();
}
```

Each sub-stage is wrapped:

```cpp
const auto t0 = std::chrono::steady_clock::now();
result.some_stage = SomeStage();
result.some_stage.duration_ms = elapsed_ms(t0);
```

`total_duration_ms` spans from the moment `CheckNewKeyFrames()` returns true
to the moment the callback is about to be fired (i.e. it includes all
sub-stages but not the callback execution itself).

---

## 8. Refactoring scope

| File | Change |
|------|--------|
| `include/LocalMappingResult.h` | **New file** — all result structs and callback typedef. |
| `include/LocalMapping.h` | Add `#include "LocalMappingResult.h"`, `SetCallback()`, `mCallback`, `mMutexCallback`, `mIterationCounter`. Change private method signatures to return sub-result structs. |
| `src/LocalMapping.cc` | Instrument `RunLoop()` with timing and result assembly. Refactor `ProcessNewKeyFrame`, `MapPointCulling`, `CreateNewMapPoints`, `SearchInNeighbors`, `KeyFrameCulling` to return their respective structs. Extract LBA block into `MaybeRunLBA()` returning `LocalBundleAdjustmentResult`. |
| `include/System.h` | Add `#include "LocalMappingResult.h"`, `SetLocalMappingCallback()`. |
| `src/System.cc` | Implement `SetLocalMappingCallback()`. |

---

## 9. Example client usage

```cpp
ORB_SLAM3::System slam(vocFile, configFile, ORB_SLAM3::System::STEREO, calib);

slam.SetLocalMappingCallback([](const ORB_SLAM3::LocalMappingResult& r) {
    // Called from the LocalMapping thread — copy and return quickly.
    printf("[KF %lu | frame %lu | t=%.3f s]\n",
           r.keyframe_id, r.frame_id, r.timestamp);
    printf("  Loop:        %.1f ms\n", r.total_duration_ms);
    printf("  ProcessKF:   %.1f ms  (%d MPs associated)\n",
           r.process_new_keyframe.duration_ms,
           r.process_new_keyframe.num_associated_map_points);
    printf("  MPCulling:   %.1f ms  (%d culled, %d graduated)\n",
           r.map_point_culling.duration_ms,
           (int)r.map_point_culling.culled_map_point_ids.size(),
           r.map_point_culling.num_graduated);
    printf("  CreateMPs:   %.1f ms  (%d created from %d epipolar matches)\n",
           r.create_new_map_points.duration_ms,
           r.create_new_map_points.num_created,
           r.create_new_map_points.num_epipolar_matches);
    if (!r.search_in_neighbors_skipped)
        printf("  SearchNeigh: %.1f ms  (%d target KFs)\n",
               r.search_in_neighbors.duration_ms,
               r.search_in_neighbors.num_target_kfs);
    if (!r.lba.skipped)
        printf("  LBA:         %.1f ms  (%d opt KFs, %d MPs, %d edges)\n",
               r.lba.duration_ms,
               r.lba.num_optimised_kfs,
               r.lba.num_map_points,
               r.lba.num_edges);
    else
        printf("  LBA:         skipped (%s)\n", r.lba.skip_reason.c_str());
    printf("  KFCulling:   %.1f ms  (%d culled)\n",
           r.keyframe_culling.duration_ms,
           r.keyframe_culling.num_kfs_culled);
    printf("  Map delta:   +%d MPs  -%d MPs\n",
           (int)r.added_map_points.size(),
           (int)r.removed_map_point_ids.size());
});

while (hasFrames())
{
    auto result = slam.TrackStereo(left, right, ts);
    // ...
}
```

---

## 10. Open questions / future work

* **LBA-refined poses** — after LBA, all local KeyFrame poses change.  A
  future extension could include a `vector<{kf_id, new_pose}>` in
  `LocalBundleAdjustmentResult` so the client can update its own trajectory
  estimate without polling.
* **Map-point position updates after LBA** — similarly, LBA refines 3-D
  positions.  A `vector<{mp_id, new_pos_world}>` in `LocalBundleAdjustmentResult`
  would let clients keep positions in sync without re-querying the map.
* **Fused map-point ID remapping** — `SearchInNeighbors` can merge duplicate
  MapPoints (one survives, one is marked bad).  Exposing the
  `{old_id → surviving_id}` mapping would let clients consolidate their own
  representations.  This is deferred because it requires instrumentation inside
  `FeatureMatcher::Fuse`.
