# Track() Control Flow

This documents the main `Tracking::Track()` function in `Tracking.cc`.

`Track()` is the top-level per-frame entry point. It handles initialization, pose estimation, local map tracking, state transitions, and keyframe insertion.

`Track()` delegates most of its work to a few small helpers:

- `PrepareFrameForTracking()` – state/bias setup and IMU preintegration
- `UpdateMapChangeState()` – map-change detection under the map mutex
- `InitializeIfNeeded()` – stereo/mono initialization and first-frame bookkeeping
- `UpdateAfterTracking(bool bOK)` – motion model, cleanup, and keyframe decision

## High-Level Flow

```text
Track()
  |
  v
[1] Sanity checks & timestamp validation
  |
  v
[2] IMU preintegration (if IMU sensor)
  |
  v
[3] Lock map mutex
  |
  v
[4] NOT_INITIALIZED? --> Initialization
  |                        |
  |                        +-- failed --> save frame, return
  |                        +-- success --> mState = OK, continue
  |
  v (already initialized)
[5] Pose estimation (state == OK or LOST)
  |
  v
[6] Track local map (if pose estimation succeeded)
  |
  v
[7] State transition (OK / LOST)
  |
  v
[8] Post-tracking (motion model, cleanup, keyframe decision)
  |
  v
[9] Lost handling / trajectory storage
```

## Detailed Breakdown

### [1] Sanity Checks

```text
Bad IMU flag set?
  +-- yes --> ResetActiveMap(), return
  |
  v no
State != NO_IMAGES_YET?
  |
  +-- yes --> Check timestamps:
  |             |
  |             +-- currentTime < lastTime (backward jump)
  |             |     --> clear IMU queue, CreateMapInAtlas(), return
  |             |
  |             +-- currentTime > lastTime + 1.0s (forward jump)
  |                   +-- inertial sensor?
  |                   |     +-- IMU initialized & BA2 done --> CreateMapInAtlas(), return
  |                   |     +-- IMU initialized & no BA2  --> ResetActiveMap(), return
  |                   |     +-- IMU not initialized       --> ResetActiveMap(), return
  |                   +-- visual only --> (no action, continue)
  |
  v
IMU sensor & mpLastKeyFrame exists?
  +-- yes --> set current frame bias from last KF
  |
  v
State == NO_IMAGES_YET?
  +-- yes --> mState = NOT_INITIALIZED
```

### [2] IMU Preintegration

Handled inside `PrepareFrameForTracking()`. Only for `IMU_MONOCULAR` or `IMU_STEREO`, and only if we didn't just create a new map:

```text
IMU sensor AND !mbCreatedMap?
  +-- yes --> PreintegrateIMU()
```

### [3] Map Lock & Change Detection

Acquires `mMutexMapUpdate`, then calls `UpdateMapChangeState(pCurrentMap)`. This checks if the map was modified by another thread (e.g. local mapper, loop closer) since last frame and sets `mbMapUpdated`.

### [4] Initialization

Initialization is now encapsulated in `InitializeIfNeeded()`, but the behavior is unchanged:

```text
State == NOT_INITIALIZED?
  |
  +-- STEREO / IMU_STEREO --> StereoInitialization()
  +-- MONO / IMU_MONO     --> MonocularInitialization()
  |
  v
  mState == OK? (initialization succeeded?)
    +-- no  --> save frame, return
    +-- yes --> record first frame ID (if first map), continue
```

After successful initialization the function does NOT enter the tracking pipeline below -- it skips directly to trajectory storage at the end.

### [5] Pose Estimation

This is the core tracking step. It runs when the system is already initialized.

```text
State == OK?
  |
  +-- yes:
  |     CheckReplacedInLastFrame()
  |     |
  |     Has velocity AND (IMU initialized or visual)?
  |       |
  |       +-- no velocity --> TrackReferenceKeyFrame()
  |       |
  |       +-- has velocity --> TrackWithMotionModel()
  |                              |
  |                              +-- failed --> fallback to TrackReferenceKeyFrame()
  |     |
  |     Both failed?
  |       +-- yes --> mState = LOST
  |
  +-- State == LOST:
        KFs in map <= LostResetMinKFs?
          +-- yes --> ResetActiveMap()
          +-- no  --> CreateMapInAtlas()
        Clear mpLastKeyFrame, return
```

The pose estimation strategy is:

1. **Motion model** (preferred when velocity is available): projects map points using predicted pose.
2. **Reference keyframe** (fallback): matches against the reference keyframe via BoW.
3. If both fail, the state transitions to `LOST`.

### [6] Track Local Map

Only runs if pose estimation succeeded (`bOK == true`):

```text
bOK?
  +-- yes --> TrackLocalMap()
  |             |
  |             +-- success --> log inliers count
  |             +-- failure --> log failure
  +-- no  --> skip
```

`TrackLocalMap()` refines the pose by matching against the local map (nearby keyframes and their map points). This is where `mnMatchesInliers` is set.

### [7] State Transition

```text
bOK (after local map)?
  +-- yes --> mState = OK
  +-- no  --> was OK before?
                +-- yes --> mState = LOST, record mTimeStampLost
```

### [8] Post-Tracking: Success Path

Only entered when `bOK == true`. The logic below now lives in `UpdateAfterTracking(bOK)`, but the behavior is the same:

```text
[8a] Update map drawer with current pose

[8b] Update motion model
       last & current frame both set?
         +-- yes --> mVelocity = T_current * T_last^-1, mbVelocity = true
         +-- no  --> mbVelocity = false

[8c] Clean VO matches
       Remove map points with Observations < 1

[8d] Delete temporal map points

[8e] Keyframe decision
       NeedNewKeyFrame()?
         +-- yes AND (bOK OR (insertKFsLost AND IMU sensor))
         |     --> CreateNewKeyFrame()
         +-- no --> skip

[8f] Discard outliers
       Remove map points flagged as outliers by optimization
```

### [9] Lost Handling & Trajectory Storage

```text
State == LOST?
  |
  +-- KFs in map <= LostResetMinKFs --> ResetActiveMap(), return
  +-- IMU sensor & not initialized --> ResetActiveMap(), return
  +-- mbAtlasNewMaps --> CreateMapInAtlas()
  +-- return (don't store trajectory)

State == OK?
  |
  +-- Store relative pose, reference KF, timestamp, lost flag
```

## State Machine

The tracking states and their transitions:

```text
  NO_IMAGES_YET
       |
       v (first frame arrives)
  NOT_INITIALIZED
       |
       +-- init success --> OK
       |
       v (stays NOT_INITIALIZED until success)

  OK <--+
   |    |
   |    +-- TrackLocalMap succeeds
   |
   +-- pose estimation or local map fails
   |
   v
  LOST
   |
   +-- few KFs (<= LostResetMinKFs) --> ResetActiveMap --> NOT_INITIALIZED
   +-- many KFs                     --> CreateMapInAtlas --> NOT_INITIALIZED
```

## Configurable Parameters

| YAML Key | Type | Default | Description |
| --- | --- | --- | --- |
| Tracking.LostResetMinKFs | int | 999999 | Min KFs in map before LOST triggers a map reset. High value = never reset, always reuse map. |

## Key Functions Called

| Function | Purpose | Called When |
| --- | --- | --- |
| StereoInitialization() | Initialize map from stereo pair | NOT_INITIALIZED, stereo sensor |
| MonocularInitialization() | Initialize map from two monocular frames | NOT_INITIALIZED, mono sensor |
| CheckReplacedInLastFrame() | Update map points replaced by local mapper | State OK, before pose estimation |
| TrackWithMotionModel() | Estimate pose using constant-velocity model | State OK, velocity available |
| TrackReferenceKeyFrame() | Estimate pose via BoW matching to ref KF | State OK, no velocity or motion model failed |
| TrackLocalMap() | Refine pose against local map points | Pose estimation succeeded |
| NeedNewKeyFrame() | Decide whether to insert a keyframe | Tracking succeeded (see KeyframeDecision.md) |
| CreateNewKeyFrame() | Build and insert a new keyframe | NeedNewKeyFrame() returned true |
| PreintegrateIMU() | Preintegrate IMU measurements | IMU sensor, before map lock |
| CreateMapInAtlas() | Abandon current map, start fresh | Timestamp jump, or LOST with enough KFs |

## TrackReferenceKeyFrame()

**Purpose**: Estimate camera pose by matching the current frame against the reference keyframe using BoW, then optimizing.

**Control flow**:

```text
TrackReferenceKeyFrame()
  |
  v
[1] Compute BoW for current frame
    mCurrentFrame.ComputeBoW()

[2] Match against reference keyframe
    ORBmatcher(mReferenceKeyframeNNRatio)
    nmatches = SearchByBoW(refKF, currentFrame)

[3] Check BoW match count
    nmatches < mReferenceKeyframeMinBoWMatches ?
      yes -> log failure, return false
      no  -> continue

[4] Initialize pose and associations
    mCurrentFrame.mvpMapPoints = vpMapPointMatches
    mCurrentFrame.SetPose(mLastFrame.GetPose())

[5] Pose optimization
    Optimizer::PoseOptimization(&mCurrentFrame)

[6] Outlier removal & map-point stats
    For each matched point:
      if outlier:
        - clear association in frame
        - clear trackInView flag (left/right)
        - decrement nmatches
      else if Observations() > 0:
        - increment nmatchesMap

[7] Decision
    IMU sensor?
      yes -> return true
      no:
        nmatchesMap >= mReferenceKeyframeMinOptimizedMapMatches ?
          yes -> log success, return true
          no  -> log failure, return false
```

## TrackWithMotionModel()

**Purpose**: Use a constant-velocity motion model to predict pose, project previous-frame points, and refine via optimization.

**Control flow**:

```text
TrackWithMotionModel()
  |
  v
[1] Prepare matcher
    ORBmatcher(mMotionModelNNRatio, true)

[2] Update last frame pose & VO points
    UpdateLastFrame()

[3] IMU-initialized fast path?
    if mpAtlas->isImuInitialized():
      PredictStateIMU()
      return true
    else:
      mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose())

[4] Clear current frame map-point associations
    fill(mCurrentFrame.mvpMapPoints, NULL)

[5] First projection search
    th = (sensor == STEREO) ? mMotionModelProjectionSearchThStereo
                            : mMotionModelProjectionSearchThMono
    nmatches = SearchByProjection(current, last, th, isMonoLike)

[6] Retry with wider window if few matches
    thRetry = (sensor == STEREO) ? mMotionModelRetryProjectionSearchThStereo
                                 : mMotionModelRetryProjectionSearchThMono
    if nmatches < mMotionModelMinInitialMatches:
      log low matches
      clear associations
      nmatches = SearchByProjection(current, last, thRetry, isMonoLike)

[7] Minimum retry matches check
    if nmatches < mMotionModelMinRetryMatches:
      IMU sensor?
        yes -> return true
        no  -> log failure, return false

[8] Pose optimization
    Optimizer::PoseOptimization(&mCurrentFrame)

[9] Outlier removal & inlier counting
    For each matched point:
      if outlier:
        - clear association
        - clear trackInView(L/R)
        - decrement nmatches
      else if Observations() > 0:
        - nmatchesMap++

[10] Final decision
    IMU sensor?
      yes -> return true
      no:
        nmatchesMap >= mMotionModelMinOptimizedMapMatches ?
          yes -> log success, return true
          no  -> log failure, return false
```

## TrackLocalMap()

**Purpose**: Use the local map (nearby keyframes + map points) to refine the current pose and decide if tracking was successful.

**Control flow**:

```text
TrackLocalMap()
  |
  v
[1] Update local map context
    UpdateLocalMap()
    SearchLocalPoints()   // project & match local points into current frame

[2] Pose optimization (visual or inertial)
    if !mpAtlas->isImuInitialized():
      Optimizer::PoseOptimization(&mCurrentFrame)
    else:
      if !mbMapUpdated:
        PoseInertialOptimizationLastFrame(&mCurrentFrame)
      else:
        PoseInertialOptimizationLastKeyFrame(&mCurrentFrame)

[3] Count inlier matches and update map-point stats
    mnMatchesInliers = 0
    for each feature i:
      if has MapPoint:
        if not outlier:
          MapPoint->IncreaseFound()
          if Observations() > 0:
            mnMatchesInliers++
        else if sensor == STEREO:
          clear association (drop stereo outlier)

[4] Expose inlier count to LocalMapping
    mpLocalMapper->mnMatchesInliers = mnMatchesInliers

[5] Generic success check
    if mnMatchesInliers > mLocalMapGenericMinInliers:
      return true

[6] Mode-specific checks
    if sensor == IMU_MONOCULAR:
      if (mnMatchesInliers < 15 && imuInitialized) ||
         (mnMatchesInliers < 50 && !imuInitialized):
        return false
      else:
        return true

    else if sensor == IMU_STEREO:
      if mnMatchesInliers < 15:
        return false
      else:
        return true

    else (pure visual: MONO or STEREO/RGBD):
      if mnMatchesInliers < mLocalMapVisualMinInliers:
        log failure, return false
      else:
        return true
```
