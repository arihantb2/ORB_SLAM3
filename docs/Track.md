# Track() Control Flow

This documents the main `Tracking::Track()` function in `Tracking.cc` (line 472).

`Track()` is the top-level per-frame entry point. It handles initialization, pose estimation, local map tracking, state transitions, and keyframe insertion.

## High-Level Flow

```
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

### [1] Sanity Checks (lines 474-529)

```
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

### [2] IMU Preintegration (lines 538-542)

Only for `IMU_MONOCULAR` or `IMU_STEREO`, and only if we didn't just create a new map:

```
IMU sensor AND !mbCreatedMap?
  +-- yes --> PreintegrateIMU()
```

### [3] Map Lock & Change Detection (lines 544-555)

Acquires `mMutexMapUpdate`. Checks if the map was modified by another thread (e.g. local mapper, loop closer) since last frame and sets `mbMapUpdated`.

### [4] Initialization (lines 557-578)

```
State == NOT_INITIALIZED?
  |
  +-- STEREO / IMU_STEREO --> StereoInitialization()
  +-- MONO / IMU_MONO     --> MonocularInitialization()
  |
  v
  mState == OK? (initialization succeeded?)
    +-- no  --> save frame, return
    +-- yes --> record first frame ID, continue to post-tracking
```

After successful initialization the function does NOT enter the tracking pipeline below -- it skips directly to trajectory storage at the end.

### [5] Pose Estimation (lines 579-643)

This is the core tracking step. It runs when the system is already initialized.

```
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

### [6] Track Local Map (lines 648-662)

Only runs if pose estimation succeeded (`bOK == true`):

```
bOK?
  +-- yes --> TrackLocalMap()
  |             |
  |             +-- success --> log inliers count
  |             +-- failure --> log failure
  +-- no  --> skip
```

`TrackLocalMap()` refines the pose by matching against the local map (nearby keyframes and their map points). This is where `mnMatchesInliers` is set.

### [7] State Transition (lines 664-679)

```
bOK (after local map)?
  +-- yes --> mState = OK
  +-- no  --> was OK before?
                +-- yes --> mState = LOST, record mTimeStampLost
```

### [8] Post-Tracking: Success Path (lines 694-761)

Only entered when `bOK == true`:

```
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

### [9] Lost Handling & Trajectory Storage (lines 764-814)

```
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

```
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
|---|---|---|---|
| `Tracking.LostResetMinKFs` | int | 999999 | Min KFs in map before LOST triggers a map reset. High value = never reset, always reuse map. |

## Key Functions Called

| Function | Purpose | Called When |
|---|---|---|
| `StereoInitialization()` | Initialize map from stereo pair | `NOT_INITIALIZED`, stereo sensor |
| `MonocularInitialization()` | Initialize map from two monocular frames | `NOT_INITIALIZED`, mono sensor |
| `CheckReplacedInLastFrame()` | Update map points replaced by local mapper | State `OK`, before pose estimation |
| `TrackWithMotionModel()` | Estimate pose using constant-velocity model | State `OK`, velocity available |
| `TrackReferenceKeyFrame()` | Estimate pose via BoW matching to ref KF | State `OK`, no velocity or motion model failed |
| `TrackLocalMap()` | Refine pose against local map points | Pose estimation succeeded |
| `NeedNewKeyFrame()` | Decide whether to insert a keyframe | Tracking succeeded (see [KeyframeDecision.md](KeyframeDecision.md)) |
| `CreateNewKeyFrame()` | Build and insert a new keyframe | `NeedNewKeyFrame()` returned true |
| `PreintegrateIMU()` | Preintegrate IMU measurements | IMU sensor, before map lock |
| `CreateMapInAtlas()` | Abandon current map, start fresh | Timestamp jump, or LOST with enough KFs |
