# Keyframe Decision Flow

This documents the `NeedNewKeyFrame()` decision logic in `Tracking.cc`.

## Call Site

`NeedNewKeyFrame()` is called from the main tracking loop after a successful track. If it returns true, `CreateNewKeyFrame()` is invoked.

```text
Track() main loop
       |
       v
  NeedNewKeyFrame()?
       |
       +-- false --> skip
       |
       v true
  bOK? --yes--> CreateNewKeyFrame()
   |
   no
   |
   mInsertKFsLost &&
   sensor is IMU_MONO/IMU_STEREO?
       |
       +-- yes --> CreateNewKeyFrame()
       +-- no  --> skip
```

## NeedNewKeyFrame() Decision

The function is a cascade of early-exits followed by a compound boolean expression.

### Early Exits

```text
  Sensor is IMU_MONO or IMU_STEREO
  AND IMU not initialized?
       |
       +-- yes --> return (timeSinceLastKF >= 0.25s)   [time-based only]
       |
       v no
  LocalMapper stopped or stop requested?
       |
       +-- yes --> return false                         [mapper busy]
       |
       v no
  (continue to main decision)
```

### Inputs

- `nKFs`: Total keyframes in the map.
- `nRefMatches`: Tracked map points in reference KF (with min observations >= 2 or 3).
- `bLocalMappingIdle`: Whether the local mapper is accepting keyframes.
- `nTrackedClose`: Close points currently tracked (stereo/RGBD only).
- `nNonTrackedClose`: Close points not tracked (stereo/RGBD only).
- `bNeedToInsertClose`: `nTrackedClose < MinTrackedClosePoints AND nNonTrackedClose > MinNonTrackedClosePoints`.
- `thRefRatio`: Reference ratio threshold (varies by sensor mode).

### Reference Ratio (thRefRatio) by Sensor

- IMU_MONOCULAR:
  - `inliers > 350` → 0.75
  - `inliers <= 350` → 0.90
- MONOCULAR:
  - `thRefRatio = RefRatioMono` (default 0.9).
- STEREO/RGBD:
  - `nKFs < 2` → `RefRatioStereoFewKFs` (default 0.4).
  - `nKFs >= 2` → `RefRatioStereo` (default 0.75).

### Conditions

- **c1a**: `framesSinceLastKF >= MaxFrames` → Too many frames without a keyframe.
- **c1b**: `framesSinceLastKF >= MinFrames AND mapperIdle` → Enough frames passed and mapper is free.
- **c1c**: Stereo/RGBD only (excludes IMU): `inliers < nRefMatches * WeakTrackingRatio OR bNeedToInsertClose` → tracking is weak.
- **c2**: `(inliers < nRefMatches * thRefRatio OR bNeedToInsertClose) AND inliers > MinInliers` → few tracked points relative to reference, but enough to be valid.
- **c3**: IMU only: `timeSinceLastKF >= 0.5s` → temporal condition for inertial.
- **c4**: IMU_MONO only: `15 < inliers < 75` → low inlier count for IMU monocular.

### Final Decision

```text
  needKeyFrame = ((c1a OR c1b OR c1c) AND c2)
                  OR c3
                  OR c4
       |
       +-- false --> return false
       |
       v true
  Mapper idle or initializing?
       |
       +-- yes --> return true
       |
       v no
  InterruptBA()
       |
  Sensor is MONO or IMU_MONO?
       |
       +-- yes --> return false    [can't force KF for monocular]
       |
       v no
  return (KF queue < MaxKFsInQueue)  [stereo: only if queue isn't full]
```

## Configurable Parameters

All parameters are loaded from the YAML config file. If omitted, defaults are used.

- `Tracking.NewKF.MinTrackedClosePoints` (int, default 100): used in `bNeedToInsertClose` (stereo/RGBD).
- `Tracking.NewKF.MinNonTrackedClosePoints` (int, default 70): used in `bNeedToInsertClose` (stereo/RGBD).
- `Tracking.NewKF.RefRatioMono` (float, default 0.9): used for `thRefRatio` (monocular).
- `Tracking.NewKF.RefRatioStereoFewKFs` (float, default 0.4): used for `thRefRatio` (stereo, `< 2` KFs).
- `Tracking.NewKF.RefRatioStereo` (float, default 0.75): used for `thRefRatio` (stereo, normal).
- `Tracking.NewKF.WeakTrackingRatio` (float, default 0.25): used in **c1c** inlier fraction.
- `Tracking.NewKF.MinInliers` (int, default 15): used as c2 minimum inlier count.
- `Tracking.NewKF.MaxKFsInQueue` (int, default 3): mapper queue limit (stereo).

The following are also relevant but currently hardcoded (IMU paths):

- 0.25s: used in early exit as IMU pre-init time threshold.
- 350: used in `thRefRatio` as IMU mono inlier switch.
- 0.75 / 0.90: used in `thRefRatio` as IMU mono ref ratios.
- 0.5s: used in c3 as IMU temporal threshold.
- 75 / 15: used in c4 as IMU mono inlier bounds.
