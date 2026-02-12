# Keyframe Decision Flow

This documents the `NeedNewKeyFrame()` decision logic in `Tracking.cc`.

## Call Site

`NeedNewKeyFrame()` is called from the main tracking loop after a successful track. If it returns true, `CreateNewKeyFrame()` is invoked.

```
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

```
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

| Variable | Description |
|---|---|
| `nKFs` | Total keyframes in the map |
| `nRefMatches` | Tracked map points in reference KF (with min observations >= 2 or 3) |
| `bLocalMappingIdle` | Whether the local mapper is accepting keyframes |
| `nTrackedClose` | Close points currently tracked (stereo/RGBD only) |
| `nNonTrackedClose` | Close points not tracked (stereo/RGBD only) |
| `bNeedToInsertClose` | `nTrackedClose < MinTrackedClosePoints AND nNonTrackedClose > MinNonTrackedClosePoints` |
| `thRefRatio` | Reference ratio threshold (varies by sensor mode) |

### Reference Ratio (thRefRatio) by Sensor

| Sensor | Condition | thRefRatio |
|---|---|---|
| IMU_MONOCULAR | `inliers > 350` | 0.75 |
| IMU_MONOCULAR | `inliers <= 350` | 0.90 |
| MONOCULAR | -- | `RefRatioMono` (default 0.9) |
| STEREO/RGBD | `nKFs < 2` | `RefRatioStereoFewKFs` (default 0.4) |
| STEREO/RGBD | `nKFs >= 2` | `RefRatioStereo` (default 0.75) |

### Conditions

| Condition | Expression | Meaning |
|---|---|---|
| **c1a** | `framesSinceLastKF >= MaxFrames` | Too many frames without a keyframe |
| **c1b** | `framesSinceLastKF >= MinFrames AND mapperIdle` | Enough frames passed and mapper is free |
| **c1c** | Stereo/RGBD only (excludes IMU): `inliers < nRefMatches * WeakTrackingRatio OR bNeedToInsertClose` | Tracking is weak |
| **c2** | `(inliers < nRefMatches * thRefRatio OR bNeedToInsertClose) AND inliers > MinInliers` | Few tracked points relative to reference, but enough to be valid |
| **c3** | IMU only: `timeSinceLastKF >= 0.5s` | Temporal condition for inertial |
| **c4** | IMU_MONO only: `15 < inliers < 75` | Low inlier count for IMU monocular |

### Final Decision

```
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

| YAML Key | Type | Default | Used in |
|---|---|---|---|
| `Tracking.NewKF.MinTrackedClosePoints` | int | 100 | `bNeedToInsertClose` (stereo/RGBD) |
| `Tracking.NewKF.MinNonTrackedClosePoints` | int | 70 | `bNeedToInsertClose` (stereo/RGBD) |
| `Tracking.NewKF.RefRatioMono` | float | 0.9 | `thRefRatio` (monocular) |
| `Tracking.NewKF.RefRatioStereoFewKFs` | float | 0.4 | `thRefRatio` (stereo, <2 KFs) |
| `Tracking.NewKF.RefRatioStereo` | float | 0.75 | `thRefRatio` (stereo, normal) |
| `Tracking.NewKF.WeakTrackingRatio` | float | 0.25 | c1c inlier fraction |
| `Tracking.NewKF.MinInliers` | int | 15 | c2 minimum inlier count |
| `Tracking.NewKF.MaxKFsInQueue` | int | 3 | Mapper queue limit (stereo) |

The following are also relevant but currently hardcoded (IMU paths):

| Value | Used in | Description |
|---|---|---|
| 0.25s | Early exit | IMU pre-init time threshold |
| 350 | `thRefRatio` | IMU mono inlier switch |
| 0.75 / 0.90 | `thRefRatio` | IMU mono ref ratios |
| 0.5s | c3 | IMU temporal threshold |
| 75 / 15 | c4 | IMU mono inlier bounds |
