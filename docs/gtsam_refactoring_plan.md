# Refactoring Plan: Replacing g2o with GTSAM in ORB-SLAM3

## Context

ORB-SLAM3 uses [g2o](https://github.com/RainerKuemmerle/g2o) as its sole optimization backend. All visual bundle adjustment, inertial optimization, loop closing, and pose graph optimization functions call into g2o's sparse graph solver infrastructure. The goal of this refactoring is to replace g2o with [GTSAM](https://gtsam.org/) while maintaining identical mathematical behavior across all optimization functions, preferring GTSAM built-in types and factors wherever possible and using custom implementations only where no built-in equivalent exists.

**Motivation:**

- GTSAM provides a cleaner factor-graph API better aligned with probabilistic SLAM formulations
- GTSAM is already used in the sibling `uwlocalization` package in this workspace, reducing total dependency footprint
- GTSAM's `imuBias::ConstantBias`, `GenericProjectionFactor`, `GenericStereoFactor`, `BetweenFactor`, and `PriorFactor` cover the majority of ORB-SLAM3's optimization factors directly
- GTSAM's `Similarity3` natively supports the 7-DoF similarity transforms required for monocular loop closing

---

## Scope

### Primary Files to Rewrite

| File | Role |
| --- | --- |
| `src/Optimizer.cc` | All optimization functions (~5585 lines) |
| `include/G2oTypes.h` + `src/G2oTypes.cc` | Custom g2o vertex/edge types → GTSAM factors |
| `include/OptimizableTypes.h` + `src/OptimizableTypes.cpp` | Additional custom g2o types → GTSAM factors |

### Secondary Files to Update

| File | Change |
| --- | --- |
| `CMakeLists.txt` | Remove g2o, add GTSAM |
| `src/CMakeLists.txt` | Update source list and link libraries |
| `include/LoopClosing.h` | Replace `g2o::Sim3` in typedef and member variables |
| `include/Optimizer.h` | Remove `g2o::Sim3` from `OptimizeSim3` signature |
| `include/Converter.h` + `src/Converter.cc` | Remove g2o conversion utilities |
| `src/LoopClosing.cc` | Update all `g2o::Sim3` variable types |

### New Files to Create

| File | Purpose |
| --- | --- |
| `include/GTSAMTypes.h` | Custom GTSAM factor class declarations, key utilities, conversion helpers |
| `src/GTSAMTypes.cc` | Custom GTSAM factor implementations |

### Files to Delete (after full migration)

- `include/G2oTypes.h`
- `src/G2oTypes.cc`
- `include/OptimizableTypes.h`
- `src/OptimizableTypes.cpp`

---

## Type Mapping

### Variables: g2o Vertices → GTSAM Values

| g2o Vertex | DoF | GTSAM Type | Key Symbol |
| --- | --- | --- | --- |
| `g2o::VertexSE3Expmap` | 6 | `gtsam::Pose3` (Tcw) | `Symbol('x', kfId)` |
| `VertexPose` (ImuCamPose body) | 6 | `gtsam::Pose3` (Twb, IMU body frame) | `Symbol('p', kfId)` |
| `VertexPose4DoF` | 4 | `gtsam::Pose3` + roll/pitch prior | `Symbol('f', kfId)` |
| `g2o::VertexSBAPointXYZ` | 3 | `gtsam::Point3` | `Symbol('l', mpId)` |
| `VertexVelocity` | 3 | `gtsam::Vector3` | `Symbol('v', kfId)` |
| `VertexGyroBias` + `VertexAccBias` | 3+3 | `gtsam::imuBias::ConstantBias` | `Symbol('b', kfId)` |
| `VertexGDir` | 2 | `gtsam::Rot3` (constrained Jacobian) | `Symbol('g', 0)` |
| `VertexScale` | 1 | `double` stored as `log(s)` | `Symbol('s', 0)` |
| `g2o::VertexSim3Expmap` | 7 | `gtsam::Similarity3` | `Symbol('S', kfId)` |

**Bias convention:** GTSAM's `imuBias::ConstantBias` stores `(biasAcc, biasGyro)`. ORB-SLAM3's `IMU::Bias` stores `(bax, bay, baz, bwx, bwy, bwz)` — same order, direct conversion:

```cpp
gtsam::imuBias::ConstantBias toGTSAMBias(const IMU::Bias& b) {
    return gtsam::imuBias::ConstantBias(
        gtsam::Vector3(b.bax, b.bay, b.baz),
        gtsam::Vector3(b.bwx, b.bwy, b.bwz));
}
IMU::Bias fromGTSAMBias(const gtsam::imuBias::ConstantBias& cb) {
    const auto& ba = cb.accelerometer(); const auto& bg = cb.gyroscope();
    return IMU::Bias(ba.x(), ba.y(), ba.z(), bg.x(), bg.y(), bg.z());
}
```

**Scale convention:** Store `log(s)` in Values so GTSAM uses additive retraction. Recover `s = exp(log_s)` inside factors.

**Coordinate convention:** GTSAM `Pose3` for `VertexPose` stores `Twb` (world-from-IMU-body). Factors internally compute `Tcw = Tcb * Twb⁻¹` using constant `Tbc` passed as a constructor argument. This is handled identically to `GenericProjectionFactor`'s `body_P_sensor` parameter.

---

## Factor Mapping: Prefer GTSAM Built-ins

### GTSAM Built-in Factors (use directly, no custom code needed)

| g2o Edge | GTSAM Built-in Factor | Notes |
| --- | --- | --- |
| `g2o::EdgeSE3ProjectXYZ` (pinhole mono) | `gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2>` | Pass `body_P_sensor=Tbc` for multi-camera |
| `g2o::EdgeStereoSE3ProjectXYZ` (pinhole stereo) | `gtsam::GenericStereoFactor<Pose3, Point3>` | Use `Cal3_S2Stereo` with baseline |
| `EdgeMono` (pinhole, with `VertexPose`) | `gtsam::GenericProjectionFactor<Pose3, Point3, Cal3_S2>` | Set `body_P_sensor=Tbc[cam_idx]` per camera |
| `EdgeStereo` (pinhole, with `VertexPose`) | `gtsam::GenericStereoFactor<Pose3, Point3>` | `Pose3` is `Twb`; `body_P_sensor=Tbc` |
| `EdgeGyroRW` + `EdgeAccRW` | `gtsam::BetweenFactor<imuBias::ConstantBias>` | 6×6 block-diagonal info matrix (see §Bias Random Walk) |
| `g2o::EdgeSim3` | `gtsam::BetweenFactor<gtsam::Similarity3>` | `Similarity3` has GTSAM traits; `BetweenFactor` works directly |
| `EdgePriorAcc` + `EdgePriorGyro` | `gtsam::PriorFactor<imuBias::ConstantBias>` | Single 6D prior on combined bias |
| Fixed KF pose | `gtsam::PriorFactor<gtsam::Pose3>` | Tight sigma 1e-6 replaces `setFixed(true)` |
| Fixed Sim3 pose | `gtsam::PriorFactor<gtsam::Similarity3>` | Tight noise for fixed reference KF |
| Fixed scale (stereo `OptimizeSim3`) | `gtsam::PriorFactor<double>` | Prior on `log_s=0` with tight noise |

**`GenericProjectionFactor` with multi-camera support:**

```cpp
// KF with multiple cameras: one factor per camera, each with its own Tbc
auto Tbc_cam0 = pKF->GetCamera(0)->GetTbc();  // body-to-cam extrinsic
auto noise = makeHuberNoise(2, 5.991, invSigma2);
auto cal = boost::make_shared<gtsam::Cal3_S2>(fx, fy, 0, cx, cy);
graph.add(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(
    obs2d, noise, imuPoseKey(kfId), pointKey(mpId), cal,
    /*body_P_sensor=*/ sophusToGTSAMPose(Tbc_cam0)));
```

**`BetweenFactor<Similarity3>` for Sim3 loop edges:**

```cpp
// Replaces g2o::EdgeSim3 exactly:
// BetweenFactor error = Logmap(measured^-1 * Ti^-1 * Tj) on Similarity3 manifold
graph.add(gtsam::BetweenFactor<gtsam::Similarity3>(
    sim3Key(id_i), sim3Key(id_j), Sij_measured, uniformNoise7));
```

### Custom Factors (only where GTSAM has no built-in equivalent)

| g2o Edge | Custom Factor | Reason Custom is Needed |
| --- | --- | --- |
| `EdgeSE3ProjectXYZOnlyPose` (pinhole) | `MonoOnlyPoseFactor` | GTSAM has no unary projection factor with fixed 3D point |
| `EdgeStereoSE3ProjectXYZOnlyPose` | `StereoOnlyPoseFactor` | Same reason |
| `EdgeMonoOnlyPose` (fisheye+OnlyPose) | `MonoOnlyPoseFactor` | Same reason |
| `EdgeStereoOnlyPose` (fisheye+OnlyPose) | `StereoOnlyPoseFactor` | Same reason |
| `EdgeMono` (fisheye camera) | `FisheyeProjectionFactor` | `Cal3_S2` doesn't model Kannala-Brandt distortion |
| `EdgeStereo` (fisheye camera) | `FisheyeStereoFactor` | Same reason |
| `EdgeInertial` | `InertialFactor` | ORB-SLAM3 uses its own `IMU::Preintegrated`; `CombinedImuFactor` requires GTSAM's preintegration format |
| `EdgeInertialGS` | `InertialGSFactor` | 7-key factor with gravity `Rot3` and scale `double`; no GTSAM equivalent |
| `EdgePriorPoseImu` | `PriorNavFactor` | 15D joint prior with cross-covariance across pose+vel+bias; splitting into 3 separate PriorFactors loses cross-correlation terms |
| `EdgeSim3ProjectXYZ` | `Sim3ProjectionFactor` | Unary factor on `Similarity3` with fixed 3D point; no GTSAM built-in |
| `EdgeInverseSim3ProjectXYZ` | `InverseSim3ProjectionFactor` | Same reason |
| `Edge4DoF` | `FourDOFBetweenFactor` | `BetweenFactor<Pose3>` updates all 6 DoF; 4-DoF constraint requires custom Jacobian structure |

**Summary:** 4 built-in GTSAM factors cover 9 of the 17 g2o edge types. Only 9 custom factors are needed, down from 17 if everything were custom.

### Noise Models

| g2o Concept | GTSAM Equivalent |
| --- | --- |
| `setInformation(I * invSigma2)` | `noiseModel::Isotropic::Precision(dim, invSigma2)` |
| `setInformation(Matrix)` | `noiseModel::Gaussian::Information(M)` |
| `RobustKernelHuber(sqrt(5.991))` | `noiseModel::Robust::Create(mEstimator::Huber::Create(sqrt(5.991)), baseModel)` |
| `RobustKernelHuber(sqrt(7.815))` (stereo) | Same pattern with `sqrt(7.815)` |
| `RobustKernelHuber(sqrt(16.92))` (IMU) | Same pattern with `sqrt(16.92)` |

> **Huber delta convention:** g2o's `setDelta(k)` and GTSAM's `Huber::Create(k)` both threshold the whitened residual norm — directly equivalent.

### Optimizer Infrastructure

| g2o | GTSAM |
| --- | --- |
| `g2o::SparseOptimizer` | `gtsam::NonlinearFactorGraph` + `gtsam::Values` |
| `BlockSolver_6_3 / BlockSolverX` | Handled internally by GTSAM's factor graph structure |
| `OptimizationAlgorithmLevenberg` | `gtsam::LevenbergMarquardtOptimizer` |
| `LinearSolverEigen` | Handled internally by GTSAM's Cholesky backend |
| `vertex.setMarginalized(true)` | `Ordering::ColamdConstrainedFirst(graph, landmarkKeys)` |
| `optimizer.setForceStopFlag(flag)` | Manual `opt.iterate()` loop with flag check between iterations |

---

## Infrastructure (GTSAMTypes.h/cc)

### Key Generation Utilities

```cpp
inline gtsam::Key poseKey(uint32_t id)    { return gtsam::Symbol('x', id); }
inline gtsam::Key imuPoseKey(uint32_t id) { return gtsam::Symbol('p', id); }
inline gtsam::Key pointKey(uint32_t id)   { return gtsam::Symbol('l', id); }
inline gtsam::Key velKey(uint32_t id)     { return gtsam::Symbol('v', id); }
inline gtsam::Key biasKey(uint32_t id)    { return gtsam::Symbol('b', id); }
inline gtsam::Key gravKey()               { return gtsam::Symbol('g', 0); }
inline gtsam::Key scaleKey()              { return gtsam::Symbol('s', 0); }
inline gtsam::Key sim3Key(uint32_t id)    { return gtsam::Symbol('S', id); }
```

### Conversion Helpers

```cpp
gtsam::Pose3         sophusToGTSAMPose(const Sophus::SE3f& T);
Sophus::SE3f         gtsamToSophusPose(const gtsam::Pose3& P);
gtsam::imuBias::ConstantBias toGTSAMBias(const IMU::Bias& b);
IMU::Bias            fromGTSAMBias(const gtsam::imuBias::ConstantBias& cb);
gtsam::Similarity3   toGTSAMSim3(const Sophus::Sim3f& S);
Sophus::Sim3f        fromGTSAMSim3(const gtsam::Similarity3& S);
gtsam::Cal3_S2       toGTSAMCal(const GeometricCamera* pCam);  // pinhole only
gtsam::Cal3_S2Stereo toGTSAMStereoCal(const GeometricCamera* pCam, float bf);
```

### Noise Model Helper

```cpp
// Returns Huber-wrapped isotropic noise model:
//   baseModel = Isotropic::Precision(dim, invSigma2)
//   wrapped   = Robust(Huber(sqrt(chi2Threshold)), baseModel)
gtsam::SharedNoiseModel makeHuberNoise(int dim, double chi2Threshold, double invSigma2);
```

---

## Custom Factor Implementations

### `MonoOnlyPoseFactor` (replaces `EdgeSE3ProjectXYZOnlyPose` and `EdgeMonoOnlyPose`)

Unary factor on `Pose3`; 3D point is fixed (stored in factor, not a GTSAM variable). Supports both pinhole and fisheye via `GeometricCamera::project()`:

```cpp
class MonoOnlyPoseFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
    gtsam::Point3    Xw_;        // fixed world point
    gtsam::Pose3     Tbc_;       // body-to-camera extrinsic
    GeometricCamera* pCamera_;

    gtsam::Vector evaluateError(const gtsam::Pose3& Twb,
                                gtsam::OptionalMatrixType H) const override {
        gtsam::Matrix6 dTcw_dTwb;
        gtsam::Pose3 Tcw = (Twb * Tbc_).inverse(H ? &dTcw_dTwb : nullptr);
        gtsam::Matrix36 dXc_dTcw;
        gtsam::Point3 Xc = Tcw.transformFrom(Xw_, H ? &dXc_dTcw : nullptr, nullptr);
        gtsam::Matrix23 dProj_dXc;
        gtsam::Point2 proj = pCamera_->project(Xc, H ? &dProj_dXc : nullptr);
        if (H) *H = -dProj_dXc * dXc_dTcw * dTcw_dTwb;
        return measurement() - proj;
    }
};
```

### `StereoOnlyPoseFactor` (replaces `EdgeStereoSE3ProjectXYZOnlyPose` and `EdgeStereoOnlyPose`)

Same pattern as `MonoOnlyPoseFactor` but 3D measurement `[u_l, v, u_r]`, error is 3D.

### `FisheyeProjectionFactor` (replaces `EdgeMono`/`EdgeSE3ProjectXYZ` for fisheye cameras)

Binary factor `Pose3 × Point3 → ℝ²` using `GeometricCamera::project()` (Kannala-Brandt model). Only needed when `pCamera->GetType() == GeometricCamera::CAM_FISHEYE`. For pinhole cameras, use `GenericProjectionFactor` instead.

### `FisheyeStereoFactor` (replaces `EdgeStereo` for fisheye cameras)

Binary factor `Pose3 × Point3 → ℝ³` using the fisheye stereo projection.

### `InertialFactor` (replaces `EdgeInertial`)

Derives from `gtsam::NonlinearFactor`. Connects 5 keys: `pose1(Twb)`, `vel1`, `bias1(ConstantBias)`, `pose2(Twb)`, `vel2`. (Compared to g2o's 6 vertices, gyro and acc biases are merged into one `ConstantBias` key.)

```cpp
class InertialFactor : public gtsam::NonlinearFactor {
    IMU::Preintegrated* mpInt_;  // ORB-SLAM3's preintegration result
    gtsam::Vector3 g_;           // gravity in world frame

  public:
    // error() replicates EdgeInertial::computeError() exactly:
    //   9D residual = [rotation_err; velocity_err; position_err]
    double error(const gtsam::Values& c) const override;

    // linearize() replicates EdgeInertial::linearizeOplus() exactly:
    //   returns JacobianFactor with same 9×(6+3+6+6+3) Jacobian blocks
    boost::shared_ptr<gtsam::GaussianFactor>
    linearize(const gtsam::Values& c) const override;
};
```

> **Why not `CombinedImuFactor`?** GTSAM's `CombinedImuFactor` requires `PreintegratedCombinedMeasurements`, which means re-running IMU integration using GTSAM's API. ORB-SLAM3's `IMU::Preintegrated` uses a different (but mathematically equivalent) integration pipeline. Porting the preintegration is outside the primary scope and risks introducing subtle numerical differences. `InertialFactor` preserves all existing behavior.

### `InertialGSFactor` (replaces `EdgeInertialGS`)

7-key factor: `pose1(Twb)`, `vel1`, `bias1(ConstantBias)`, `pose2(Twb)`, `vel2`, `gravityRot(Rot3)`, `logScale(double)`. Replicates `EdgeInertialGS::computeError()` and `linearizeOplus()` exactly.

**Gravity direction Jacobian:** Only 2 columns are non-zero (the two tangent directions that tilt the gravity vector). The 3rd column (yaw-around-gravity direction) is set to zero, matching `VertexGDir::oplusImpl`:

```cpp
gtsam::Matrix93 Jg = computeGravityJacobian(...);
Jg.col(2).setZero();   // freeze yaw around gravity axis
```

**Scale Jacobian:** Chain rule on `s = exp(log_s)`:

```cpp
// dE/d(log_s) = dE/ds * ds/d(log_s) = dE/ds * s
gtsam::Matrix91 Js = dE_ds * scale;
```

### `PriorNavFactor` (replaces `EdgePriorPoseImu`)

3-key factor: `pose(Twb)`, `vel(Vector3)`, `bias(ConstantBias)`. 15D residual with 15×15 information matrix (preserving cross-covariance across all 15 DoF — the reason a joint factor is needed rather than three separate `PriorFactor`s):

```cpp
// 15D residual = [LogSO3(R_prior^T * R_cur);   // 3D rotation error
//                 t_cur - t_prior;               // 3D translation error
//                 v_cur - v_prior;               // 3D velocity error
//                 ba_cur - ba_prior;             // 3D accel bias error
//                 bg_cur - bg_prior]             // 3D gyro bias error
```

### `Sim3ProjectionFactor` (replaces `EdgeSim3ProjectXYZ`)

Unary factor on `Similarity3`. The 3D point is fixed (stored in factor, not a GTSAM variable):

```cpp
class Sim3ProjectionFactor : public gtsam::NoiseModelFactorN<gtsam::Similarity3> {
    gtsam::Point3    P3Dc_;    // fixed point in camera 1's frame
    GeometricCamera* pCamera_;

    gtsam::Vector evaluateError(const gtsam::Similarity3& S12, ...) const override {
        gtsam::Point3 p = S12.transformFrom(P3Dc_);
        return measurement() - pCamera_->project(p);
    }
};
```

### `InverseSim3ProjectionFactor` (replaces `EdgeInverseSim3ProjectXYZ`)

Same as `Sim3ProjectionFactor` but applies `S12.inverse()` before projecting (inverse projection direction):

```cpp
gtsam::Point3 p = S12.inverse().transformFrom(P3Dc2_);
return measurement() - pCamera_->project(p);
```

### `FourDOFBetweenFactor` (replaces `Edge4DoF`)

Binary factor between two `Pose3` variables (stored as `Tcw` in this graph context). Replicates `Edge4DoF::computeError()` exactly with 6D residual. Roll and pitch are frozen via tight `PriorFactor<Pose3>` constraints on each KF:

```cpp
gtsam::Vector evaluateError(const gtsam::Pose3& Ti, const gtsam::Pose3& Tj, ...) const {
    // Replicate Edge4DoF::computeError():
    auto Rcwi = Ti.rotation().matrix();  auto tcwi = Ti.translation();
    auto Rcwj = Tj.rotation().matrix();  auto tcwj = Tj.translation();
    gtsam::Vector6 err;
    err.head<3>() = LogSO3(Rcwi * Rcwj.transpose() * dRij_.transpose());
    err.tail<3>() = Rcwi * (-Rcwj.transpose() * tcwj) + tcwi - dtij_;
    return err;
}
```

---

## Bias Random Walk: Combined ConstantBias Factor

The g2o implementation uses separate `EdgeGyroRW` (3×3 info) and `EdgeAccRW` (3×3 info). With `imuBias::ConstantBias` (6D combined), both are replaced by a single `BetweenFactor<ConstantBias>` with a 6×6 block-diagonal information matrix:

```cpp
// InfoA = C.block<3,3>(12,12)  (accelerometer random walk)
// InfoG = C.block<3,3>(9,9)    (gyroscope random walk)
gtsam::Matrix6 Info6 = gtsam::Matrix6::Zero();
Info6.block<3,3>(0,0) = InfoA;   // acc block (ConstantBias stores acc first)
Info6.block<3,3>(3,3) = InfoG;   // gyro block
auto rwNoise = gtsam::noiseModel::Gaussian::Information(Info6);
graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
    biasKey(prevId), biasKey(curId),
    gtsam::imuBias::ConstantBias(),  // zero mean (biases should not change)
    rwNoise));
```

---

## Optimization Function Implementation Plan

### Phase 1 — Infrastructure

Create `GTSAMTypes.h/cc` with all utilities and custom factor implementations.

### Phase 2 — Visual-Only Functions

#### `BundleAdjustment` / `GlobalBundleAdjustment`

```cpp
// Poses as Pose3(Tcw), landmarks as Point3
// Fixed KF via PriorFactor<Pose3> with tight noise
// Per-observation: GenericProjectionFactor (pinhole) or FisheyeProjectionFactor
// Landmark ordering for Schur complement (equivalent to setMarginalized):
gtsam::KeyVector landmarkKeys; // all point keys
gtsam::Ordering ordering = gtsam::Ordering::ColamdConstrainedFirst(graph, landmarkKeys);
gtsam::LevenbergMarquardtOptimizer opt(graph, initial, ordering, params);
```

#### `LocalBundleAdjustment` (visual-only)

Same structure. Fixed KFs: `PriorFactor<Pose3>` with sigma 1e-6 (replaces `vertex.setFixed(true)`).

#### `PoseOptimization`

Single-frame pose with 4 outer iterations and manual outlier exclusion:

```cpp
for (int iter = 0; iter < 4; ++iter) {
    // Rebuild graph with non-outlier MonoOnlyPoseFactor / StereoOnlyPoseFactor
    auto result = gtsam::LevenbergMarquardtOptimizer(activeGraph, initial, params).optimize();
    initial = result;
    // Detect outliers: 2.0 * factor.error(initial) gives chi² value
    for (size_t i = 0; i < factors.size(); ++i)
        isOutlier[i] = (2.0 * factors[i]->error(initial) > chi2Threshold[i]);
}
```

#### `OptimizeSim3`

Single `Similarity3` variable. `Sim3ProjectionFactor` and `InverseSim3ProjectionFactor`. For stereo (fixed scale): `PriorFactor<double>` on `scaleKey()` with tight noise.

### Phase 3 — Inertial Functions

#### `FullInertialBA`

Per-KF `Pose3(Twb)`, `Vector3(vel)`, `ConstantBias(bias)`. Between KFs: `InertialFactor` + `BetweenFactor<ConstantBias>` (bias random walk). Visual: `GenericProjectionFactor` / `GenericStereoFactor`. Bias prior when `bInit=true`: `PriorFactor<ConstantBias>`.

Force-stop flag:

```cpp
gtsam::LevenbergMarquardtOptimizer opt(graph, initial, params);
for (int i = 0; i < nIterations; ++i) {
    if (pbStopFlag && *pbStopFlag) break;
    opt.iterate();
}
gtsam::Values result = opt.values();
```

#### `LocalInertialBA`

Windowed version of `FullInertialBA`. Fixed KF at window start via `PriorFactor<Pose3>`.

#### `InertialOptimization` (3 overloads)

- **Overload 1** (full): `InertialGSFactor` over all KF pairs; optimizes velocities, biases, `Rot3(gravity)`, `double(log_scale)`
- **Overload 2** (biases only): `InertialFactor` with fixed poses (tight `PriorFactor<Pose3>`); optimizes velocities and `ConstantBias`
- **Overload 3** (gravity+scale only): `InertialGSFactor` with fixed poses, velocities, biases; optimizes only `Rot3` and `double`

#### `MergeInertialBA`

Dual-map fusion using `PriorNavFactor` (15D joint prior) to link the two submaps at the merge boundary.

#### `PoseInertialOptimizationLastKeyFrame` / `LastFrame`

Single-frame inertial pose. After optimization, assembles the 30×30 Hessian by calling `computeJacobians()` on each custom factor and summing `Jᵀ·Ω·J` blocks. Then calls the existing pure-Eigen `Optimizer::Marginalize()` (no changes needed — it is a standalone matrix utility) to produce the 15×15 `ConstraintPoseImu`.

### Phase 4 — Pose Graph / Loop Closing

#### `OptimizeEssentialGraph`

All KFs as `Similarity3`. Loop and spanning-tree edges: `BetweenFactor<Similarity3>`. Fixed reference KF: `PriorFactor<Similarity3>`.

#### `OptimizeEssentialGraph4DoF`

All KFs as `Pose3(Tcw)`. Edges: `FourDOFBetweenFactor`. Roll/pitch constrained via `PriorFactor<Pose3>` with tight noise on those 2 DoF per KF.

---

## Build System Changes

### `CMakeLists.txt`

```cmake
# Remove:
find_package(g2o REQUIRED CONFIG)

# Add:
find_package(GTSAM REQUIRED)
message(STATUS "Found GTSAM version: ${GTSAM_VERSION}")
```

### `src/CMakeLists.txt`

```cmake
# Remove from source list: G2oTypes.cc, OptimizableTypes.cpp
# Add to source list:       GTSAMTypes.cc

# Remove link target: g2o::g2o (or g2o::g2o_interface)
# Add:
target_include_directories(${PROJECT_NAME} PUBLIC ${GTSAM_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ... gtsam ...)
```

### `cmake/ORB_SLAM3Config.cmake.in`

```cmake
# Replace: find_dependency(g2o CONFIG)
# With:    find_dependency(GTSAM)
```

---

## Key Risks and Mitigations

| Risk | Mitigation |
| --- | --- |
| **Outlier exclusion** — g2o uses `setLevel(1)` between passes | Track `isOutlier[]` per factor; rebuild factor graph each outer iteration; use `2.0 * factor->error(values)` for chi² |
| **Landmark marginalization** — g2o's `setMarginalized(true)` triggers Schur complement | `Ordering::ColamdConstrainedFirst(graph, landmarkKeys)` eliminates landmarks first |
| **Force-stop flag** in GBA | Manual `opt.iterate()` loop with flag check between iterations |
| **Scale retraction** — `VertexScale::oplusImpl` uses multiplicative `s *= exp(δ)` | Store `log(s)` in Values; additive GTSAM retraction works correctly |
| **GDir 2-DoF constraint** — g2o freezes the yaw-around-gravity update | Zero out column 2 of the gravity Jacobian in `InertialGSFactor::linearize()` |
| **Hessian for ConstraintPoseImu** — g2o `GetHessian()` on edges assembles 30×30 H | Expose `computeJacobians()` on custom factors; assemble H manually; reuse existing `Optimizer::Marginalize()` unchanged |
| **Bias split** — g2o has separate 3D gyro/acc vertices | Single `ConstantBias`; `BetweenFactor<ConstantBias>` with 6×6 block-diagonal info matrix |
| **Coordinate convention** — ORB-SLAM3 uses `Tcw`; VertexPose uses `Twb` | VertexPose-style factors store `Twb` and compute `Tcw = Tcb * Twb⁻¹`; document per-function convention |
| **`g2o::Sim3` in LoopClosing interface** | Replace with `gtsam::Similarity3` in `LoopClosing.h` typedef and all `mg2o*` members |
| **Boost `shared_ptr` in GTSAM 4.3** | GTSAM's `find_package` exports Boost transitively; verify with explicit `find_package(Boost REQUIRED)` |
| **Fisheye vs pinhole dispatch** | Check `pCamera->GetType() == GeometricCamera::CAM_FISHEYE` at factor construction time; use `GenericProjectionFactor` for pinhole, `FisheyeProjectionFactor` for fisheye |

---

## Verification Plan

### 1. Unit Tests per Custom Factor

For each custom factor, verify Jacobians numerically:

```cpp
#include <gtsam/base/numericalDerivative.h>
// Error at true values ≈ 0; analytical H matches numerical H to 1e-5
auto H_num = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    [&](const gtsam::Pose3& p){ return factor.evaluateError(p, truePoint); }, truePose);
EXPECT_TRUE(gtsam::assert_equal(H_analytical, H_num, 1e-5));
```

### 2. Optimizer Regression Tests

Per optimization function: build a synthetic scene, add noise, perturb initial values, run g2o and GTSAM from the same start. Compare final graph cost and optimized values (rotation error < 1e-4 rad, translation < 1e-3 m).

### 3. End-to-End Functional Tests

Run on standard sequences; compare trajectory RMSE vs g2o baseline:

- **Monocular** — BA, Sim3 loop closing, essential graph
- **Stereo** — stereo BA, scale-fixed essential graph
- **Visual-inertial** — inertial BA, gravity/scale initialization

### 4. Build Verification

```bash
cd /home/a.lunawat/ws/ros2_ws
colcon build --packages-select ORB_SLAM3 --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select ORB_SLAM3
```

---

## Implementation Sequence

| Step | Action | Files |
| --- | --- | --- |
| 1 | Create key utilities, conversions, noise model helpers | `include/GTSAMTypes.h`, `src/GTSAMTypes.cc` |
| 2 | Implement `MonoOnlyPoseFactor`, `StereoOnlyPoseFactor` | `GTSAMTypes.h/cc` |
| 3 | Implement `FisheyeProjectionFactor`, `FisheyeStereoFactor` | `GTSAMTypes.h/cc` |
| 4 | Implement `InertialFactor` (port EdgeInertial exactly) | `GTSAMTypes.h/cc` |
| 5 | Implement `InertialGSFactor` | `GTSAMTypes.h/cc` |
| 6 | Implement `PriorNavFactor` | `GTSAMTypes.h/cc` |
| 7 | Implement `Sim3ProjectionFactor`, `InverseSim3ProjectionFactor` | `GTSAMTypes.h/cc` |
| 8 | Implement `FourDOFBetweenFactor` | `GTSAMTypes.h/cc` |
| 9 | Rewrite `BundleAdjustment` / `GlobalBundleAdjustment` | `src/Optimizer.cc` |
| 10 | Rewrite `LocalBundleAdjustment` (visual path) | `src/Optimizer.cc` |
| 11 | Rewrite `PoseOptimization` | `src/Optimizer.cc` |
| 12 | Rewrite `OptimizeSim3` | `src/Optimizer.cc` |
| 13 | Rewrite `FullInertialBA` | `src/Optimizer.cc` |
| 14 | Rewrite `LocalInertialBA` | `src/Optimizer.cc` |
| 15 | Rewrite `PoseInertialOptimizationLastKeyFrame/Frame` | `src/Optimizer.cc` |
| 16 | Rewrite `InertialOptimization` (3 overloads) | `src/Optimizer.cc` |
| 17 | Rewrite `MergeInertialBA` | `src/Optimizer.cc` |
| 18 | Rewrite `OptimizeEssentialGraph` | `src/Optimizer.cc` |
| 19 | Rewrite `OptimizeEssentialGraph4DoF` | `src/Optimizer.cc` |
| 20 | Update `LoopClosing.h/cc` — replace `g2o::Sim3` with `gtsam::Similarity3` | `include/LoopClosing.h`, `src/LoopClosing.cc` |
| 21 | Update `Optimizer.h`, `Converter.h/cc` | Remove g2o includes/conversions |
| 22 | Update `CMakeLists.txt` files | Remove g2o, add GTSAM |
| 23 | Delete g2o type files | `G2oTypes.h/cc`, `OptimizableTypes.h/cpp` |
| 24 | Build and run verification | `colcon build`, regression tests |
