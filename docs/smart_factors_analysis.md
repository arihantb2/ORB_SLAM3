# Smart Factors for ORB-SLAM3 Optimizer: Feasibility Analysis

## 1. Background: What Are GTSAM Smart Factors?

GTSAM's `SmartProjectionFactor` (and its stereo counterpart `SmartStereoProjectionFactor`) are **landmark-free** multi-view projection factors. Instead of maintaining a 3D landmark variable in the factor graph, a smart factor:

1. Collects **all observations of a single landmark** across multiple poses into one factor.
2. Analytically **triangulates the landmark position** from the current pose estimates at each linearization step.
3. **Marginalizes the landmark** out of the linear system using the Schur complement, producing a Hessian block solely over the involved camera poses.

The result is a factor graph that contains **only pose variables** — landmarks never enter the linear system explicitly.

---

## 2. Current Implementation: How the Code Works

### 2.1 `LocalBundleAdjustment` (`Optimizer.cc:391–946`)

The graph contains two types of explicit variables:
- **Camera poses** (`gtsam::Pose3`, keyed `poseKey(id)` = `Symbol('x', id)`)
- **Landmark positions** (`gtsam::Point3`, keyed `pointKey(id)` = `Symbol('l', id)`)

For every observation of every landmark, a separate binary factor is added to the graph:

| Observation type | Factor | Variables |
|-----------------|--------|-----------|
| Monocular left  | `PinholeMonoTcwFactor` | `Pose3 × Point3 → ℝ²` |
| Stereo (left+right) | `PinholeStereoTcwFactor` | `Pose3 × Point3 → ℝ³` |

The solver uses `LevenbergMarquardtOptimizer` with COLAMD ordering constrained so that landmark keys are eliminated first (`Optimizer.cc:816`). This mimics the Schur complement trick in software but still instantiates all landmark variables in the linear system.

**Representative graph sizes** for a typical LBA window (10–20 local KFs, 500–2000 local map points, ~4–10 observations per point):

| Quantity | Typical range |
|---------|--------------|
| Optimized poses | 10–20 |
| Fixed poses | 5–15 |
| Landmarks (as variables) | 500–2000 |
| Projection factors | 3000–15000 |
| Total variables in system | 515–2035 |

### 2.2 `PoseOptimization` (`Optimizer.cc:245–372`)

This function fixes all landmarks and optimizes **only the single current frame pose**. Each observation becomes a unary factor (`PinholeMonoPoseTcwFactor`, `PinholeStereoPoseTcwFactor`) that hardcodes the 3D landmark position as a constant. The graph has exactly **one variable** (the pose) and up to ~500 measurement factors.

---

## 3. Applicability of Smart Factors

### 3.1 `LocalBundleAdjustment` — Primary Candidate

This is the function where smart factors offer the most potential.

**Mapping to `SmartProjectionFactor`:**

Each local `MapPoint` with observations across multiple keyframes maps directly to one `SmartProjectionPoseFactor<Cal3_S2>` (or its stereo variant). Instead of:

```
// Current: one binary factor per observation
for each (pKF, pMP) observation:
    graph.add(PinholeMonoTcwFactor(poseKey(kf), pointKey(mp), obs, noise, camera))
```

The smart factor equivalent would be:

```
// Smart factor: one factor per landmark, all observations bundled
for each pMP:
    auto sf = SmartProjectionPoseFactor<Cal3_S2>(noise, camera_cal)
    for each (pKF, obs) in pMP->GetObservations():
        sf->add(obs, poseKey(kf))
    graph.add(sf)
```

The landmark position is never inserted into `initial` values — it is solved internally.

### 3.2 `PoseOptimization` — Not a Candidate

Smart factors are **not applicable** here. The function optimizes a single pose against **fixed** landmarks. There is nothing to marginalize and no benefit to landmark-free factors; all landmarks are already constants. The unary factor formulation is already optimal for this use case.

---

## 4. Pros of Switching to Smart Factors in LBA

### 4.1 Smaller Linear System

The most significant benefit. With ~1000 landmarks at 3 DOF each, the current implementation adds 3000 columns to the linear system. Smart factors eliminate all landmark variables from the Hessian. For a window of 15 poses (6 DOF each = 90 DOF) and 1000 landmarks (3000 DOF), the reduction is **97% fewer variables** in the linear system.

### 4.2 Exact Schur Complement

Currently the code approximates the Schur complement ordering by using `ColamdConstrainedFirst` (line 816), which asks COLAMD to order landmark variables for early elimination. Smart factors perform this marginalization **exactly and analytically** during linearization, before any matrix decomposition is needed. This eliminates fill-in that COLAMD can only partially mitigate.

### 4.3 Reduced Memory Footprint

Each `Point3` variable consumes memory in the `Values` container, the factor graph adjacency structure, and the linear system Jacobian matrices. With thousands of landmarks, smart factors cut this overhead substantially, which is relevant for embedded or memory-constrained deployments.

### 4.4 Numerically Better Conditioned System

The reduced-camera Hessian (Schur complement) is denser but smaller and better conditioned than the full pose+landmark system. Iterative solvers converge faster on better-conditioned systems.

### 4.5 Faster Per-Iteration Wall Time for Large Maps

GTSAM benchmarks (Carlone et al., 2014; Indelman et al., 2015) show smart factors outperform explicit landmark formulations when the ratio of landmarks to poses is high — exactly the scenario in LBA, where map points outnumber keyframes by 50:1 to 100:1.

---

## 5. Cons and Complications

### 5.1 Loss of Explicit Landmark Estimates Between Iterations

The current code reads back landmark positions after optimization (`Optimizer.cc:848–856`) and calls `pMP->SetWorldPos(...)`. Smart factors compute the landmark position implicitly but do **not** expose it as a `gtsam::Values` entry. Retrieval requires calling `SmartProjectionFactor::point(result)`, which re-triangulates from the final poses. This is an extra step that must be added explicitly.

### 5.2 Incompatibility with Per-Landmark Outlier Rejection

The current outlier rejection loop (lines 876–945) iterates over individual `(pKFi, pMP, leftIndex)` edge tuples and erases bad observations from the map. Smart factors bundle all observations of a landmark together — they either accept or reject the **entire landmark** (via `SmartProjectionParams::degeneracyMode`), not individual views.

To replicate the current granular per-edge outlier removal, one would need to:
- Extract the triangulation and per-observation reprojection errors from the smart factor, or
- Maintain a parallel per-observation residual tracking structure.

This is the **single largest implementation challenge**.

### 5.3 No Support for Custom Tcw Pose Convention

All current factors use a `Tcw` (world-to-camera) convention implemented in the custom factor classes (`PinholeMonoTcwFactor`, etc., defined in `GTSAMTypes.h`). GTSAM's `SmartProjectionPoseFactor` uses the standard GTSAM `Pose3` convention (Twc, camera-in-world). Adopting smart factors would require either:
- Converting all pose variables to Twc convention (a significant refactor of `sophusToGTSAMPose`/`gtsamToSophusPose` and all downstream code), or
- Wrapping the smart factor with a custom pose-inverting adapter.

### 5.4 No Native Huber Robust Kernel in Smart Factors

The current code uses `makeHuberNoise(dim, chi2, invSigma2)` for robust M-estimation on every projection factor. `SmartProjectionFactor` accepts a `SharedNoiseModel` but applies it uniformly to the entire multi-view factor, not per-observation. Huber per-view robustness is not natively supported.

Workaround options:
- Use `SmartProjectionParams::linearizationMode = HESSIAN` with a custom robust wrapper.
- Accept reduced robustness (Gaussian noise) and rely on post-hoc outlier removal.
- Implement a custom smart factor subclass that evaluates per-view Huber costs — significant engineering effort.

### 5.5 Fixed Keyframe Handling

The current LBA treats "fixed" keyframes (those that observe local map points but are outside the local optimization window) by giving them very tight pose priors and still inserting them as `Pose3` variables. Smart factors require **all observing poses to be in the Values container**. Fixed keyframes must still be present as variables (with priors) or the smart factor must be split to handle the fixed/free boundary — which `SmartProjectionFactor` does not support natively.

### 5.6 No Support for Stereo + Mono Mixed Observations Per Landmark

Some landmarks are observed monoculraly in some keyframes and stereoscopically in others. The current per-edge factor model handles this naturally. `SmartProjectionFactor` is monocular-only; `SmartStereoProjectionFactor` is stereo-only. Mixed observations per landmark would require either:
- Splitting the landmark into two smart factors (one mono, one stereo) and handling the shared triangulation externally, or
- Falling back to the explicit landmark variable for mixed-mode points.

### 5.7 Degenerate Triangulation / Near-Planar Scenes

Smart factors fail to triangulate landmarks with insufficient parallax or degenerate configurations (collinear cameras, near-planar motion). GTSAM handles this via `SmartProjectionParams::degeneracyMode` (IGNORE, ZERO_ON_DEGENERACY, HANDLE_INFINITY). In ORB-SLAM3, newly triangulated points are validated before insertion via `CheckDistEpipolarLine` and depth bounds — this pre-filtering reduces the risk, but degenerate cases must still be handled explicitly in the smart factor path, adding defensive logic.

### 5.8 Interaction with Odometry/Scale Priors

The LBA graph optionally includes `BetweenFactorTcw` and `ScaleFactorTcw` constraints between consecutive keyframes. These factors are independent of landmarks and remain unchanged under a smart factor migration. However, they affect the overall Hessian structure, and the benefit of smart factor marginalization must be re-evaluated in conjunction with these additional pose-pose constraints.

---

## 6. Summary Assessment

| Criterion | Smart Factors | Current Approach |
|-----------|--------------|-----------------|
| Linear system size | Small (poses only) | Large (poses + landmarks) |
| Per-iteration cost | Lower (for high landmark:pose ratio) | Higher |
| Per-edge outlier rejection | Not natively supported | Fully supported |
| Tcw pose convention | Requires refactor | Natively supported |
| Huber per-observation | Not natively supported | Fully supported |
| Mixed mono/stereo per landmark | Requires split | Natively supported |
| Landmark retrieval after opt | Requires extra step | Direct from Values |
| Code complexity | Higher | Moderate |
| COLAMD equivalent | Exact Schur, automatic | Approximate via ordering |

### Recommendation

**Smart factors are viable for `LocalBundleAdjustment` but carry significant implementation cost.** The mathematical benefit is real and would be most noticeable in large LBA windows (>20 keyframes, >2000 landmarks). However, several ORB-SLAM3-specific design choices — the Tcw convention, mixed mono/stereo observations per landmark, per-edge Huber robustness, and fine-grained outlier rejection — are not directly supported by GTSAM's off-the-shelf smart factor API.

**`PoseOptimization` should not be migrated** — there are no landmark variables to eliminate and the current unary factor formulation is already the minimal representation.

### Recommended Path if Pursuing Smart Factors

1. **Convert pose convention** from Tcw to Twc throughout `Optimizer.cc` and `GTSAMTypes.h`. This is a prerequisite for using any standard GTSAM projection factor.
2. **Separate mono and stereo observations per landmark** to use `SmartProjectionPoseFactor` and `SmartStereoProjectionFactor` respectively. For landmarks with mixed observations, keep explicit `Point3` variables with standard binary factors as a fallback.
3. **Implement post-optimization outlier rejection** using `SmartProjectionFactor::point(result)` for triangulated positions, then compute per-view reprojection errors manually — mirroring the current loop at lines 876–945.
4. **Accept Gaussian noise** in the smart factor noise model (dropping Huber per-observation), supplemented by the post-optimization outlier rejection to handle gross outliers.
5. **Benchmark** on representative sequences (e.g., EuRoC MAV, KITTI) to confirm the expected speedup, since the benefit depends on whether the linear solve dominates the per-iteration wall time.

The total engineering effort is estimated at **medium-to-high**. The benefit is most justified for long-running trajectories with large local windows or in memory-constrained deployments. For the current typical LBA window sizes in ORB-SLAM3 (10–20 keyframes), the speedup may be modest given the COLAMD ordering already approximates the Schur complement structure.
