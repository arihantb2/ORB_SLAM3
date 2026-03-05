# ORB_SLAM3 Nondeterminism Analysis

This document identifies sources of nondeterminism in the ORB_SLAM3 codebase that can affect reproducibility across runs.

---

## 1. Random Sampling in RANSAC Solvers (HIGH)

**Category:** Random number generation
**Impact:** Different geometric models produced per run

All three RANSAC-based solvers use `static thread_local std::mt19937 gen(0u)` — a fixed-seed generator that is reset on each new thread. While the seed is constant (`0u`), different thread scheduling means different threads' generators can diverge, yielding different minimal sets.

| File | Line | Details |
|------|------|---------|
| `src/Sim3Solver.cc` | 173, 256 | Two `iterate()` overloads each declare their own `gen(0u)` |
| `src/TwoViewReconstruction.cc` | 82 | `Reconstruct()` RANSAC for H/F selection |
| `src/MLPnPsolver.cpp` | 125 | `find()` method RANSAC loop |

**Root cause:** Each call to `iterate()` samples random point indices via `std::uniform_int_distribution`. With `static thread_local` storage, generator state persists across calls within a thread but differs across threads.

---

## 2. Parallel Thread Scheduling (HIGH)

**Category:** Threading / race on score comparison
**Impact:** Different algorithm branch selected depending on thread timing

### A. Homography vs. Fundamental Matrix Selection — `TwoViewReconstruction.cc:107-114`

```cpp
std::thread threadH(&TwoViewReconstruction::FindHomography, ...);
std::thread threadF(&TwoViewReconstruction::FindFundamental, ...);
threadH.join();
threadF.join();
// Then select based on score ratio:  SH / (SH + SF)
```

Both matrices are computed concurrently. When scores `SH` and `SF` are near-equal, floating-point rounding from different execution orders can flip the selected model.

### B. Global Bundle Adjustment — `src/LoopClosing.cc:1653-1666`

```cpp
mpThreadGBA = new std::thread(&LoopClosing::RunGlobalBundleAdjustment, ...);
```

GBA convergence depends on optimizer iteration order, which is thread-scheduling-dependent.

---

## 3. Floating-Point Non-Determinism (MEDIUM)

**Category:** Numerical precision in linear algebra
**Impact:** Sign ambiguity and index selection in eigensolvers / SVD

### A. Eigenvalue Solver — `src/Sim3Solver.cc:374-388`

```cpp
Eigen::EigenSolver<Eigen::Matrix4f> eigSolver;
eigSolver.compute(N);
Eigen::Vector4f eval = eigSolver.eigenvalues().real();
int maxIndex;
eval.maxCoeff(&maxIndex);  // undefined behavior when values are equal
```

When two eigenvalues are nearly equal, `maxCoeff()` may return different indices across runs.

### B. SVD Decomposition — `src/TwoViewReconstruction.cc:271, 303, 634`

```cpp
Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);
```

When singular values are close, eigenvector sign ambiguity propagates into the recovered H/F matrices.

### C. Score-Based Model Selection — `src/TwoViewReconstruction.cc:177-232`

Homography/Fundamental matrix selection uses floating-point score comparison. Near-equal scores can be flipped by floating-point rounding from parallel execution.

---

## 4. Time-Dependent Optimization Scheduling (MEDIUM)

**Category:** Wall-clock time controlling algorithmic decisions
**Impact:** Different optimization histories → different final map state

`src/LocalMapping.cc:124-138`:

```cpp
constexpr double OPTIMIZE_EVERY_T_SECONDS = 5.0;
constexpr double TIME_EPSILON = 0.1;
const auto time_since_last_optimize = mpCurrentKeyFrame->mTimeStamp - prevOptimizedKFTimestamp;
if (!mbInertial && time_since_last_optimize < OPTIMIZE_EVERY_T_SECONDS - TIME_EPSILON)
{
    b_doLBA = false;
}
```

LBA is skipped when sensor timestamps are too close together. Replaying the same sequence at a different data rate changes how many LBA iterations are performed.

---

## 5. Pointer-Based Map/Set Ordering (LOW–MEDIUM)

**Category:** Memory address-dependent iteration
**Impact:** With ASLR enabled, iteration order varies across process invocations

| File | Line | Type | Usage |
|------|------|------|-------|
| `include/MapPoint.h` | 143 | `std::map<KeyFrame*, tuple<int,int>>` | Observation storage |
| `include/Map.h` | 112 | `std::set<KeyFrame*>` | All keyframes |
| `src/Map.cc` | 292–299 | Iterates `mspKeyFrames` directly | Change detection |
| `src/Optimizer.cc` | 149, 596, 1057, 1186 | Iterates observation maps | Graph edge construction |
| `src/KeyFrame.cc` | 537–540 | Iterates observation map | Covisibility update |

`std::map` and `std::set` are sorted by pointer value. Without ASLR (`-pie` disabled) this is deterministic per binary, but with ASLR the order changes every run.

**Note:** `src/Map.cc:147-149` correctly converts to vector and sorts by ID — this is the right pattern and should be applied elsewhere.

---

## Summary

| Severity | Category | Primary Files | Lines |
|----------|----------|---------------|-------|
| HIGH | RANSAC random sampling | `Sim3Solver.cc`, `TwoViewReconstruction.cc`, `MLPnPsolver.cpp` | 173, 256, 82, 125 |
| HIGH | Parallel thread scheduling | `TwoViewReconstruction.cc`, `LoopClosing.cc` | 107–114, 1653–1666 |
| MEDIUM | Floating-point (eigen/SVD) | `Sim3Solver.cc`, `TwoViewReconstruction.cc` | 374–388, 271, 303 |
| MEDIUM | Time-dependent LBA | `LocalMapping.cc` | 124–138 |
| LOW–MED | Pointer-keyed maps/sets | `MapPoint.h`, `Map.h`, `Optimizer.cc`, `KeyFrame.cc` | 143, 112, various |

---

## Recommendations

1. **RANSAC seeds**: Derive seed from input data hash (e.g., sum of keypoint coordinates) instead of hardcoded `0u`.
2. **Sequential H/F computation**: Run `FindHomography` and `FindFundamental` sequentially when reproducibility is required, or ensure tie-breaking is deterministic.
3. **Eigensolver tie-breaking**: After `maxCoeff(&maxIndex)`, add a secondary sort criterion when eigenvalues differ by less than a threshold.
4. **Frame-count-based LBA scheduling**: Replace timestamp delta with keyframe-count delta to decouple from data rate.
5. **Stable map iteration**: Wherever observation or keyframe maps are iterated and order affects output, convert to `std::vector` and sort by ID before processing (as already done in `Map.cc:147-149`).
