# ORB_SLAM3 Non-Determinism: Mitigation Design

This document catalogues every identified source of non-determinism in ORB_SLAM3, explains the root cause, and prescribes a concrete fix. Issues are ordered by thread priority: **Tracking thread first, Mapping thread second**.

See also: [`NONDETERMINISM_ANALYSIS.md`](NONDETERMINISM_ANALYSIS.md) for the original high-level survey.

---

## Overview

Non-determinism means that running the same sensor sequence twice produces different maps, trajectories, or initialization outcomes. This makes:

- Regression testing unreliable (flaky pass/fail)
- Bug reproduction difficult (symptom changes between runs)
- Quantitative evaluation noisy (ATE/RPE vary without code changes)

The goal of this document is **same data → same result**, every run. We explicitly do *not* require that different replay rates produce the same result (see [Out of Scope](#out-of-scope)).

---

## Issue Catalogue

| ID | Thread | File:Line | Category | Severity | Status |
|----|--------|-----------|----------|----------|--------|
| T1 | Tracking | `TwoViewReconstruction.cc:82`, `MLPnPsolver.cpp:125` | RNG state carry-over | HIGH | Fixed |
| T2 | Tracking | `Tracking.cc:1988–2035` | Pointer-ordered map iteration | MEDIUM | Fixed |
| T3 | Tracking | `Tracking.cc:2065–2078` | Pointer-ordered set, break on first | MEDIUM | Fixed |
| T4 | Tracking | `Map.cc:147–156` | Pointer-ordered set → vector | LOW-MEDIUM | Fixed |
| T5 | Tracking | `TwoViewReconstruction.cc:107–134` | Parallel H/F score tie (residual) | LOW | Resolved by T1 |
| M1 | Mapping | `Optimizer.cc` (3 sites) | Pointer-ordered observation map | MEDIUM | Fixed |
| M2 | Mapping | `KeyFrame.cc:292` | Equal-weight covisibility tie-break | LOW | Fixed |

---

## Tracking Thread

### T1 — RANSAC RNG state carries over between calls

**Severity:** HIGH

**Files:**
- [`src/TwoViewReconstruction.cc:82`](../src/TwoViewReconstruction.cc)
- [`src/MLPnPsolver.cpp:125`](../src/MLPnPsolver.cpp)

**Root cause:**

```cpp
static thread_local std::mt19937 gen(0u);
```

`static` means the generator is initialised once per thread with seed `0` and then **retains state across calls**. The number of RANSAC-eligible frames processed before any given call varies per run (tracking failures, keyframe selection, IMU vs. stereo paths). So `gen` is at a different point in its sequence each time initialisation is attempted, producing different random minimal sets.

The `thread_local` qualifier compounds this: if the SLAM system spawns helper threads (e.g., for parallel H/F computation — see T5), each thread starts fresh at seed `0`, but the accumulation of calls is still order-dependent.

**Effect:** Different RANSAC minimal sets → different estimated H/F/Sim3 → different map initialisation, relocalisation, or loop closure outcome per run.

**Fix:** Replace the `static thread_local` generator with a **local generator seeded from the input data**. A stable, cheap seed is the XOR-sum of the quantized keypoint coordinates.

*TwoViewReconstruction.cc* — replace lines 82–99:

```cpp
// BEFORE
static thread_local std::mt19937 gen(0u);

for (int it = 0; it < mMaxIterations; it++) {
    vAvailableIndices = vAllIndices;
    for (size_t j = 0; j < 8; j++) {
        std::uniform_int_distribution<int> dis(0, (int)vAvailableIndices.size() - 1);
        int randi = dis(gen);
        ...
    }
}

// AFTER
uint32_t seed = 0;
for (const auto& m : mvMatches12) {
    seed ^= static_cast<uint32_t>(mvKeys1[m.first].pt.x * 100 + 1)
          ^ static_cast<uint32_t>(mvKeys1[m.first].pt.y * 100 + 1) * 2654435761u
          ^ static_cast<uint32_t>(mvKeys2[m.second].pt.x * 100 + 1) * 40503u
          ^ static_cast<uint32_t>(mvKeys2[m.second].pt.y * 100 + 1) * 134775813u;
}
std::mt19937 gen(seed);   // local — no state carried between calls

for (int it = 0; it < mMaxIterations; it++) {
    vAvailableIndices = vAllIndices;
    for (size_t j = 0; j < 8; j++) {
        std::uniform_int_distribution<int> dis(0, (int)vAvailableIndices.size() - 1);
        int randi = dis(gen);
        ...
    }
}
```

*MLPnPsolver.cpp* — replace line 125, seed from 3D point coordinates:

```cpp
// BEFORE
static thread_local std::mt19937 gen(0u);

// AFTER (at top of find(), before the while loop)
uint32_t seed = 0;
for (size_t i = 0; i < mvP3Dw.size(); i++) {
    seed ^= static_cast<uint32_t>(mvP3Dw[i][0] * 1000 + 1) * 2654435761u
          ^ static_cast<uint32_t>(mvP3Dw[i][1] * 1000 + 1) * 40503u
          ^ static_cast<uint32_t>(mvP3Dw[i][2] * 1000 + 1) * 134775813u;
}
std::mt19937 gen(seed);
```

**Why this is safe:** The quantized coordinates are fixed for a given set of matches regardless of how many prior RANSAC calls have been made. Equal inputs → equal seed → equal random sequence → equal RANSAC sets every run.

---

### T2 — `UpdateLocalKeyFrames`: pointer-ordered map determines `pKFmax` and local KF list order

**Severity:** MEDIUM

**File:** [`src/Tracking.cc:1988–2035`](../src/Tracking.cc)

**Root cause:**

```cpp
std::map<KeyFrame*, int> keyframeCounter;   // ordered by pointer address
...
for (auto it = keyframeCounter.begin(); it != keyframeCounter.end(); it++) {
    if (it->second > max) { max = it->second; pKFmax = it->first; }
    mvpLocalKeyFrames.push_back(it->first);
}
```

`std::map<KeyFrame*, int>` sorts by pointer value. With ASLR enabled, `KeyFrame` objects land at different virtual addresses each run, so iteration order changes. Two effects:

1. **`pKFmax` selection:** When two keyframes have equal vote counts, the one with the lower pointer address wins. This changes `mpReferenceKF` (used in subsequent pose estimation).
2. **`mvpLocalKeyFrames` ordering:** `UpdateLocalPoints` iterates `mvpLocalKeyFrames` in reverse; a different order means different map points are deduplicated first.

**Fix:** After building `keyframeCounter`, sort into a vector by `(votes DESC, mnId ASC)` — identical to the pattern used in `KeyFrame::UpdateBestCovisibles()` at [`KeyFrame.cc:284–292`](../src/KeyFrame.cc):

```cpp
// BEFORE
int max = 0;
KeyFrame* pKFmax = nullptr;
mvpLocalKeyFrames.clear();
mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

for (auto it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++) {
    KeyFrame* pKF = it->first;
    if (pKF->isBad()) continue;
    if (it->second > max) { max = it->second; pKFmax = pKF; }
    mvpLocalKeyFrames.push_back(pKF);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
}

// AFTER
// 1. Sort by (votes DESC, mnId ASC) for deterministic ordering.
std::vector<std::pair<int, KeyFrame*>> vSortedKFs;
vSortedKFs.reserve(keyframeCounter.size());
for (auto& kv : keyframeCounter)
    vSortedKFs.emplace_back(kv.second, kv.first);
std::sort(vSortedKFs.begin(), vSortedKFs.end(),
    [](const std::pair<int,KeyFrame*>& a, const std::pair<int,KeyFrame*>& b) {
        if (a.first != b.first) return a.first > b.first;   // votes DESC
        return a.second->mnId < b.second->mnId;              // mnId ASC tie-break
    });

int max = 0;
KeyFrame* pKFmax = nullptr;
mvpLocalKeyFrames.clear();
mvpLocalKeyFrames.reserve(3 * vSortedKFs.size());

for (auto& [votes, pKF] : vSortedKFs) {
    if (pKF->isBad()) continue;
    if (votes > max) { max = votes; pKFmax = pKF; }   // first entry wins — deterministic
    mvpLocalKeyFrames.push_back(pKF);
    pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
}
```

---

### T3 — `GetChilds()` returns pointer-ordered set; `break` picks first child by pointer

**Severity:** MEDIUM

**Files:**
- [`src/Tracking.cc:2065–2078`](../src/Tracking.cc) (call site)
- [`src/KeyFrame.cc:603–607`](../src/KeyFrame.cc) (return type)

**Root cause:**

```cpp
// KeyFrame.cc:603
std::set<KeyFrame*> KeyFrame::GetChilds() {
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mspChildrens;   // std::set<KeyFrame*> — ordered by pointer
}

// Tracking.cc:2065
const std::set<KeyFrame*> spChilds = pKF->GetChilds();
for (auto sit = spChilds.begin(); sit != spChilds.end(); sit++) {
    KeyFrame* pChildKF = *sit;
    if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
            mvpLocalKeyFrames.push_back(pChildKF);
            pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            break;   // <-- takes the first non-bad child in pointer order
        }
    }
}
```

With ASLR, `mspChildrens` iterates in a different pointer order each run, so `break` selects a different child keyframe, adding a different KF to `mvpLocalKeyFrames`.

**Fix (preferred — localised to call site, no API change):** Sort children by `mnId` before the loop:

```cpp
// BEFORE
const std::set<KeyFrame*> spChilds = pKF->GetChilds();
for (auto sit = spChilds.begin(); sit != spChilds.end(); sit++) {
    ...
    break;
}

// AFTER
std::vector<KeyFrame*> vChilds;
{
    const std::set<KeyFrame*> spChilds = pKF->GetChilds();
    vChilds.assign(spChilds.begin(), spChilds.end());
}
std::sort(vChilds.begin(), vChilds.end(),
    [](KeyFrame* a, KeyFrame* b) { return a->mnId < b->mnId; });
for (KeyFrame* pChildKF : vChilds) {
    if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
            mvpLocalKeyFrames.push_back(pChildKF);
            pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            break;
        }
    }
}
```

**Alternative (API change):** Change `KeyFrame::GetChilds()` to return `std::vector<KeyFrame*>` sorted by `mnId`. This fixes any other call sites automatically but changes the return type across the codebase.

---

### T4 — `Map::GetAllKeyFrames` / `GetAllMapPoints` return pointer-ordered vectors

**Severity:** LOW-MEDIUM

**File:** [`src/Map.cc:147–156`](../src/Map.cc)

**Root cause:**

```cpp
std::vector<KeyFrame*> Map::GetAllKeyFrames() {
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
}

std::vector<MapPoint*> Map::GetAllMapPoints() {
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<MapPoint*>(mspMapPoints.begin(), mspMapPoints.end());
}
```

Both convert a `std::set<T*>` (pointer-ordered) to vector without sorting. Callers that process the vector in sequence (e.g., map save, evaluation, visualisation) get non-deterministic ordering.

Note: a correctly sorted variant already exists at `Map.cc:~115`:
```cpp
std::vector<KeyFrame*> vpKFs(mspKeyFrames.begin(), mspKeyFrames.end());
sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
```

**Fix:** Apply the same sort inside both public functions:

```cpp
// AFTER
std::vector<KeyFrame*> Map::GetAllKeyFrames() {
    std::unique_lock<std::mutex> lock(mMutexMap);
    std::vector<KeyFrame*> v(mspKeyFrames.begin(), mspKeyFrames.end());
    std::sort(v.begin(), v.end(), KeyFrame::lId);
    return v;
}

std::vector<MapPoint*> Map::GetAllMapPoints() {
    std::unique_lock<std::mutex> lock(mMutexMap);
    std::vector<MapPoint*> v(mspMapPoints.begin(), mspMapPoints.end());
    std::sort(v.begin(), v.end(),
        [](MapPoint* a, MapPoint* b) { return a->mnId < b->mnId; });
    return v;
}
```

---

### T5 — Parallel H/F score tie (residual, blocked on T1)

**Severity:** LOW (residual after T1 is fixed)

**File:** [`src/TwoViewReconstruction.cc:107–134`](../src/TwoViewReconstruction.cc)

**Root cause:** The two threads (`FindHomography`, `FindFundamental`) operate on disjoint output variables and are fully joined before `SH`/`SF` are read — there is no data race. The non-determinism here is entirely downstream of T1: different RANSAC sets → different SH/SF. Once T1 is fixed, SH and SF are deterministic.

A residual risk remains only in near-degenerate scenes where `SH ≈ SF ≈ 0.50`. The current threshold `RH > 0.50` already provides a deterministic tie-break (strict inequality always resolves to F).

**No code change required** after T1 is fixed. If extra hardening is ever needed, a note-to-self:

```cpp
// Optional secondary tie-break if |RH - 0.50| < epsilon:
// prefer F (fundamental matrix) as the less degenerate assumption.
constexpr float kTieEpsilon = 0.005f;
if (RH > 0.50f + kTieEpsilon)
    return ReconstructH(...);
else
    return ReconstructF(...);
```

---

## Mapping Thread

### M1 — Optimizer observation map iteration in pointer order

**Severity:** MEDIUM

**File:** [`src/Optimizer.cc`](../src/Optimizer.cc) — lines 149, 596, 1057, 1186 (and similar patterns throughout)

**Root cause:**

```cpp
const std::map<KeyFrame*, std::tuple<int,int>> observations = pMP->GetObservations();
for (const auto& mit : observations) {   // iterates in pointer order
    KeyFrame* pKFi = mit.first;
    ...
    // add edge to g2o / GTSAM graph
}
```

`mObservations` is `std::map<KeyFrame*, tuple<int,int>>` (declared in [`include/MapPoint.h:143`](../include/MapPoint.h)), sorted by pointer address. The graph edges are added in this order. While a correctly-implemented optimizer produces the same minimum given the same graph, different edge/vertex ordering changes:

- Internal data structure layout (cache effects → different numerical path through sparse Cholesky)
- When two edges compete for the same variable, accumulation order affects floating-point rounding

**Fix:** Before each observation iteration that feeds into graph construction, sort into a vector by KF `mnId`:

```cpp
// Helper (can be a free function in Optimizer.cc or an inline lambda):
auto getSortedObs = [](const std::map<KeyFrame*, std::tuple<int,int>>& obs) {
    std::vector<std::pair<KeyFrame*, std::tuple<int,int>>> v(obs.begin(), obs.end());
    std::sort(v.begin(), v.end(),
        [](const auto& a, const auto& b) { return a.first->mnId < b.first->mnId; });
    return v;
};

// Usage (replaces the raw map loop):
for (const auto& [pKFi, tup] : getSortedObs(observations)) {
    ...
}
```

Apply this pattern at all four call sites (lines 149, 596, 1057, 1186) and any other `for (... : observations)` loops in `Optimizer.cc`.

---

### M2 — Equal-weight covisibility tie-break by pointer in `UpdateBestCovisibles`

**Severity:** LOW

**File:** [`src/KeyFrame.cc:292`](../src/KeyFrame.cc)

**Root cause:**

```cpp
// KeyFrame.cc:284–292
for (auto mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); mit++)
    vPairs.push_back({mit->second, mit->first});  // {weight, KF*}
sort(vPairs.begin(), vPairs.end());   // sorts by weight; equal weights → pointer order
```

`std::sort` with `std::pair` uses lexicographic comparison: first by weight, then by pointer. When two neighbors share the same covisibility count, their relative order in `mvpOrderedConnectedKeyFrames` depends on pointer value (ASLR-dependent).

`GetBestCovisibilityKeyFrames(N)` slices this list at index `N`. Near the cut-off, different runs may include/exclude different neighbors, affecting `CreateNewMapPoints` and `SearchInNeighbors` in the mapping thread.

**Fix:** Use an explicit comparator with `mnId` as the secondary key:

```cpp
// BEFORE
sort(vPairs.begin(), vPairs.end());

// AFTER
std::sort(vPairs.begin(), vPairs.end(),
    [](const std::pair<int,KeyFrame*>& a, const std::pair<int,KeyFrame*>& b) {
        if (a.first != b.first) return a.first < b.first;   // weight ASC (list is front-prepended)
        return a.second->mnId < b.second->mnId;              // mnId ASC tie-break
    });
```

---

## Out of Scope

The following are **intentional design choices** and are not to be changed:

| Item | Reason |
|------|--------|
| `LocalMapping.cc:181–196` — time-based LBA throttle | Intentional feature. Same data replayed at the same rate → same result. Different replay rates → different LBA count is acceptable and expected. |
| `LoopClosing.cc:1653–1666` — Global Bundle Adjustment thread | GBA is a background asynchronous operation. Its convergence varying with thread scheduling is acceptable for the current use case. |
| IMU pre-integration divergence across runs | Depends on sensor timestamps and system clock; addressed separately if needed. |
| RANSAC being probabilistic | The goal is reproducibility (same data → same result), not removing RANSAC. T1 achieves this by data-derived seeding. |

---

## Testing Strategy

1. **Identical-run test (T1):** Run the same rosbag twice. Assert that `TwoViewReconstruction::Reconstruct` selects the same model (H or F) both times — log the `RH` value and the model chosen on every initialisation attempt.

2. **Local KF set test (T2, T3):** Add a debug log of `mvpLocalKeyFrames` KF IDs (sorted by vector position, not value) after each `UpdateLocalKeyFrames()` call. Run twice; diff the logs.

3. **Reference KF stability test (T2):** Log `mpReferenceKF->mnId` per frame. Run twice; assert identical sequence.

4. **Sorted-output test (T4):** Call `GetAllKeyFrames()` and `GetAllMapPoints()` and assert the returned vectors are sorted by `mnId` (simple unit test, no full SLAM run needed).

5. **Optimizer edge-order test (M1):** Log the vertex IDs of edges added to the g2o graph in `BundleAdjustment`. Run twice; assert the edge list is identical.

6. **Covisibility stability test (M2):** Log `GetBestCovisibilityKeyFrames(10)` for a fixed reference KF after map build. Run twice; assert identical neighbour IDs and order.

7. **Regression:** Full pipeline tests in `test/` must pass unchanged after all fixes.
