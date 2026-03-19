# SIFT Support Design (Tracking + Mapping, no LC/Reloc/Merge)

## 1. Summary
Your repository already contains an `SIFTFeatureExtractor` that produces `CV_32F` SIFT descriptors and is wired into `Tracking` via `FeatureExtractor.type: "SIFT"`.

In your current architecture, you have removed **loop closure**, **relocalization**, and **map merge**. That simplifies what “end-to-end” means: **tracking + local mapping** must work with SIFT without any binary-descriptor assumptions.

Decision (per user):
- When using SIFT features, **disable BoW-based reference keyframe tracking** (`TrackReferenceKeyFrameWithBoW()` / `ORBmatcher::SearchByBoW()`).

This document analyzes the current call graph where SIFT fails, then proposes a phased implementation plan:

1. **Phase A (required):** Make descriptor distance + thresholds work for both binary and float descriptors by introducing a **descriptor metric strategy** and replacing Hamming/binary thresholds in geometric matching code.
2. **Phase B (required):** Remove / gate the remaining **BoW-dependent calls** in the tracking/mapping pipeline that are no longer desired in SIFT mode.

The design targets minimal disruption: geometry code remains the same; only descriptor handling, thresholds, and BoW-dependent paths change.

## 2. Scope
In scope:
- Ensure `FeatureExtractor.type: "SIFT"` runs through the full ORB-SLAM3 pipeline.
- Ensure pose tracking and local mapping work without BoW assumptions.
  - Specifically: reference-keyframe tracking must work **without** BoW in SIFT mode.

Out of scope for this design doc:
- Loop closure / relocalization / merge (these components are removed in this repo).
- Implementing a float-descriptor vocabulary backend (e.g., DBoW3-like) from scratch.

## 3. Current State (What Works)
- `SIFTFeatureExtractor` exists and is already instantiated in `Tracking::loadFromSettings()` when `FeatureExtractor.type == "SIFT"`.
- `SIFTFeatureExtractor` builds its own scale space via `cv::SIFT::detect()`, groups keypoints into SLAM pyramid levels, and computes descriptors using `cv::SIFT::compute()`.
- Descriptors produced by `SIFTFeatureExtractor` are float matrices with:
  - descriptor rows = number of keypoints for that level
  - descriptor columns = 128
  - matrix type = `CV_32FC1`

Relevant files:
- `src/ORB_SLAM3/src/feature_extractor/SIFTFeatureExtractor.cc`
- `src/ORB_SLAM3/include/feature_extractor/SIFTFeatureExtractor.h`
- `src/ORB_SLAM3/src/Tracking.cc` (SIFT instantiation)

## 4. What Prevents End-to-End SIFT
SIFT fails end-to-end because several remaining ORB-SLAM3 components assume ORB descriptors:

### 4.1 Hamming distance is hard-coded
`ORBmatcher::DescriptorDistance(const cv::Mat&, const cv::Mat&)` implements Hamming distance using popcount on `int32_t` words.

This is binary-descriptor-specific and is incompatible with `CV_32F` descriptors.

Relevant file:
- `src/ORB_SLAM3/src/ORBmatcher.cc` (`ORBmatcher::DescriptorDistance`)

### 4.2 ORB thresholds are used with binary-distance scale
ORBmatcher uses constants:
- `ORBmatcher::TH_LOW = 50`
- `ORBmatcher::TH_HIGH = 100`

These thresholds are meaningful for Hamming distance. They are not correct for float descriptor L2 distances.

Relevant file:
- `src/ORB_SLAM3/src/ORBmatcher.cc` (TH_LOW/TH_HIGH and acceptance logic)

### 4.3 Other descriptor users also call the Hamming path
Even if `ORBmatcher` is fixed, SIFT still breaks if other code paths compute descriptor distances via `ORBmatcher::DescriptorDistance()` and/or compare to `TH_*`.

Examples:
- Stereo matching uses `ORBmatcher::DescriptorDistance(dL, dR)` and `ORBmatcher::TH_*` thresholds.
  - `src/ORB_SLAM3/src/Frame.cc` (`Frame::ComputeStereoMatches()`)
- MapPoint distinctive descriptor selection uses `ORBmatcher::DescriptorDistance(...)`.
  - `src/ORB_SLAM3/src/MapPoint.cc` (`MapPoint::ComputeDistinctiveDescriptors()`)

### 4.4 BoW is still present in code (but should be gated for SIFT)
Even though you removed loop closure / relocalization / merge, the code still contains BoW computation and BoW-based matching calls.

The system loads `ORBVocabulary`, which is defined as:
- `DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>`

This is designed for ORB binary descriptors. For SIFT, `Frame::ComputeBoW()` and `KeyFrame::ComputeBoW()` still call `mpORBvocabulary->transform(...)` with SIFT float descriptors.

In practice, this causes either incorrect quantization or an invalid assumption inside DBoW2’s binary descriptor pipeline.

Also, reference-keyframe tracking currently calls `TrackReferenceKeyFrameWithBoW()` (BoW-based). This must be disabled/replaced in SIFT mode:
- `src/ORB_SLAM3/src/Tracking.cc`

Local mapping currently calls `ComputeBoW()` on each new keyframe during `ProcessNewKeyFrame()`. Since BoW is not required in your stripped pipeline, this should be skipped (at least for SIFT mode, and optionally for all modes):
- `src/ORB_SLAM3/src/LocalMapping.cc`

Relevant files:
- `src/ORB_SLAM3/include/ORBVocabulary.h`
- `src/ORB_SLAM3/src/System.cc` (loads ORBVocabulary unconditionally)
- `src/ORB_SLAM3/src/Frame.cc` (`Frame::ComputeBoW()`)
- `src/ORB_SLAM3/src/Tracking.cc` (`TrackReferenceKeyFrameWithBoW`)
- `src/ORB_SLAM3/src/LocalMapping.cc` (calls `KeyFrame::ComputeBoW()` during KF insertion)
- `src/ORB_SLAM3/src/Converter.cc` (`Converter::toDescriptorVector` only slices rows; no dtype conversion)

## 5. Requirements
For “end-to-end SIFT” mode, the system must:

1. Run without descriptor-type crashes (float matrices must never be interpreted as bitsets).
2. Use a correct distance metric (L2 for SIFT) consistently across all descriptor matching uses required by:
   - pose tracking (reference/keyframe matching and motion model projection matching)
   - stereo matching (if stereo mode is used)
   - MapPoint distinctive descriptor selection (used later for matching)
3. In SIFT mode, reference-keyframe tracking must **not** depend on BoW (`SearchByBoW`).
4. Gate BoW computation so SIFT float descriptors are never fed into the binary ORB vocabulary.
5. Keep existing ORB/GridORB modes behavior unchanged (backward compatibility).

## 6. Proposed Implementation

### 6.1 Phase A (Required): Descriptor metric strategy
Goal: remove binary assumptions from geometric matching code.

Approach:
1. Add a descriptor-metric abstraction that provides:
   - `distance(a,b)` where `a,b` are descriptor rows (`cv::Mat` of one keypoint descriptor)
   - `thLow()` / `thHigh()` equivalents to replace ORBmatcher’s Hamming thresholds
2. Select the metric based on the active feature extractor:
   - For `DescriptorType::BINARY`: use Hamming/popcount
   - For `DescriptorType::FLOAT32`: use L2 distance (or squared L2 consistently)
3. Replace all uses in the matcher and other direct descriptor-distance call sites:
   - `DescriptorDistance(...)` => metric->distance(...)
   - comparisons to `ORBmatcher::TH_LOW/TH_HIGH` => metric->thLow()/metric->thHigh()

Why metric injection instead of duplicating matchers:
- ORB-SLAM3 matcher contains extensive geometry logic (projection search, fusion, rotation histograms).
- The only feature-type-specific part is descriptor distance + thresholds.

Files to modify (typical list):
- `src/ORB_SLAM3/src/ORBmatcher.cc` and `src/ORB_SLAM3/include/ORBmatcher.h`
- `src/ORB_SLAM3/src/Frame.cc` (`ComputeStereoMatches` needs L2 and float thresholds)
- `src/ORB_SLAM3/src/MapPoint.cc` (`ComputeDistinctiveDescriptors` needs metric distance)

Calibration requirement:
- L2 thresholds are dataset-dependent. Provide defaults but expect tuning:
  - simplest start: empirical thresholds and/or normalize SIFT descriptors consistently

### 6.2 Phase B (Required): Disable BoW ref-KF tracking for SIFT
Goal: keep SIFT mode functional without BoW-based reference tracking.

Constraint:
- Current `ORBVocabulary` and `DBoW2` are wired for binary ORB descriptors.
- Implementing float vocabulary is a separate backend project.

Recommended approach in this repo (SIFT mode):
- When `FeatureExtractor.type == "SIFT"`:
  1. **Replace reference-keyframe tracking that currently uses BoW**
     - Replace `Tracking::TrackReferenceKeyFrameWithBoW()` with a non-BoW method.
     - Recommended implementation:
       - **Projection-only matching (preferred):**
         - Use `ORBmatcher::SearchByProjection(...)` variants (with L2 metric) to match `mpReferenceKF`’s map points into `mCurrentFrame`.
         - Continue using the existing pose optimization + outlier rejection logic.
     - Fallback implementation:
       - **Direct descriptor matching:**
         - Run L2 KNN matching between `mCurrentFrame.mDescriptors` and `mpReferenceKF->mDescriptors`, apply ratio test, then map matches to MapPoints via KeyFrame observation lists.
  2. **Skip BoW computation for SIFT keyframes**
     - `LocalMapping::ProcessNewKeyFrame()` currently calls `KeyFrame::ComputeBoW()` unconditionally.
     - In SIFT mode, this should be a no-op (or skipped at the call site).
  3. **(Optional) Make ORB vocabulary optional**
     - `System` currently loads `ORBVocabulary` unconditionally.
     - If BoW is no longer used anywhere in your stripped pipeline, allow constructing `System` without loading a vocabulary file (or accept an empty/unused vocabulary pointer).

Where to implement the gating:
- `src/ORB_SLAM3/src/Tracking.cc`
- `src/ORB_SLAM3/src/LocalMapping.cc`
- `src/ORB_SLAM3/src/System.cc` (make vocabulary optional or unused)

### 6.3 Compatibility guarantees
- Default metric for ORB mode remains identical to current Hamming + current thresholds.
- BoW behavior remains unchanged for ORB/GridORB.
- SIFT mode uses:
  - L2 metric in all descriptor comparisons
  - non-BoW reference-keyframe tracking

## 7. Concrete Change List (High Level)
This is intentionally a checklist, not the implementation.

Phase A:
1. Add metric abstraction (`DescriptorMetric`) with Hamming and L2 implementations.
2. Update `ORBmatcher` to use injected metric instead of binary-only Hamming.
3. Update stereo matching and MapPoint distinctive descriptor computation similarly.

Phase B:
1. Replace `Tracking::TrackReferenceKeyFrameWithBoW()` with a non-BoW tracking path in SIFT mode.
2. Skip `ComputeBoW()` in local mapping for SIFT mode (and optionally stop computing BoW entirely if unused).
3. (Optional) Make `ORBVocabulary` optional if it is not used by your stripped pipeline.

## 8. Risks and Tradeoffs
1. Threshold tuning for L2:
   - If L2 thresholds are too strict, pose estimation will lose matches.
   - If too loose, geometric validation might reject more often, reducing loop closure frequency.
2. Removing BoW coupling:
   - If `TrackReferenceKeyFrameWithBoW()` is replaced with projection-only matching, behavior may differ from BoW matching and needs threshold calibration.
3. MapPoint distinctiveness:
   - MapPoint stores a single “best/most representative” descriptor cloned from observed descriptors.
   - The selection logic must use the correct metric to remain meaningful in SIFT mode.

## 9. Verification Plan

### 9.1 Functional tests (must pass)
- `mono_sift.yaml`
  - initialization succeeds
  - tracking doesn’t crash during normal operation
- `stereo_sift.yaml` (if stereo is used)
  - stereo init succeeds
  - `Frame::ComputeStereoMatches()` produces reasonable depths (no descriptor-distance crash)

### 9.2 Behavioral tests (regression checks)
- ORB configs (`mono_orb.yaml`, `stereo_gridorb.yaml`) produce results comparable to baseline:
  - no changes in ORB match acceptance behavior
  - no performance cliffs in ORB matcher

### 9.3 Instrumentation (recommended)
- Log distance metric type in SIFT mode.
- Log match counts:
  - best/second best distances for L2 ratio test (stats only, not per-descriptor dumps)
  - number of reference-keyframe matches before/after pose optimization

## 10. Key Open Questions
1. Should SIFT matching use raw L2 or squared L2 (and be consistent everywhere)?
2. Should SIFT descriptors be normalized before computing L2 (if not already normalized by OpenCV)?
3. For the non-BoW reference-keyframe tracking path, do you prefer:
   - projection-only matching (minimal new code, uses geometry), or
   - descriptor-only matching (more direct, potentially heavier)?
4. Should `System` stop loading `ORBVocabulary` entirely once BoW is removed from tracking/mapping?
5. Should ORB mode keep BoW (current behavior) while SIFT mode skips it?

## 11. Notes on Repository State
At the time of writing, `System` no longer constructs a `LoopClosing` thread, and there is no `KeyFrameDatabase` wiring in `System`. However, BoW-based code paths still exist in `Tracking` and `LocalMapping` and must be gated or removed for SIFT mode.

   - raw L2, or
   - squared L2,
   - and should SIFT descriptors be normalized before computing L2?

