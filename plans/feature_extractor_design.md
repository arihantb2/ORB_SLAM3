# FeatureExtractor Design — ORB-SLAM3

## Table of Contents
1. [Scope](#1-scope)
2. [Current Architecture](#2-current-architecture)
   - 2.1 [Data Flow](#21-data-flow)
   - 2.2 [ORBextractor — Feature Extraction Pipeline](#22-orbextractor--feature-extraction-pipeline)
   - 2.3 [Feature Grid — Frame's Spatial Index](#23-feature-grid--frames-spatial-index)
   - 2.4 [ORBmatcher — Dependency Surface](#24-orbmatcher--dependency-surface)
3. [FeatureExtractor Interface Design](#3-featureextractor-interface-design)
   - 3.1 [Supporting Types](#31-supporting-types)
   - 3.2 [FeatureGrid](#32-featuregrid)
   - 3.3 [FeatureExtractor Base Class](#33-featureextractor-base-class)
   - 3.4 [ORBFeatureExtractor Specialization](#34-orbfeatureextractor-specialization)
   - 3.5 [SIFTFeatureExtractor Specialization](#35-siftfeatureextractor-specialization)
4. [DescriptorMetric — Isolating the Matcher](#4-descriptormetric--isolating-the-matcher)
   - 4.1 [The Problem](#41-the-problem)
   - 4.2 [Options Considered](#42-options-considered)
   - 4.3 [Recommended Approach: DescriptorMetric Strategy](#43-recommended-approach-descriptormetric-strategy)
   - 4.4 [BoW Vocabulary Caveat](#44-bow-vocabulary-caveat)
5. [Frame After Refactoring](#5-frame-after-refactoring)
6. [Implementation Scope and File Map](#6-implementation-scope-and-file-map)
7. [Standalone Library](#7-standalone-library)
   - 7.1 [Dependency Audit](#71-dependency-audit)
   - 7.2 [Library Layout](#72-library-layout)
   - 7.3 [CMake Design](#73-cmake-design)
   - 7.4 [Consuming the Library in ORB-SLAM3](#74-consuming-the-library-in-orb-slam3)

---

## 1. Scope

### What this refactor does

This work isolates the feature extraction and spatial indexing logic from ORB-SLAM3's `Frame` and `ORBextractor` into a clean polymorphic `FeatureExtractor` hierarchy. The goals are:

1. **Replace** `ORBextractor` with an abstract `FeatureExtractor` base class, keeping ORB as one concrete implementation.
2. **Move the feature grid** (`mGrid`, `AssignFeaturesToGrid`, `GetFeaturesInArea`, `PosInGrid`) out of `Frame` and into `FeatureExtractor`, so it becomes part of the extraction contract.
3. **Enable SIFT (and other detectors)** with minimal per-type code: only the detection kernel, descriptor computation, and orientation assignment vary between types.
4. **Isolate the descriptor distance metric** from `ORBmatcher` so that SIFT's L2 distance can be used without duplicating the entire matching infrastructure.

### What this refactor does NOT do

- It does not change the stereo matching logic (`ComputeStereoMatches`), pose estimation, or the map.
- It does not change `ORBVocabulary` or `KeyFrameDatabase`; BoW vocabulary support for non-binary descriptors is noted as a limitation (see §4.4).
- It does not change `Tracking`, `LocalMapping`, or `LoopClosing` beyond the extractor pointer type and grid queries.

### In scope / out of scope summary

| Area | In Scope |
|---|---|
| `ORBextractor` → `FeatureExtractor` hierarchy | Yes |
| Feature grid abstracted into `FeatureGrid` | Yes |
| Grid assignment / query moved out of `Frame` | Yes |
| `ORBmatcher::DescriptorDistance` isolated | Yes |
| SIFT specialization (structure only) | Yes |
| BoW vocabulary re-training for SIFT | No |
| Changing map, optimizer, loop-closer | No |

---

## 2. Current Architecture

### 2.1 Data Flow

```
System::TrackStereo / TrackMonocular
    └─ Tracking::GrabImageStereo / GrabImageMonocular
            │
            │  Creates:  new Frame(image, ..., mpORBextractorLeft, ...)
            │
            ▼
        Frame constructor
            ├─ ExtractORB(left,  image) → (*mpORBextractorLeft)(im, ..., mvKeys, mDescriptors, vLapping)
            ├─ ExtractORB(right, image) → (*mpORBextractorRight)(im, ..., mvKeysRight, mDescriptorsRight, vLapping)
            │       [parallel std::thread for stereo]
            ├─ UndistortKeyPoints()    → mvKeysUn   [needs K, distCoef from camera]
            ├─ ComputeStereoMatches()  → mvDepth, mvuRight
            └─ AssignFeaturesToGrid() → mGrid[64][48]

        ORBmatcher reads Frame / KeyFrame:
            mvKeys, mvKeysUn, mDescriptors, mFeatVec,
            mvScaleFactors[level], mvLevelSigma2[level], mvInvLevelSigma2[level],
            GetFeaturesInArea(x, y, r, minLevel, maxLevel)
```

**Key types created per frame:**
- `mvKeys` — `std::vector<cv::KeyPoint>`, distorted, level-0 scale
- `mvKeysUn` — undistorted, same indexing
- `mDescriptors` — `cv::Mat` (CV_8UC1, N×32), one row per keypoint
- `mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS]` — `std::vector<size_t>` per cell, stores keypoint indices

---

### 2.2 ORBextractor — Feature Extraction Pipeline

**Header:** `include/ORBextractor.h`
**Implementation:** `src/ORBextractor.cc`

```cpp
ORBextractor(int nfeatures, float scaleFactor, int nlevels,
             int iniThFAST, int minThFAST);

int operator()(cv::InputArray image, cv::InputArray mask,
               std::vector<cv::KeyPoint>& keypoints,
               cv::OutputArray descriptors,
               std::vector<int>& vLappingArea);
```

**Pipeline executed by `operator()`:**

| Step | Method | What it does |
|---|---|---|
| 1 | `ComputePyramid(image)` | Builds `nlevels`-deep Gaussian pyramid with `EDGE_THRESHOLD=19` border padding (BORDER_REFLECT_101). Fills `mvImagePyramid`. |
| 2 | `ComputeKeyPointsOctTree(allKeypoints)` | For each level: divides into 35×35 cells, runs FAST at `iniThFAST` (fallback `minThFAST`), calls `DistributeOctTree()` to enforce spatial uniformity, computes IC-Angle orientation. |
| 3 | Gaussian blur per level | `GaussianBlur(workingMat, ..., Size(7,7), 2, 2)` before descriptor computation. |
| 4 | `computeDescriptors(workingMat, kps, descs, pattern)` | For each keypoint: rotates the 512-point BRIEF pattern by IC-Angle, compares intensity pairs, produces 32-byte (256-bit) rBRIEF descriptor row. |
| 5 | Scale coords + lapping reorder | Scales `kp.pt *= mvScaleFactor[level]` to level-0 coords. Separates features into mono vs. stereo-overlap regions, returns `monoIndex`. |

**`DistributeOctTree()` — QuadTree spatial distribution:**
Recursively subdivides the image into quadrants until each node has ≤1 keypoint or the target count is met. From each leaf, retains the single keypoint with highest FAST response. This replaces the naive grid-based distribution and produces more uniform spatial coverage.

**Scale pyramid metadata (exposed via getters, copied into Frame):**
```cpp
std::vector<float> mvScaleFactor;       // scaleFactor^level
std::vector<float> mvInvScaleFactor;
std::vector<float> mvLevelSigma2;       // scaleFactor^(2*level)
std::vector<float> mvInvLevelSigma2;
```

---

### 2.3 Feature Grid — Frame's Spatial Index

**Location:** `include/Frame.h` (grid definition), `src/Frame.cc` (assignment and query)

```cpp
// Constants (Frame.h lines 40-41)
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

// Member variables (Frame.h)
static float mfGridElementWidthInv;    // FRAME_GRID_COLS / (mnMaxX - mnMinX)
static float mfGridElementHeightInv;   // FRAME_GRID_ROWS / (mnMaxY - mnMinY)
std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];  // indices
```

**`AssignFeaturesToGrid()` — `src/Frame.cc` lines 354–379:**
```
for i in 0..N:
    PosInGrid(mvKeysUn[i]) → (posX, posY)
    mGrid[posX][posY].push_back(i)
```
Uses **undistorted** keypoints (`mvKeysUn`). Stores indices, not coordinates.

**`GetFeaturesInArea(x, y, r, minLevel, maxLevel)` — `src/Frame.cc` lines 611–687:**
Computes cell range covering the (x±r, y±r) rectangle, iterates those cells, filters by `kp.octave` and Euclidean distance. Returns a vector of keypoint indices.

**`PosInGrid(kp, posX, posY)` — `src/Frame.cc` lines 689–701:**
`posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv)`. Returns false if out of bounds.

**This grid is the primary coupling point for `ORBmatcher`** — all projection-based matchers call `GetFeaturesInArea()`.

---

### 2.4 ORBmatcher — Dependency Surface

**Header:** `include/ORBmatcher.h`
**Implementation:** `src/ORBmatcher.cc`

ORBmatcher never touches `ORBextractor` directly. Its feature-related dependencies are:

| What it accesses | Where | Why |
|---|---|---|
| `mDescriptors.row(i)` | Frame / KeyFrame | Descriptor comparison |
| `DescriptorDistance(a, b)` | static method | Hamming distance, 32-byte binary |
| `TH_LOW = 50`, `TH_HIGH = 100` | class constants | Thresholds for 0–256 Hamming range |
| `mvKeys`, `mvKeysUn`, `mvKeysRight` | Frame / KeyFrame | Keypoint positions and `.angle` for rotation histogram |
| `kp.octave` | cv::KeyPoint field | Level-based search radius and filtering |
| `mvScaleFactors[level]` | Frame / KeyFrame | `radius = th * mvScaleFactors[predictedLevel]` |
| `mvLevelSigma2[level]` | Frame / KeyFrame | Epipolar constraint covariance weight |
| `mvInvLevelSigma2[level]` | Frame / KeyFrame | Reprojection error weight in Fuse |
| `mFeatVec` | Frame / KeyFrame | DBoW2 FeatureVector for vocab-guided matching |
| `GetFeaturesInArea(x, y, r, l0, l1)` | Frame / KeyFrame | Grid-accelerated candidate lookup |

**`DescriptorDistance()` implementation (`src/ORBmatcher.cc` lines 1906–1922):**
```cpp
int ORBmatcher::DescriptorDistance(const cv::Mat& a, const cv::Mat& b) {
    const int* pa = a.ptr<int32_t>();
    const int* pb = b.ptr<int32_t>();
    int dist = 0;
    for (int i = 0; i < 8; i++, pa++, pb++) {  // 8 × int32 = 32 bytes
        unsigned int v = *pa ^ *pb;
        // popcount via parallel bit-summation
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }
    return dist;  // range: 0..256
}
```

This hard-codes binary, 32-byte (256-bit), Hamming distance. It is the only ORB-specific code in `ORBmatcher`.

---

## 3. FeatureExtractor Interface Design

The design uses the **Template Method** pattern: `extract()` is a non-virtual public method that orchestrates the pipeline and calls virtual hooks at the right moments. Subclasses override only what differs.

### 3.1 Supporting Types

```cpp
// include/FeatureTypes.h

namespace ORB_SLAM3 {

// Descriptor storage format — determines distance metric.
// Binary: CV_8UC1, 32 bytes/row (ORB, BRIEF, BRISK). Use Hamming.
// Float32: CV_32FC1, 128 floats/row (SIFT, SURF). Use L2.
enum class DescriptorType { BINARY, FLOAT32 };

// Whether to apply QuadTree spatial distribution after detection.
// QUADTREE: needed for corner detectors (FAST) that detect many redundant
//           candidates and need explicit uniform distribution.
// NONE:     detectors with internal NMS (SIFT DoG, SURF Hessian) skip this.
enum class DistributionPolicy { QUADTREE, NONE };

} // namespace ORB_SLAM3
```

---

### 3.2 FeatureGrid

Encapsulates all grid state currently scattered across `Frame`'s static members and raw arrays. Fully self-contained and copyable — `KeyFrame` can copy it from `Frame` without special logic.

```cpp
// include/FeatureGrid.h

class FeatureGrid {
public:
    FeatureGrid() = default;

    // Configure grid for the given image bounds and cell counts.
    // minX/maxX/minY/maxY come from Frame::ComputeImageBounds().
    FeatureGrid(int cols, int rows,
                float minX, float maxX, float minY, float maxY);

    // Populate the grid from undistorted keypoints.
    // Clears any previous assignment.
    void assignFeatures(const std::vector<cv::KeyPoint>& keysUn);

    // Returns indices of keypoints within (x±r, y±r), filtered by octave.
    // Mirrors Frame::GetFeaturesInArea exactly.
    std::vector<size_t> getFeaturesInArea(float x, float y, float r,
                                          int minLevel = -1,
                                          int maxLevel = -1) const;

    // Maps an undistorted keypoint to a grid cell.
    // Returns false if out of bounds. Mirrors Frame::PosInGrid exactly.
    bool posInGrid(const cv::KeyPoint& kp, int& posX, int& posY) const;

    // Accessors for KeyFrame compatibility.
    // KeyFrame copies this data in its constructor.
    const std::vector<std::vector<std::vector<size_t>>>& data() const;
    int   numCols()              const;
    int   numRows()              const;
    float gridElementWidthInv()  const;
    float gridElementHeightInv() const;
    float minX() const; float maxX() const;
    float minY() const; float maxY() const;

private:
    int   mNumCols{64}, mNumRows{48};
    float mMinX{}, mMaxX{}, mMinY{}, mMaxY{};
    float mfGridElementWidthInv{}, mfGridElementHeightInv{};

    // Layout: [col][row] → vector of keypoint indices.
    // Matches KeyFrame's dynamic mGrid layout (eliminates the copy mismatch
    // between Frame's fixed array and KeyFrame's dynamic vector).
    std::vector<std::vector<std::vector<size_t>>> mGrid;
};
```

---

### 3.3 FeatureExtractor Base Class

```cpp
// include/FeatureExtractor.h

class FeatureExtractor {
public:
    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    // nfeatures   : desired total keypoints across all pyramid levels
    // scaleFactor : pyramid scale ratio between levels (e.g. 1.2 for ORB)
    // nlevels     : pyramid depth
    // gridRows/Cols : FeatureGrid dimensions (default: 48×64 = FRAME_GRID_*)
    FeatureExtractor(int nfeatures, float scaleFactor, int nlevels,
                     int gridRows = 48, int gridCols = 64);

    virtual ~FeatureExtractor() = default;
    FeatureExtractor(const FeatureExtractor&) = delete;
    FeatureExtractor& operator=(const FeatureExtractor&) = delete;

    // -----------------------------------------------------------------------
    // Main entry point — Template Method
    // -----------------------------------------------------------------------

    // Extracts features from `image` (CV_8UC1 for ORB; SIFT handles both).
    // `mask` is currently unused (mirrors original ORBextractor behaviour).
    // `vLappingArea` = {x0, x1} stereo overlap column range; pass {0,0} for mono.
    //
    // Populates result.keypoints, result.descriptors, result.imagePyramid,
    // and all scale metadata.
    //
    // NOTE: result.keypointsUn and result.grid are NOT populated here.
    // Undistortion requires camera parameters (K, distCoeffs) that belong
    // to Frame. The caller (Frame) must call UndistortKeyPoints() and then
    // result.grid.assignFeatures(result.keypointsUn).
    //
    // Returns monoCount (same semantics as the original operator() return value).
    virtual int extract(cv::InputArray image,
                        cv::InputArray mask,
                        const std::vector<int>& vLappingArea,
                        FeatureExtractorResult& result);

    // Drop-in compatibility with the original ORBextractor calling convention.
    // Internally calls extract(). Allows Frame::ExtractORB() to remain unchanged.
    int operator()(cv::InputArray image, cv::InputArray mask,
                   std::vector<cv::KeyPoint>& keypoints,
                   cv::OutputArray descriptors,
                   std::vector<int>& vLappingArea);

    // -----------------------------------------------------------------------
    // Scale info accessors — identical names as ORBextractor
    // -----------------------------------------------------------------------
    int   GetLevels()                                 const { return mnLevels; }
    float GetScaleFactor()                            const { return mfScaleFactor; }
    const std::vector<float>& GetScaleFactors()       const { return mvScaleFactor; }
    const std::vector<float>& GetInverseScaleFactors()const { return mvInvScaleFactor; }
    const std::vector<float>& GetScaleSigmaSquares()  const { return mvLevelSigma2; }
    const std::vector<float>& GetInverseScaleSigmaSquares() const { return mvInvLevelSigma2; }

    // Descriptor format — for matching code dispatch.
    virtual DescriptorType getDescriptorType() const = 0;

    // -----------------------------------------------------------------------
    // Public image pyramid
    // -----------------------------------------------------------------------
    // Populated after extract(). Exposed publicly because
    // Frame::ComputeStereoMatches() indexes mvImagePyramid[kp.octave]
    // directly for the subpixel sliding-window correlation step.
    std::vector<cv::Mat> mvImagePyramid;

protected:
    // -----------------------------------------------------------------------
    // Virtual hooks — override in specializations
    // -----------------------------------------------------------------------

    // Detect keypoints across all pyramid levels.
    // Called after buildPyramid(). mvImagePyramid is already populated.
    //
    // Output: allKeypoints[level] = detections in level-local coordinates,
    //         with kp.octave and kp.size set, kp.angle left for
    //         computeOrientation() to fill.
    //
    // ORBFeatureExtractor: FAST per 35×35 cell → DistributeQuadTree()
    // SIFTFeatureExtractor: cv::SIFT::detect() on full image; groups by kp.octave
    virtual void detect(std::vector<std::vector<cv::KeyPoint>>& allKeypoints) = 0;

    // Compute orientation for one pyramid level's keypoints in-place.
    // Called by extract() between detect() and computeDescriptors().
    //
    // ORBFeatureExtractor: IC-Angle via umax lookup table.
    // SIFTFeatureExtractor: no-op (cv::SIFT sets kp.angle during detect()).
    virtual void computeOrientation(const cv::Mat& levelImage,
                                    std::vector<cv::KeyPoint>& levelKeypoints) = 0;

    // Compute descriptors for all pyramid levels.
    // `workingPyramid[level]` = Gaussian-blurred level image
    //   (pre-blurred by base class; SIFT override may ignore it).
    //
    // ORBFeatureExtractor: rBRIEF rotated by kp.angle → CV_8UC1, 32 bytes/row
    // SIFTFeatureExtractor: cv::SIFT::compute() → CV_32FC1, 128 floats/row
    virtual void computeDescriptors(
        const std::vector<cv::Mat>& workingPyramid,
        std::vector<std::vector<cv::KeyPoint>>& allKeypoints,
        std::vector<cv::Mat>& descriptors) = 0;

    // Build the image pyramid into mvImagePyramid.
    // Default: Gaussian pyramid with EDGE_THRESHOLD=19 border padding,
    // BORDER_REFLECT_101 — identical to ORBextractor::ComputePyramid().
    // SIFTFeatureExtractor overrides this to produce a 2x-downscaled pyramid
    // compatible with cv::SIFT's octave labeling (needed for
    // Frame::ComputeStereoMatches() to index correctly).
    virtual void buildPyramid(const cv::Mat& image);

    // -----------------------------------------------------------------------
    // Shared utilities — not virtual
    // -----------------------------------------------------------------------

    // QuadTree spatial distribution of keypoints within a rectangle.
    // Implements DistributeOctTree() logic from ORBextractor.cc.
    // Called by ORBFeatureExtractor::detect(); SIFT does NOT use this.
    std::vector<cv::KeyPoint> distributeQuadTree(
        const std::vector<cv::KeyPoint>& candidates,
        int minX, int maxX, int minY, int maxY,
        int nFeatures, int level) const;

    // Fills mnFeaturesPerLevel via geometric series.
    void computeFeaturesPerLevel();

    // -----------------------------------------------------------------------
    // Base class state (shared by all specializations)
    // -----------------------------------------------------------------------
    int   mnFeatures;
    float mfScaleFactor;
    int   mnLevels;
    int   mGridRows, mGridCols;

    std::vector<int>   mnFeaturesPerLevel;
    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};
```

**`extract()` orchestration (pseudocode):**
```
extract(image, mask, vLappingArea, result):
    buildPyramid(image)                      // shared → fills mvImagePyramid

    detect(allKeypoints)                     // virtual: FAST cells vs SIFT DoG

    for each level:
        computeOrientation(pyramid[level],   // virtual: IC-Angle vs no-op
                           allKeypoints[level])

    workingPyramid = GaussianBlur(mvImagePyramid)   // shared

    computeDescriptors(workingPyramid,       // virtual: rBRIEF vs cv::SIFT
                       allKeypoints, perLevelDescs)

    // Scale kp.pt to level-0, apply lapping reorder — shared
    mergeAndReorder(allKeypoints, perLevelDescs, vLappingArea,
                    result.keypoints, result.descriptors, result.monoCount)

    result.imagePyramid   = mvImagePyramid
    result.scaleFactors   = mvScaleFactor  // etc.
    result.descriptorType = getDescriptorType()
```

---

### 3.4 ORBFeatureExtractor Specialization

```cpp
// include/ORBFeatureExtractor.h

class ORBFeatureExtractor : public FeatureExtractor {
public:
    // Matches original ORBextractor constructor exactly.
    // Drop-in replacement for all call sites in Tracking.cc and Frame.cc.
    ORBFeatureExtractor(int nfeatures, float scaleFactor, int nlevels,
                        int iniThFAST, int minThFAST);

    DescriptorType getDescriptorType() const override {
        return DescriptorType::BINARY;
    }

protected:
    // FAST in 35×35 cells, iniThFAST with minThFAST fallback,
    // then distributeQuadTree() per level.
    void detect(std::vector<std::vector<cv::KeyPoint>>& allKeypoints) override;

    // IC-Angle via umax lookup. Sets kp.angle in-place.
    void computeOrientation(const cv::Mat& levelImage,
                            std::vector<cv::KeyPoint>& levelKeypoints) override;

    // rBRIEF rotated by kp.angle using bit_pattern_31_.
    // Output: CV_8UC1, 32 bytes per row.
    void computeDescriptors(const std::vector<cv::Mat>& workingPyramid,
                            std::vector<std::vector<cv::KeyPoint>>& allKeypoints,
                            std::vector<cv::Mat>& descriptors) override;

    // buildPyramid() inherited (Gaussian pyramid is correct for ORB).

private:
    int mIniThFAST, mMinThFAST;
    std::vector<cv::Point> mPattern;   // 512-point rBRIEF pattern
    std::vector<int>       mumax;      // IC-Angle circular boundary
};
```

**What ORBFeatureExtractor does NOT override:**
- `buildPyramid()` — identical Gaussian pyramid
- `extract()` — full pipeline inherited
- `distributeQuadTree()` — utility called from `detect()`

The body of `ORBFeatureExtractor` is a direct migration of the private methods from `ORBextractor.cc`:
`ComputeKeyPointsOctTree` → `detect()`,  `IC_Angle` → `computeOrientation()`,  `computeOrbDescriptor` → `computeDescriptors()`.

---

### 3.5 SIFTFeatureExtractor Specialization

SIFT's scale-space is Difference-of-Gaussians (DoG), not a simple Gaussian image pyramid. This architectural difference makes some ORB quality features trivial to port and others impractical without re-implementing SIFT internals from scratch.

#### Feature parity: what is implemented vs. left out

| Feature | Status | Rationale |
| --- | --- | --- |
| QuadTree spatial distribution | **In** | ~30 lines. Base class `distributeQuadTree()` is reused directly after `cv::SIFT::detect()`. |
| Custom Gaussian pyramid exposure | **In** | ~20 lines. Build a separate N-level 2×-downscaled Gaussian pyramid in `buildPyramid()`. Required for `ComputeStereoMatches()`. |
| Dual-threshold coverage (approximation) | **In** | ~15 lines. Run SIFT at a low `contrastThreshold` to get all candidates, then let QuadTree distribution select the best N uniformly. This achieves the same coverage goal without per-cell detection. |
| Custom scale factor (e.g. 1.2) | **Out** | Requires implementing DoG scale-space extrema detection from scratch. `cv::SIFT` hardcodes octave-doubling (factor 2.0). |
| Per-cell SIFT detection | **Out** | DoG scale-space requires sufficiently large image regions; a 35×35 cell is too small for meaningful multi-scale detection. |

#### Why the dual-threshold approximation works differently for SIFT

ORB's dual-threshold operates per cell: run FAST at `iniThFAST`; if the cell is empty, retry at `minThFAST`. The goal is to guarantee at least one feature in sparse regions.

For SIFT, the equivalent is:
1. Run `cv::SIFT::detect()` globally at a low `contrastThreshold` (getting all candidates, including weak ones in sparse regions).
2. Apply `distributeQuadTree()` per octave level to select N features uniformly.

QuadTree selection naturally promotes weaker features in sparse areas (since they are the only candidates in those cells) and strong features in dense areas. This achieves the same spatial coverage goal with a single detection pass. There is no meaningful benefit to a two-pass SIFT call for this purpose.

#### Class interface

```cpp
// include/SIFTFeatureExtractor.h

class SIFTFeatureExtractor : public FeatureExtractor {
public:
    // nfeatures          : desired total keypoints (passed to cv::SIFT and QuadTree)
    // nOctaveLayers      : SIFT layers per octave (default 3; more = finer scale steps)
    // contrastThreshold  : DoG response threshold; lower = more features in flat regions
    // edgeThresh         : suppresses edge-like responses
    // sigma              : initial Gaussian blur sigma
    // nlevels            : number of octaves to expose in the base class scale tables;
    //                      set to match the SIFT noctaves you configure cv::SIFT with.
    //                      Scale factor is always 2.0 (one octave = 2× downscale).
    SIFTFeatureExtractor(int    nfeatures,
                         int    nOctaveLayers    = 3,
                         double contrastThreshold = 0.03,   // lower than cv default (0.04)
                         double edgeThresh        = 10.0,
                         double sigma             = 1.6,
                         int    nlevels           = 4,
                         int    gridRows          = 48,
                         int    gridCols          = 64);

    DescriptorType getDescriptorType() const override {
        return DescriptorType::FLOAT32;
    }

protected:
    // Builds a standard N-level Gaussian pyramid at 2× downscale per level.
    // imagePyramid[i] = image at scale 2^i, produced with GaussianBlur + resize.
    // cv::SIFT sets kp.octave = (kp.octave & 0xFF) to match this 2× convention,
    // so Frame::ComputeStereoMatches() can safely index mvImagePyramid[kp.octave].
    void buildPyramid(const cv::Mat& image) override;

    // 1. Runs cv::SIFT::detect() globally at contrastThreshold (low, to get
    //    all candidates including those in sparse regions).
    // 2. Decodes kp.octave: SIFT packs octave = (octave & 0xFF). Groups
    //    keypoints by decoded octave → allKeypoints[level].
    // 3. Calls distributeQuadTree() per level to select mnFeaturesPerLevel[level]
    //    keypoints with uniform spatial coverage.
    void detect(std::vector<std::vector<cv::KeyPoint>>& allKeypoints) override;

    // No-op: cv::SIFT sets kp.angle during detect() via gradient histogram.
    void computeOrientation(const cv::Mat&,
                            std::vector<cv::KeyPoint>&) override {}

    // Calls cv::SIFT::compute() per level on the pyramid level image.
    // Output: CV_32FC1, 128 floats per row.
    // workingPyramid (Gaussian-blurred) is ignored; SIFT uses its own internals.
    void computeDescriptors(const std::vector<cv::Mat>& workingPyramid,
                            std::vector<std::vector<cv::KeyPoint>>& allKeypoints,
                            std::vector<cv::Mat>& descriptors) override;

private:
    cv::Ptr<cv::SIFT> mSIFT;
};
```

#### Scale table note

The base class scale tables (`mvScaleFactor`, `mvLevelSigma2`, etc.) are initialised with `scaleFactor = 2.0` and `nlevels` set to the number of SIFT octaves. These tables are what `ORBmatcher` uses to compute search radii (`radius = th * mvScaleFactors[predictedLevel]`). Since the exposed pyramid also uses 2× downscaling and SIFT's octave index matches it, the search radii are correct without any additional mapping.

**Per-type specialization summary:**

| Concern | FeatureExtractor base | ORBFeatureExtractor | SIFTFeatureExtractor |
|---|---|---|---|
| Gaussian pyramid | `buildPyramid()` default (configurable scale) | Inherited | Override: 2×-downscale per octave |
| Detection | Pure virtual | FAST per 35×35 cell + QuadTree | `cv::SIFT::detect()` globally |
| Coverage in sparse regions | — | Per-cell minThFAST fallback | Low `contrastThreshold` → QuadTree picks sparse-region candidates |
| QuadTree distribution | Utility `distributeQuadTree()` | Called from `detect()` | Called from `detect()` |
| Orientation | Pure virtual | IC-Angle via umax | No-op (SIFT sets `kp.angle`) |
| Descriptors | Pure virtual | rBRIEF, CV_8UC1 × 32 bytes | `cv::SIFT::compute()`, CV_32FC1 × 128 floats |
| Pipeline merging / lapping | Non-virtual in `extract()` | Inherited | Inherited |
| Scale tables | Constructor, non-virtual | `scaleFactor` = constructor param | `scaleFactor` fixed at 2.0 |
| Custom scale factor | Supported for ORB | Yes (e.g. 1.2) | No — cv::SIFT hardcodes 2.0 |

---

## 4. DescriptorMetric — Isolating the Matcher

### 4.1 The Problem

`ORBmatcher` is a large class (~1900 lines) containing sophisticated matching algorithms: projection-based search, BoW-guided search, triangulation search, Sim3 matching, and fusing. All of these use **two ORB-specific pieces**:

1. `DescriptorDistance()` — Hamming distance on 32-byte binary descriptors
2. `TH_LOW = 50` and `TH_HIGH = 100` — thresholds calibrated for the 0–256 Hamming range

Everything else in `ORBmatcher` is **feature-type-agnostic geometry**: projection, area search, rotation histograms, scale-radius heuristics. None of it reads `ORBextractor` directly.

Replacing `ORBmatcher` wholesale for SIFT would duplicate ~1900 lines for the sake of 20 lines of distance logic.

---

### 4.2 Options Considered

**Option A — Duplicate ORBmatcher as SIFTMatcher (rejected)**
Copies all geometric matching logic just to change the distance function. Creates a maintenance burden every time the matching logic is updated.

**Option B — Full `FeatureMatcher` abstract base class (rejected for now)**
Extracting a virtual interface for all 16 overloaded `Search*()` methods is an O(large) refactor with no immediate benefit beyond what Option C provides. The matching logic has no branching on feature type — only the distance function does.

**Option C — `DescriptorMetric` strategy injected into ORBmatcher (recommended)**
ORBmatcher becomes a `FeatureMatcher` with a pluggable distance metric. This is ~30 lines of new code and a single-point change in `ORBmatcher`.

---

### 4.3 Recommended Approach: DescriptorMetric Strategy

```cpp
// include/DescriptorMetric.h

class DescriptorMetric {
public:
    virtual ~DescriptorMetric() = default;

    // Distance between two descriptor rows. Range must be [0, maxDistance()].
    virtual int distance(const cv::Mat& a, const cv::Mat& b) const = 0;

    // Thresholds equivalent to TH_LOW and TH_HIGH in ORBmatcher.
    // ORBmatcher uses them as: accept match if dist < thHigh,
    // prefer match if dist < thLow. Both scale linearly with maxDistance().
    virtual int thLow()  const = 0;
    virtual int thHigh() const = 0;
};

// -----------------------------------------------------------------------
class HammingMetric : public DescriptorMetric {
public:
    int distance(const cv::Mat& a, const cv::Mat& b) const override;
    int thLow()  const override { return 50;  }   // original TH_LOW
    int thHigh() const override { return 100; }   // original TH_HIGH
};

// -----------------------------------------------------------------------
// L2 distance for float descriptors (SIFT, SURF).
// Thresholds are empirical: 0.7 × max_SIFT_L2 ≈ 200 for normalized SIFT.
// These MUST be calibrated for a given dataset before production use.
class L2Metric : public DescriptorMetric {
public:
    explicit L2Metric(int thLow = 200, int thHigh = 400);
    int distance(const cv::Mat& a, const cv::Mat& b) const override;
    int thLow()  const override { return mThLow;  }
    int thHigh() const override { return mThHigh; }
private:
    int mThLow, mThHigh;
};
```

**Changes to ORBmatcher:**

```cpp
class ORBmatcher {
public:
    // DescriptorMetric defaults to HammingMetric for backward compatibility.
    // Pass an L2Metric for SIFT.
    explicit ORBmatcher(float nnratio = 0.6,
                        bool checkOri = true,
                        std::shared_ptr<DescriptorMetric> metric =
                            std::make_shared<HammingMetric>());

    // DescriptorDistance becomes a non-static method (or calls mMetric).
    // All internal call sites replace:
    //   DescriptorDistance(a, b)       →  mMetric->distance(a, b)
    //   TH_LOW                         →  mMetric->thLow()
    //   TH_HIGH                        →  mMetric->thHigh()

private:
    std::shared_ptr<DescriptorMetric> mMetric;
    // mfNNratio, mbCheckOrientation unchanged
};
```

**Impact:**
- ~35 call sites in ORBmatcher.cc replace `DescriptorDistance(a,b)` with `mMetric->distance(a,b)` and constants with `mMetric->thLow()/thHigh()`.
- No algorithmic changes. No new files except `DescriptorMetric.h/.cc`.
- All existing users of `ORBmatcher` continue to work with no change (default metric = Hamming).

**Usage with SIFT:**
```cpp
auto metric = std::make_shared<L2Metric>(/*thLow=*/200, /*thHigh=*/400);
ORBmatcher matcher(0.7f, true, metric);
matcher.SearchByProjection(frame, mapPoints, 3.0f);
```

---

### 4.4 BoW Vocabulary Caveat

`ORBmatcher::SearchByBoW()` and `SearchForTriangulation()` rely on `Frame::mFeatVec` (DBoW2 `FeatureVector`). DBoW2 vocabularies are trained on **binary** ORB descriptors and cannot be directly used with SIFT float descriptors.

Options when using SIFT:
- **Disable SearchByBoW** and fall back to brute-force or projection-only methods. This affects relocalization and loop closure quality.
- **Retrain the vocabulary** with DBoW3 (supports float descriptors) or a custom vocabulary trained on SIFT descriptors from the target dataset.
- **Use a different place recognition approach** (e.g., NetVLAD, DBOW-float).

This is outside the scope of this refactor but must be resolved before SIFT is used in loop-closure workflows.

---

## 5. Frame After Refactoring

### Constructor signature change

```cpp
// Before:
Frame(const cv::Mat& imLeft, const cv::Mat& imRight, ...,
      ORBextractor* extractorLeft, ORBextractor* extractorRight, ...);

// After:
Frame(const cv::Mat& imLeft, const cv::Mat& imRight, ...,
      FeatureExtractor* extractorLeft, FeatureExtractor* extractorRight, ...);
```

### Constructor flow

```
Frame(imLeft, imRight, ...):
    // 1. Extract features (parallel threads for stereo, unchanged)
    thread L: extractor->extract(imLeft,  {}, {0,0}, mResultLeft)
    thread R: extractor->extract(imRight, {}, {0,0}, mResultRight)

    // 2. Copy out for backward compat
    mvKeys       = mResultLeft.keypoints
    mDescriptors = mResultLeft.descriptors
    N            = mvKeys.size()

    // 3. Undistort (Frame still owns camera params)
    UndistortKeyPoints()           // fills mvKeysUn from mvKeys + K + distCoef

    // 4. Build grid (Frame still owns image bounds)
    ComputeImageBounds(imLeft)     // fills mnMinX, mnMaxX, mnMinY, mnMaxY
    mGrid = FeatureGrid(64, 48, mnMinX, mnMaxX, mnMinY, mnMaxY)
    mGrid.assignFeatures(mvKeysUn)

    // 5. Scale info now comes from result, not extractor getters
    mnScaleLevels    = mResultLeft.nlevels
    mfScaleFactor    = mResultLeft.scaleFactor
    mvScaleFactors   = mResultLeft.scaleFactors
    mvLevelSigma2    = mResultLeft.levelSigma2
    mvInvLevelSigma2 = mResultLeft.invLevelSigma2
```

### Grid query delegation

```cpp
// In Frame.cc:
std::vector<size_t> Frame::GetFeaturesInArea(
    float x, float y, float r, int minLevel, int maxLevel, bool bRight) const
{
    return bRight ? mGridRight.getFeaturesInArea(x, y, r, minLevel, maxLevel)
                  : mGrid.getFeaturesInArea(x, y, r, minLevel, maxLevel);
}
```

### What leaves Frame

| Removed from Frame | Moves to |
|---|---|
| `ORBextractor* mpORBextractorLeft/Right` | `FeatureExtractor* mpFeatureExtractorLeft/Right` |
| `mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS]` | `FeatureGrid mGrid` |
| `static float mfGridElementWidthInv/HeightInv` | `FeatureGrid` internal |
| `void AssignFeaturesToGrid()` | `FeatureGrid::assignFeatures()` |
| `bool PosInGrid(kp, x, y)` | `FeatureGrid::posInGrid()` |
| `GetFeaturesInArea(...)` | Thin wrapper delegating to `FeatureGrid` |

### KeyFrame compatibility

`KeyFrame::KeyFrame(Frame& F, ...)` currently copies `mGrid` cell-by-cell from Frame's fixed array into KeyFrame's dynamic vector. After refactoring:

```cpp
// KeyFrame.cc constructor:
mGrid             = F.mGrid.data();                       // direct copy of dynamic vector
mfGridElementWidthInv  = F.mGrid.gridElementWidthInv();
mfGridElementHeightInv = F.mGrid.gridElementHeightInv();
```

No other changes to KeyFrame are needed.

---

## 6. Implementation Scope and File Map

### New files

| File | Purpose |
|---|---|
| `include/FeatureTypes.h` | `DescriptorType`, `DistributionPolicy` enums |
| `include/FeatureGrid.h` / `src/FeatureGrid.cc` | Grid abstraction (from Frame) |
| `include/FeatureExtractor.h` / `src/FeatureExtractor.cc` | Abstract base class + shared pipeline |
| `include/ORBFeatureExtractor.h` / `src/ORBFeatureExtractor.cc` | ORB specialization (migrated from ORBextractor) |
| `include/SIFTFeatureExtractor.h` / `src/SIFTFeatureExtractor.cc` | SIFT specialization |
| `include/DescriptorMetric.h` / `src/DescriptorMetric.cc` | Distance metric strategy |

### Modified files

| File | What changes |
|---|---|
| `include/Frame.h` | `ORBextractor*` → `FeatureExtractor*`; `mGrid[][]` → `FeatureGrid`; remove static grid members |
| `src/Frame.cc` | `ExtractORB` calls `FeatureExtractor::operator()`; `AssignFeaturesToGrid` delegates to `FeatureGrid` |
| `include/Tracking.h` | `mpORBextractorLeft/Right/Ini` → `FeatureExtractor*` |
| `src/Tracking.cc` | Instantiates `ORBFeatureExtractor` (or SIFT variant) |
| `include/ORBmatcher.h` | Add `DescriptorMetric` constructor param; `DescriptorDistance` becomes non-static |
| `src/ORBmatcher.cc` | Replace `DescriptorDistance()` / `TH_LOW` / `TH_HIGH` with metric calls |
| `include/KeyFrame.h` | `mGrid` type change to match `FeatureGrid::data()` layout (already matches) |
| `src/KeyFrame.cc` | Update grid copy from Frame to use `FeatureGrid` accessors |
| `CMakeLists.txt` | Add new source files |

### Files that do NOT change

| File | Why |
|---|---|
| `src/ORBextractor.cc` | Kept as-is; `ORBFeatureExtractor` is a clean replacement, not a patch |
| `src/Optimizer.cc` | Uses `mvLevelSigma2` from Frame/KeyFrame — unchanged |
| `src/LocalMapping.cc` | Creates Frames / calls matching — interface unchanged |
| `src/LoopClosing.cc` | BoW-based — unchanged (see §4.4 for SIFT caveat) |
| `src/System.cc` | No direct extractor usage |

### Suggested implementation order

1. `FeatureTypes.h` — trivial, no dependencies
2. `FeatureGrid.h/.cc` — isolate grid logic from Frame (no FeatureExtractor dependency)
3. `FeatureExtractor.h/.cc` — base class with shared pipeline; stub virtual methods
4. `ORBFeatureExtractor.h/.cc` — migrate ORBextractor private methods here
5. Update `Frame.h/.cc` to use `FeatureExtractor*` and `FeatureGrid`
6. Update `KeyFrame.cc` grid copy
7. Update `Tracking.cc` to instantiate `ORBFeatureExtractor`
8. `DescriptorMetric.h/.cc` — inject into ORBmatcher
9. Update `ORBmatcher.cc` call sites
10. `SIFTFeatureExtractor.h/.cc` — SIFT specialization (requires opencv_features2d with SIFT)

### Verification

After step 7 (before SIFT), the system should be fully functional with ORB at identical performance:
- `rosrun orbslam3_ros mono_node` with existing configs should produce identical trajectories
- Unit test: extract ORB with `ORBFeatureExtractor` and compare keypoints/descriptors to original `ORBextractor` output on the same image — must be bit-identical
- Integration test: run on EuRoC or TUM sequence, compare ATE to baseline

After step 10 (SIFT):

- Run on a low-texture sequence where ORB struggles; verify SIFT produces more keypoints
- Verify `L2Metric` thresholds produce a reasonable inlier ratio in `SearchByProjection`
- Note: BoW-based relocalization will be degraded until vocabulary is retrained

---

## 7. Standalone Library

### 7.1 Dependency Audit

The key reason isolation is clean is that `ORBextractor.cc` already has zero ORB-SLAM3-specific dependencies — confirmed by its includes:

```cpp
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <list>
#include "ORBextractor.h"
```

No `Frame.h`, no `MapPoint.h`, no `Tracking.h`. The new classes (`FeatureGrid`, `FeatureExtractor`, `ORBFeatureExtractor`, `SIFTFeatureExtractor`, `DescriptorMetric`) inherit this property — they depend only on OpenCV and the C++ standard library.

| Component | Dependencies |
|---|---|
| `FeatureTypes.h` | None |
| `FeatureGrid` | OpenCV core |
| `FeatureExtractor` | OpenCV core, imgproc, features2d |
| `ORBFeatureExtractor` | OpenCV core, imgproc, features2d |
| `SIFTFeatureExtractor` | OpenCV features2d (requires `opencv_contrib` or OpenCV ≥ 4.4 nonfree) |
| `DescriptorMetric` | OpenCV core |

The standalone library has **no dependency on ORB-SLAM3, Eigen, Sophus, DBoW2, GTSAM, Pangolin, or ROS**.

---

### 7.2 Library Layout

The library lives as a sibling repository (or git submodule) — not inside the ORB-SLAM3 source tree.

```
feature_extractor/
├── CMakeLists.txt
├── cmake/
│   └── FeatureExtractorConfig.cmake.in
├── include/
│   └── feature_extractor/
│       ├── FeatureTypes.h
│       ├── FeatureGrid.h
│       ├── FeatureExtractor.h
│       ├── ORBFeatureExtractor.h
│       ├── SIFTFeatureExtractor.h
│       └── DescriptorMetric.h
└── src/
    ├── FeatureGrid.cc
    ├── FeatureExtractor.cc
    ├── ORBFeatureExtractor.cc     ← body migrated from ORBextractor.cc
    ├── SIFTFeatureExtractor.cc
    └── DescriptorMetric.cc
```

Headers are namespaced under `include/feature_extractor/` so consumer includes are unambiguous:

```cpp
#include <feature_extractor/ORBFeatureExtractor.h>
#include <feature_extractor/SIFTFeatureExtractor.h>
#include <feature_extractor/DescriptorMetric.h>
```

---

### 7.3 CMake Design

```cmake
cmake_minimum_required(VERSION 3.16)
project(FeatureExtractor VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# -----------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------
find_package(OpenCV 4.2 REQUIRED COMPONENTS core imgproc features2d)

# -----------------------------------------------------------------------
# Library target
# -----------------------------------------------------------------------
add_library(FeatureExtractor
    src/FeatureGrid.cc
    src/FeatureExtractor.cc
    src/ORBFeatureExtractor.cc
    src/SIFTFeatureExtractor.cc
    src/DescriptorMetric.cc
)

# Public headers are under include/feature_extractor/ — consumers get
# the include path, not a flat dump of headers.
target_include_directories(FeatureExtractor
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(FeatureExtractor
    PUBLIC
        opencv_core
        opencv_imgproc
        opencv_features2d
)

# -----------------------------------------------------------------------
# Install rules
# -----------------------------------------------------------------------
install(TARGETS FeatureExtractor
    EXPORT  FeatureExtractorTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/feature_extractor
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# -----------------------------------------------------------------------
# CMake package config (enables find_package(FeatureExtractor))
# -----------------------------------------------------------------------
set(FE_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/FeatureExtractor")

configure_package_config_file(
    cmake/FeatureExtractorConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/FeatureExtractorConfig.cmake
    INSTALL_DESTINATION ${FE_CONFIG_DIR}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/FeatureExtractorConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(EXPORT FeatureExtractorTargets
    NAMESPACE  FeatureExtractor::
    DESTINATION ${FE_CONFIG_DIR}
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/FeatureExtractorConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/FeatureExtractorConfigVersion.cmake
    DESTINATION ${FE_CONFIG_DIR}
)

# Build-tree export (no install needed for in-tree use)
export(EXPORT FeatureExtractorTargets
    NAMESPACE  FeatureExtractor::
    FILE       ${CMAKE_CURRENT_BINARY_DIR}/FeatureExtractorTargets.cmake
)
```

**`cmake/FeatureExtractorConfig.cmake.in`:**

```cmake
@PACKAGE_INIT@

include(CMakeFindDependencyMacro)
find_dependency(OpenCV 4.2 REQUIRED COMPONENTS core imgproc features2d)

include("${CMAKE_CURRENT_LIST_DIR}/FeatureExtractorTargets.cmake")
check_required_components(FeatureExtractor)
```

After `cmake --install`, consumers call:

```cmake
find_package(FeatureExtractor 1.0 REQUIRED)
target_link_libraries(my_target PRIVATE FeatureExtractor::FeatureExtractor)
```

---

### 7.4 Consuming the Library in ORB-SLAM3

#### CMakeLists.txt changes (ORB-SLAM3)

```cmake
# Before (ORB-SLAM3/CMakeLists.txt):
find_package(OpenCV 4.2 REQUIRED)

# After — add:
find_package(FeatureExtractor 1.0 REQUIRED)
```

In `src/CMakeLists.txt`, add `FeatureExtractor::FeatureExtractor` to the `target_link_libraries` of the ORB-SLAM3 library target. Remove `ORBextractor.cc` from the source list (it is superseded by `ORBFeatureExtractor` in the standalone library).

#### Backward-compatibility shim (zero churn in ORB-SLAM3 source files)

Rather than updating every `#include "ORBextractor.h"` and every `ORBextractor*` pointer in `Tracking.cc`, `Frame.cc`, etc., a one-line shim keeps existing code compiling unchanged:

```cpp
// include/ORBextractor.h  (kept in place, contents replaced)
#pragma once
#include <feature_extractor/ORBFeatureExtractor.h>

namespace ORB_SLAM3 {
    // Drop-in alias: all existing ORBextractor* pointers and new/delete
    // calls continue to work without modification.
    using ORBextractor = ORBFeatureExtractor;
}
```

With this shim:

- `Tracking.cc` lines 133–142 (`new ORBextractor(...)`) compile unchanged.
- `Frame.cc` `mpORBextractorLeft`, `mpORBextractorRight` declarations compile unchanged.
- `Frame::ExtractORB()` and its `(*mpORBextractorLeft)(...)` call compile unchanged.
- The `ORBextractor.cc` source file is retired (removed from `src/CMakeLists.txt`) but `ORBextractor.h` remains as the shim.

#### Opting in to the new interface

Code that wants to use `SIFTFeatureExtractor` or `DescriptorMetric` explicitly includes the new headers and uses `FeatureExtractor*` base pointers. Old code using the shim alias coexists without conflict. Migration can be done incrementally file by file.

#### File changes summary

| File | Change |
| --- | --- |
| `CMakeLists.txt` | Add `find_package(FeatureExtractor)` |
| `src/CMakeLists.txt` | Link `FeatureExtractor::FeatureExtractor`; remove `ORBextractor.cc` from sources |
| `include/ORBextractor.h` | Replace with compatibility shim (5 lines) |
| `src/ORBextractor.cc` | Retired — logic now lives in the standalone library |
| All other ORB-SLAM3 source files | **No changes required** |
