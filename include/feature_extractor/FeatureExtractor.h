#pragma once

#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

namespace ORB_SLAM3
{

// Helper node for QuadTree spatial distribution.
// Kept in the public header so that FeatureExtractor::distributeOctTree()
// and subclasses can use it without an extra include.
class ExtractorNode
{
public:
    ExtractorNode() : bNoMore(false) {}

    void DivideNode(ExtractorNode& n1, ExtractorNode& n2, ExtractorNode& n3, ExtractorNode& n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

// Abstract base class for feature extractors.
//
// Uses the Template Method pattern: operator() orchestrates the full pipeline
// and calls virtual hooks (buildPyramid, detect, computeOrientation,
// computeDescriptors) that subclasses override.
//
// The operator() signature is identical to the original ORBextractor::operator()
// so Frame::ExtractORB() requires zero changes.
class FeatureExtractor
{
public:
    // nfeatures   : desired total keypoints across all pyramid levels
    // scaleFactor : scale ratio between pyramid levels (e.g. 1.2 for ORB)
    // nlevels     : number of pyramid levels
    FeatureExtractor(int nfeatures, float scaleFactor, int nlevels);

    virtual ~FeatureExtractor() = default;
    FeatureExtractor(const FeatureExtractor&) = delete;
    FeatureExtractor& operator=(const FeatureExtractor&) = delete;

    // -----------------------------------------------------------------------
    // Main entry point — drop-in replacement for ORBextractor::operator()
    // -----------------------------------------------------------------------
    // Extracts features and descriptors from image (CV_8UC1).
    // mask is currently ignored (mirrors original behaviour).
    // vLappingArea = {x0, x1} stereo overlap column range; pass {0,0} for mono.
    // Returns monoCount — same semantics as the original ORBextractor.
    virtual int operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                           cv::OutputArray descriptors, std::vector<int>& vLappingArea);

    // -----------------------------------------------------------------------
    // Scale info accessors — identical names to ORBextractor
    // -----------------------------------------------------------------------
    int GetLevels() const { return mnLevels; }
    float GetScaleFactor() const { return mfScaleFactor; }
    const std::vector<float>& GetScaleFactors() const { return mvScaleFactor; }
    const std::vector<float>& GetInverseScaleFactors() const { return mvInvScaleFactor; }
    const std::vector<float>& GetScaleSigmaSquares() const { return mvLevelSigma2; }
    const std::vector<float>& GetInverseScaleSigmaSquares() const { return mvInvLevelSigma2; }

    // Matching distance thresholds appropriate for this descriptor type.
    // Subclasses override when the descriptor width differs from ORB's 256 bits.
    virtual int matchThLow()  const { return 50; }
    virtual int matchThHigh() const { return 100; }

    // -----------------------------------------------------------------------
    // Public image pyramid
    // -----------------------------------------------------------------------
    // Populated after each operator() call. Exposed publicly because
    // Frame::ComputeStereoMatches() indexes mvImagePyramid[kp.octave]
    // directly for the subpixel sliding-window correlation step.
    std::vector<cv::Mat> mvImagePyramid;

protected:
    // -----------------------------------------------------------------------
    // Virtual hooks — override in specializations
    // -----------------------------------------------------------------------

    // Build the image pyramid into mvImagePyramid.
    // Default implementation: Gaussian pyramid with EDGE_THRESHOLD=19 border
    // padding (BORDER_REFLECT_101) — identical to ORBextractor::ComputePyramid().
    virtual void buildPyramid(const cv::Mat& image);

    // Detect keypoints across all pyramid levels.
    // Called after buildPyramid(). mvImagePyramid is already populated.
    // Output: allKeypoints[level] = detections in level-local coordinates
    //         with kp.octave and kp.size set; kp.angle is NOT set here
    //         (computeOrientation fills it in the next step).
    virtual void detect(std::vector<std::vector<cv::KeyPoint>>& allKeypoints) = 0;

    // Compute keypoint orientations for one pyramid level in-place.
    // Called by operator() for each level after detect().
    // ORBFeatureExtractor: IC-Angle via umax lookup table.
    virtual void computeOrientation(const cv::Mat& levelImage, std::vector<cv::KeyPoint>& levelKeypoints) = 0;

    // Compute descriptors for one pyramid level.
    // workingMat = Gaussian-blurred level image (pre-blurred by operator()).
    // descriptors is an OUTPUT — the implementation allocates it.
    // ORBFeatureExtractor: rBRIEF → CV_8UC1, 32 bytes/row.
    virtual void computeDescriptors(const cv::Mat& workingMat, std::vector<cv::KeyPoint>& levelKeypoints,
                                    cv::Mat& descriptors) = 0;

    // -----------------------------------------------------------------------
    // Shared utility — QuadTree spatial distribution
    // -----------------------------------------------------------------------
    // Selects up to nFeatures keypoints uniformly distributed within the
    // [minX, maxX] × [minY, maxY] rectangle using recursive quadtree
    // subdivision. Each leaf retains the keypoint with the highest FAST
    // response. Mirrors ORBextractor::DistributeOctTree() exactly.
    std::vector<cv::KeyPoint> distributeOctTree(const std::vector<cv::KeyPoint>& candidates, int minX, int maxX,
                                                int minY, int maxY, int nFeatures, int level) const;

    // Fills mnFeaturesPerLevel via geometric series sum.
    void computeFeaturesPerLevel();

    // Reorders keypoints/descriptors so mono keypoints (outside the stereo
    // lapping area) come first and stereo keypoints come last. Both vectors
    // must be in correspondence. Returns monoIndex (count of mono keypoints).
    int reorderForStereo(std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                         const std::vector<int>& vLappingArea) const;

    // -----------------------------------------------------------------------
    // Base class state
    // -----------------------------------------------------------------------
    int mnFeatures;
    float mfScaleFactor;
    int mnLevels;

    std::vector<int> mnFeaturesPerLevel;
    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

}  // namespace ORB_SLAM3
