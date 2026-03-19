#pragma once

#include "feature_extractor/FeatureExtractor.h"
#include "feature_extractor/FeatureTypes.h"

#include <opencv2/features2d.hpp>

namespace ORB_SLAM3
{

// ORB specialization backed directly by cv::ORB.
//
// Detection strategy:
//   cv::ORB::detect() is called per pyramid level on the pre-built pyramid
//   images from the base class (one level at a time, with cv::ORB configured
//   for nlevels=1 to suppress its internal pyramid). Candidates are spatially
//   distributed using distributeOctTree() for uniform coverage, matching the
//   spatial quality of GridBasedORBFeatureExtractor without the custom FAST
//   dual-threshold logic.
//
// Descriptor format: CV_8UC1, 32 bytes/row. Use Hamming distance for matching.
class VanillaORBFeatureExtractor : public FeatureExtractor
{
public:
    // nfeatures     : desired total keypoints across all pyramid levels
    // scaleFactor   : scale ratio between pyramid levels (e.g. 1.2)
    // nlevels       : number of pyramid levels
    // fastThreshold : FAST corner detection threshold
    // scoreType     : cv::ORB::HARRIS_SCORE (default) or cv::ORB::FAST_SCORE
    //   HARRIS_SCORE — re-ranks FAST corners by Harris response; more stable keypoints
    //   FAST_SCORE   — ranks by raw FAST response; slightly faster, less stable
    VanillaORBFeatureExtractor(int nfeatures, float scaleFactor, int nlevels, int fastThreshold = 20,
                               cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE);

    DescriptorType getDescriptorType() const override { return DescriptorType::BINARY; }

protected:
    // buildPyramid() is inherited — Gaussian pyramid with EDGE_THRESHOLD=19 border.

    // Per-level cv::ORB detection on the pre-built pyramid, followed by
    // distributeOctTree() for uniform spatial distribution.
    void detect(std::vector<std::vector<cv::KeyPoint>>& allKeypoints) override;

    // No-op: cv::ORB::detect() already sets kp.angle.
    void computeOrientation(const cv::Mat& levelImage,
                            std::vector<cv::KeyPoint>& levelKeypoints) override;

    // cv::ORB::compute() on the Gaussian-blurred level image.
    // Output: CV_8UC1, 32 bytes/row.
    void computeDescriptors(const cv::Mat& workingMat, std::vector<cv::KeyPoint>& levelKeypoints,
                            cv::Mat& descriptors) override;

private:
    cv::Ptr<cv::ORB> mORB;
};

}  // namespace ORB_SLAM3
