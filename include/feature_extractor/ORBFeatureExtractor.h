#pragma once

#include "feature_extractor/FeatureExtractor.h"

#include <opencv2/core/core.hpp>
#include <vector>

namespace ORB_SLAM3
{

// ORB feature extractor — a direct drop-in replacement for the original
// ORBextractor class. Constructor signature is identical; all existing
// call sites in Tracking.cc and Frame.cc compile without modification.
//
// Implementation maps to ORBextractor internals:
//   detect()             ← ComputeKeyPointsOctTree()  (without orientation step)
//   computeOrientation() ← IC_Angle() per keypoint
//   computeDescriptors() ← computeOrbDescriptor() per keypoint
//   buildPyramid()         inherited (Gaussian pyramid, EDGE_THRESHOLD=19)
//   distributeOctTree()    inherited from FeatureExtractor base
class ORBFeatureExtractor : public FeatureExtractor
{
public:
    // Identical signature to the original ORBextractor constructor.
    // nfeatures   : target number of keypoints across all levels
    // scaleFactor : pyramid scale ratio between levels (e.g. 1.2)
    // nlevels     : number of pyramid levels (e.g. 8)
    // iniThFAST   : primary FAST corner threshold
    // minThFAST   : fallback FAST threshold for cells with no detections
    ORBFeatureExtractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);

    DescriptorType getDescriptorType() const override { return DescriptorType::BINARY; }

protected:
    // FAST detection in 35×35 cells with iniThFAST / minThFAST fallback,
    // followed by distributeOctTree() for uniform spatial coverage.
    void detect(std::vector<std::vector<cv::KeyPoint>>& allKeypoints) override;

    // IC-Angle orientation using the umax lookup table. Sets kp.angle in-place.
    void computeOrientation(const cv::Mat& levelImage, std::vector<cv::KeyPoint>& levelKeypoints) override;

    // Rotated BRIEF (rBRIEF) using the 512-point bit_pattern_31_ pattern.
    // Output: CV_8UC1, 32 bytes per row.
    void computeDescriptors(const cv::Mat& workingMat, std::vector<cv::KeyPoint>& levelKeypoints,
                            cv::Mat& descriptors) override;

    // buildPyramid() is inherited — same Gaussian pyramid as the original.

private:
    int mIniThFAST;
    int mMinThFAST;

    std::vector<cv::Point> mPattern;  // 512 cv::Point pairs for rBRIEF
    std::vector<int> mumax;           // IC-Angle circular boundary lookup
};

}  // namespace ORB_SLAM3
