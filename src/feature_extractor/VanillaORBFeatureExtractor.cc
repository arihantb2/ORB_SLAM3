#include "feature_extractor/VanillaORBFeatureExtractor.h"

namespace ORB_SLAM3
{

// Edge margin applied when building the base-class Gaussian pyramid.
// Must match EDGE_THRESHOLD in FeatureExtractor.cc so detections stay inside
// the valid image region.
static constexpr int EDGE_THRESHOLD = 19;

VanillaORBFeatureExtractor::VanillaORBFeatureExtractor(int nfeatures, float scaleFactor, int nlevels, int fastThreshold,
                                                       cv::ORB::ScoreType scoreType)
    : FeatureExtractor(nfeatures, scaleFactor, nlevels)
{
    // nlevels=1 prevents cv::ORB from building its own internal pyramid; we
    // supply each level image individually in detect() and computeDescriptors().
    // scaleFactor=1.0f: no additional scaling inside cv::ORB (the image is
    // already at the correct scale for this pyramid level).
    // edgeThreshold=EDGE_THRESHOLD: matches the border padding of the base
    // class pyramid so keypoints are not detected in the border region.
    //
    // nfeatures budget for the internal ORB detector: cv::ORB::create(0,...) does NOT
    // mean unlimited — it sets the per-level budget to 0 and returns zero keypoints.
    // We set a generous budget (5× the total desired features) so detect() returns
    // plenty of raw candidates for distributeOctTree() to select from.
    const int candidateBudget = std::max(nfeatures * 5, 5000);
    mORB = cv::ORB::create(/*nfeatures=*/candidateBudget, /*scaleFactor=*/1.0f, /*nlevels=*/1,
                           /*edgeThreshold=*/EDGE_THRESHOLD, /*firstLevel=*/0,
                           /*WTA_K=*/2, /*scoreType=*/scoreType,
                           /*patchSize=*/31, /*fastThreshold=*/fastThreshold);
}

void VanillaORBFeatureExtractor::detect(std::vector<std::vector<cv::KeyPoint>>& allKeypoints)
{
    allKeypoints.resize(mnLevels);
    for (int level = 0; level < mnLevels; ++level)
    {
        std::vector<cv::KeyPoint> candidates;
        mORB->detect(mvImagePyramid[level], candidates);

        const int w = mvImagePyramid[level].cols;
        const int h = mvImagePyramid[level].rows;
        allKeypoints[level] = distributeOctTree(candidates, 0, w, 0, h, mnFeaturesPerLevel[level], level);
        for (auto& kp : allKeypoints[level])
        {
            kp.octave = level;
        }
    }
}

void VanillaORBFeatureExtractor::computeOrientation(const cv::Mat& /*levelImage*/,
                                                    std::vector<cv::KeyPoint>& /*levelKeypoints*/)
{
    // cv::ORB::detect() already computes kp.angle — nothing to do here.
}

void VanillaORBFeatureExtractor::computeDescriptors(const cv::Mat& workingMat,
                                                    std::vector<cv::KeyPoint>& levelKeypoints, cv::Mat& descriptors)
{
    mORB->compute(workingMat, levelKeypoints, descriptors);
}

}  // namespace ORB_SLAM3
