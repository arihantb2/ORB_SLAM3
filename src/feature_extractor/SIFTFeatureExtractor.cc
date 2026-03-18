#include "feature_extractor/SIFTFeatureExtractor.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc/imgproc.hpp>

namespace ORB_SLAM3
{

SIFTFeatureExtractor::SIFTFeatureExtractor(int nfeatures, int nOctaveLayers,
                                           double contrastThreshold, double edgeThreshold,
                                           double sigma, int nlevels)
    : FeatureExtractor(nfeatures, 2.0f, nlevels)
{
    // Pass nfeatures=0 so cv::SIFT reports all detected keypoints without
    // capping them itself.  distributeOctTree() handles the per-level budget.
    mSIFT = cv::SIFT::create(0, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
}

void SIFTFeatureExtractor::buildPyramid(const cv::Mat& image)
{
    mvImagePyramid.resize(mnLevels);
    mvImagePyramid[0] = image.clone();
    for (int level = 1; level < mnLevels; ++level)
    {
        const float invScale = mvInvScaleFactor[level];
        cv::Size sz(cvRound(static_cast<float>(image.cols) * invScale),
                    cvRound(static_cast<float>(image.rows) * invScale));
        cv::resize(mvImagePyramid[level - 1], mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);
    }
}

void SIFTFeatureExtractor::detect(std::vector<std::vector<cv::KeyPoint>>& allKeypoints)
{
    allKeypoints.assign(mnLevels, {});

    // Detect on the full-resolution image.  SIFT builds its own internal scale
    // space; returned coordinates and kp.size are in full-resolution units.
    std::vector<cv::KeyPoint> candidates;
    mSIFT->detect(mvImagePyramid[0], candidates);

    // Group candidates into our pyramid levels.
    // SIFT encodes its internal octave in the low byte of kp.octave.
    // Octave -1 (encoded as 0xFF = 255) is the "pre-upsampled" pass — map to
    // level 0 since it represents sub-pixel scale smaller than level 0.
    std::vector<std::vector<cv::KeyPoint>> perLevel(mnLevels);
    for (const cv::KeyPoint& kp : candidates)
    {
        int oct = kp.octave & 0xFF;
        if (oct >= 128)
            oct -= 256;  // sign-extend: octave -1 stored as 255
        oct = std::max(0, std::min(oct, mnLevels - 1));

        // Convert full-res coordinates to level-local coordinates.
        cv::KeyPoint localKp = kp;
        localKp.pt.x *= mvInvScaleFactor[oct];
        localKp.pt.y *= mvInvScaleFactor[oct];
        localKp.octave = oct;
        perLevel[oct].push_back(localKp);
    }

    // Per-level spatial distribution via QuadTree (mirrors ORB's dual-threshold
    // strategy). Each leaf retains the keypoint with the highest SIFT response.
    for (int level = 0; level < mnLevels; ++level)
    {
        if (perLevel[level].empty())
        {
            allKeypoints[level].clear();
            continue;
        }
        const int w = mvImagePyramid[level].cols;
        const int h = mvImagePyramid[level].rows;
        allKeypoints[level] = distributeOctTree(perLevel[level], 0, w, 0, h,
                                                mnFeaturesPerLevel[level], level);
        for (auto& kp : allKeypoints[level])
            kp.octave = level;
    }
}

void SIFTFeatureExtractor::computeOrientation(const cv::Mat& /*levelImage*/,
                                              std::vector<cv::KeyPoint>& /*levelKeypoints*/)
{
    // cv::SIFT::detect() already sets kp.angle — nothing to do here.
}

void SIFTFeatureExtractor::computeDescriptors(const cv::Mat& /*workingMat*/,
                                              std::vector<cv::KeyPoint>& levelKeypoints,
                                              cv::Mat& descriptors)
{
    if (levelKeypoints.empty())
    {
        descriptors = cv::Mat();
        return;
    }

    // kp.pt is in level-local coordinates; kp.size was set by SIFT detect in
    // full-resolution units.  cv::SIFT::compute() must receive keypoints in
    // the same coordinate space as the image it is given.  We therefore
    // up-scale pt back to full-resolution and compute on mvImagePyramid[0].
    const int level = levelKeypoints.front().octave;
    const float scale = mvScaleFactor[level];

    std::vector<cv::KeyPoint> fullResKps = levelKeypoints;
    for (auto& kp : fullResKps)
        kp.pt *= scale;

    mSIFT->compute(mvImagePyramid[0], fullResKps, descriptors);
}

}  // namespace ORB_SLAM3
