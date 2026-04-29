#pragma once

#include "feature_extractor/FeatureExtractor.h"

#include <opencv2/features2d.hpp>

namespace ORB_SLAM3
{

// BRISK feature extractor using BRISK's native 2x octave scale space.
//
// Unlike the ORB extractors which operate per pyramid level, this class
// overrides operator() entirely and runs cv::BRISK::detectAndCompute() on
// the full-resolution image, letting BRISK build its own scale space.
//
// scaleFactor is hardcoded to 2.0 (BRISK's fixed octave scale).
// mvImagePyramid is still populated by buildPyramid() for Frame::ComputeStereoMatches.
//
// Descriptor format: CV_8UC1, 64 bytes/row (512 bits). Use Hamming distance.
class BRISKFeatureExtractor : public FeatureExtractor
{
public:
    // nfeatures : maximum total keypoints (kept by response score)
    // nlevels   : number of octaves for BRISK's scale space
    // threshold : AGAST corner detection threshold (lower = more keypoints)
    BRISKFeatureExtractor(int nfeatures, int nlevels, int threshold = 30);

    int matchThLow()  const override { return 100; }
    int matchThHigh() const override { return 200; }

    int operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints,
                   cv::OutputArray descriptors, std::vector<int>& vLappingArea) override;

protected:
    // detect/computeOrientation/computeDescriptors are not called — operator() is fully overridden.
    void detect(std::vector<std::vector<cv::KeyPoint>>&) override {}
    void computeOrientation(const cv::Mat&, std::vector<cv::KeyPoint>&) override {}
    void computeDescriptors(const cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat&) override {}

private:
    cv::Ptr<cv::BRISK> mBRISK;
};

}  // namespace ORB_SLAM3
