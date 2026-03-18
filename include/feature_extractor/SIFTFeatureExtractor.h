#pragma once

#include "feature_extractor/FeatureExtractor.h"
#include "feature_extractor/FeatureTypes.h"

#include <opencv2/features2d.hpp>

namespace ORB_SLAM3
{

// SIFT specialization of FeatureExtractor.
//
// Detection strategy:
//   cv::SIFT::detect() runs on the full-resolution pyramid level 0 image,
//   building its own Gaussian scale space internally. Detected keypoints are
//   grouped by their SIFT octave index (decoded from kp.octave & 0xFF) into
//   our pyramid levels, then spatially distributed with distributeOctTree()
//   per level to achieve uniform coverage.
//
// Scale factor:
//   SIFT natively uses 2× octave downscaling, so the base class must be
//   initialised with scaleFactor=2.0f. Pyramid level coordinates are therefore
//   at half the resolution of level 0 per level step.
//
// Descriptor format: CV_32F, 128 floats/row. Use L2 distance for matching.
class SIFTFeatureExtractor : public FeatureExtractor
{
public:
    // nfeatures        : desired total keypoints across all pyramid levels
    // nOctaveLayers    : SIFT sub-octave (DoG) layers per octave (default 3)
    // contrastThreshold: min DoG response; lower value = more candidates
    //                    (0.03 is below SIFT's default 0.04 — intentional to
    //                     feed distributeOctTree with more raw candidates)
    // edgeThreshold    : principal-curvature ratio for edge rejection
    // sigma            : Gaussian sigma applied to level-0 image before DoG
    // nlevels          : number of pyramid levels (== number of SIFT octaves)
    SIFTFeatureExtractor(int nfeatures, int nOctaveLayers = 3,
                         double contrastThreshold = 0.03,
                         double edgeThreshold = 10.0,
                         double sigma = 1.6,
                         int nlevels = 4);

    DescriptorType getDescriptorType() const override { return DescriptorType::FLOAT32; }

protected:
    // Simple 2× per-level downscale; no EDGE_THRESHOLD border padding needed.
    void buildPyramid(const cv::Mat& image) override;

    // Run SIFT detect on mvImagePyramid[0], group candidates by octave, then
    // distributeOctTree per level. Sets kp.octave = pyramid level index.
    void detect(std::vector<std::vector<cv::KeyPoint>>& allKeypoints) override;

    // No-op: cv::SIFT::detect() already writes kp.angle.
    void computeOrientation(const cv::Mat& levelImage,
                            std::vector<cv::KeyPoint>& levelKeypoints) override;

    // Computes 128-float SIFT descriptors for one pyramid level.
    // Uses mvImagePyramid[0] with full-resolution keypoint coordinates so that
    // kp.size (set in full-res space by SIFT detect) is consistent with the
    // image passed to compute().
    // Output: descriptors is CV_32F, rows = levelKeypoints.size(), cols = 128.
    // Note: workingMat (pre-blurred level image from base class) is unused here.
    void computeDescriptors(const cv::Mat& workingMat,
                            std::vector<cv::KeyPoint>& levelKeypoints,
                            cv::Mat& descriptors) override;

private:
    cv::Ptr<cv::SIFT> mSIFT;
};

}  // namespace ORB_SLAM3
