#include "feature_extractor/BRISKFeatureExtractor.h"

#include <algorithm>
#include <cassert>
#include <numeric>

namespace ORB_SLAM3
{

BRISKFeatureExtractor::BRISKFeatureExtractor(int nfeatures, int nlevels, int threshold)
    : FeatureExtractor(nfeatures, /*scaleFactor=*/2.0f, nlevels),
      mBRISK(cv::BRISK::create(threshold, /*octaves=*/nlevels - 1))
{
}

int BRISKFeatureExtractor::operator()(cv::InputArray _image, cv::InputArray /*_mask*/,
                                      std::vector<cv::KeyPoint>& _keypoints,
                                      cv::OutputArray _descriptors,
                                      std::vector<int>& vLappingArea)
{
    if (_image.empty())
        return -1;

    cv::Mat image = _image.getMat();
    assert(image.type() == CV_8UC1);

    // Build pyramid so Frame::ComputeStereoMatches can index mvImagePyramid[kp.octave].
    // scaleFactor=2.0 ensures each level matches BRISK's octave scale.
    buildPyramid(image);

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    mBRISK->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

    if (keypoints.empty())
    {
        _descriptors.release();
        _keypoints.clear();
        return 0;
    }

    // BRISK may encode an inter-octave layer in the upper bits of kp.octave.
    // Extract the clean octave index and clamp to the valid pyramid range.
    for (auto& kp : keypoints)
    {
        kp.octave = std::clamp(kp.octave & 0xFF, 0, mnLevels - 1);
        CV_DbgAssert(kp.octave >= 0 && kp.octave < mnLevels);
    }

    // Cap total keypoints at mnFeatures, keeping the highest-response ones.
    // Keypoints and descriptor rows share an index into the original detection
    // order, so the same permutation must be applied to both — sorting only
    // `keypoints` (as before) would silently re-pair each survivor with an
    // unrelated descriptor row.
    if (static_cast<int>(keypoints.size()) > mnFeatures)
    {
        std::vector<int> order(keypoints.size());
        std::iota(order.begin(), order.end(), 0);
        std::partial_sort(order.begin(), order.begin() + mnFeatures, order.end(),
                          [&keypoints](int a, int b) { return keypoints[a].response > keypoints[b].response; });
        order.resize(mnFeatures);

        std::vector<cv::KeyPoint> keptKeypoints;
        keptKeypoints.reserve(mnFeatures);
        cv::Mat keptDescriptors(mnFeatures, descriptors.cols, descriptors.type());
        for (int i = 0; i < mnFeatures; ++i)
        {
            keptKeypoints.push_back(keypoints[order[i]]);
            descriptors.row(order[i]).copyTo(keptDescriptors.row(i));
        }
        keypoints = std::move(keptKeypoints);
        descriptors = keptDescriptors;
    }

    _descriptors.create(descriptors.rows, descriptors.cols, descriptors.type());
    descriptors.copyTo(_descriptors.getMat());
    _keypoints = std::move(keypoints);

    cv::Mat outDescs = _descriptors.getMat();
    return reorderForStereo(_keypoints, outDescs, vLappingArea);
}

}  // namespace ORB_SLAM3
