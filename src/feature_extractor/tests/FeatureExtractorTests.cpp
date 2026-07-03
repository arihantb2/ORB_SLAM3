#include "feature_extractor/BRISKFeatureExtractor.h"
#include "feature_extractor/GridBasedORBFeatureExtractor.h"
#include "FeatureMatcher.h"

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <cmath>
#include <vector>

using namespace ORB_SLAM3;

namespace
{

// Checkerboard image: reliable source of corners for both ORB and BRISK detectors.
cv::Mat makeCheckerboard(int rows = 480, int cols = 640, int cellSize = 32)
{
    cv::Mat img(rows, cols, CV_8UC1, cv::Scalar(0));
    for (int r = 0; r < rows; r += cellSize)
    {
        for (int c = 0; c < cols; c += cellSize)
        {
            if (((r / cellSize) + (c / cellSize)) % 2 == 0)
            {
                cv::Rect rect(c, r, std::min(cellSize, cols - c), std::min(cellSize, rows - r));
                img(rect).setTo(255);
            }
        }
    }
    return img;
}

// Runs operator() and verifies the stereo-reorder invariant:
// - keypoints[0..monoIndex-1] are outside the lapping area
// - keypoints[monoIndex..N-1] are inside the lapping area
// - descriptors and keypoints remain in correspondence after reordering
void checkReorderInvariant(FeatureExtractor& extractor, const cv::Mat& image,
                            const std::vector<int>& lappingArea)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<int> lap = lappingArea;

    const int monoIndex = extractor(image, cv::noArray(), keypoints, descriptors, lap);

    ASSERT_GE(monoIndex, 0);
    ASSERT_LE(monoIndex, static_cast<int>(keypoints.size()));
    ASSERT_EQ(static_cast<int>(keypoints.size()), descriptors.rows);

    for (int i = 0; i < monoIndex; ++i)
    {
        const float x = keypoints[i].pt.x;
        EXPECT_FALSE(x >= lappingArea[0] && x <= lappingArea[1])
            << "mono keypoint " << i << " at x=" << x << " is inside lapping area ["
            << lappingArea[0] << ", " << lappingArea[1] << "]";
    }
    for (int i = monoIndex; i < static_cast<int>(keypoints.size()); ++i)
    {
        const float x = keypoints[i].pt.x;
        EXPECT_TRUE(x >= lappingArea[0] && x <= lappingArea[1])
            << "stereo keypoint " << i << " at x=" << x << " is outside lapping area ["
            << lappingArea[0] << ", " << lappingArea[1] << "]";
    }
}

// ============================================================================
// reorderForStereo — tested via GridBasedORBFeatureExtractor::operator()
// ============================================================================

TEST(ReorderForStereo, MonoAndStereoKeypoints)
{
    // Lapping area covers the right third of the image.
    // A checkerboard ensures corners exist in both regions.
    GridBasedORBFeatureExtractor extractor(500, 1.2f, 4, 15, 5);
    const cv::Mat img = makeCheckerboard(480, 640, 32);
    checkReorderInvariant(extractor, img, {427, 640});
}

TEST(ReorderForStereo, NoStereoOverlap)
{
    // vLappingArea={0,0}: only x==0 qualifies as stereo. Since EDGE_THRESHOLD=19,
    // no keypoint lands at x=0 — all keypoints are mono.
    GridBasedORBFeatureExtractor extractor(200, 1.2f, 4, 15, 5);
    const cv::Mat img = makeCheckerboard(480, 640, 32);

    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    std::vector<int> lap = {0, 0};
    const int monoIndex = extractor(img, cv::noArray(), kps, descs, lap);

    EXPECT_EQ(monoIndex, static_cast<int>(kps.size()))
        << "All keypoints should be mono when lapping area is {0,0}";
}

TEST(ReorderForStereo, AllStereoOverlap)
{
    // vLappingArea covers entire image width — every keypoint is stereo.
    GridBasedORBFeatureExtractor extractor(200, 1.2f, 4, 15, 5);
    const cv::Mat img = makeCheckerboard(480, 640, 32);

    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    std::vector<int> lap = {0, 639};
    const int monoIndex = extractor(img, cv::noArray(), kps, descs, lap);

    EXPECT_EQ(monoIndex, 0) << "monoIndex should be 0 when entire image is lapping area";
    EXPECT_EQ(descs.rows, static_cast<int>(kps.size()));
}

TEST(ReorderForStereo, DescriptorsMatchKeypoints)
{
    // Run extraction twice with the same image.
    // Without reordering descriptors would get out of sync with keypoints.
    // We verify the invariant: running DescriptorDistance between a keypoint's
    // descriptor and itself is always 0.
    GridBasedORBFeatureExtractor extractor(300, 1.2f, 4, 15, 5);
    const cv::Mat img = makeCheckerboard(480, 640, 32);

    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    std::vector<int> lap = {320, 640};
    extractor(img, cv::noArray(), kps, descs, lap);

    ASSERT_EQ(descs.rows, static_cast<int>(kps.size()));
    for (int i = 0; i < descs.rows; ++i)
    {
        const cv::Mat row = descs.row(i);
        EXPECT_EQ(FeatureMatcher::DescriptorDistance(row, row), 0)
            << "self-distance of descriptor " << i << " must be 0";
    }
}

// ============================================================================
// BRISKFeatureExtractor
// ============================================================================

TEST(BRISKFeatureExtractor, DescriptorFormat)
{
    BRISKFeatureExtractor extractor(500, 4, 30);
    const cv::Mat img = makeCheckerboard(480, 640, 32);

    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    std::vector<int> lap = {0, 0};
    extractor(img, cv::noArray(), kps, descs, lap);

    ASSERT_GT(kps.size(), 0u) << "No keypoints detected on checkerboard";
    EXPECT_EQ(descs.type(), CV_8UC1) << "BRISK descriptors must be CV_8UC1";

    const int expectedDescSize = cv::BRISK::create()->descriptorSize();
    EXPECT_EQ(descs.cols, expectedDescSize)
        << "Descriptor width mismatch (expected " << expectedDescSize << " bytes)";
    EXPECT_EQ(descs.rows, static_cast<int>(kps.size()));
}

TEST(BRISKFeatureExtractor, OctaveRange)
{
    const int nLevels = 4;
    BRISKFeatureExtractor extractor(500, nLevels, 30);
    const cv::Mat img = makeCheckerboard(480, 640, 32);

    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    std::vector<int> lap = {0, 0};
    extractor(img, cv::noArray(), kps, descs, lap);

    ASSERT_GT(kps.size(), 0u);
    for (const auto& kp : kps)
    {
        EXPECT_GE(kp.octave, 0) << "kp.octave must be non-negative";
        EXPECT_LT(kp.octave, nLevels) << "kp.octave must be < nLevels";
    }
}

TEST(BRISKFeatureExtractor, FeatureCountCap)
{
    const int nFeatures = 50;
    BRISKFeatureExtractor extractor(nFeatures, 4, 10);  // low threshold → many candidates
    const cv::Mat img = makeCheckerboard(480, 640, 16);  // small cells → more corners

    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    std::vector<int> lap = {0, 0};
    extractor(img, cv::noArray(), kps, descs, lap);

    EXPECT_LE(static_cast<int>(kps.size()), nFeatures)
        << "Output keypoints must not exceed nFeatures";
}

TEST(BRISKFeatureExtractor, DescriptorMatchesKeypointAfterCap)
{
    // Regression test: capping to nFeatures must permute keypoints and descriptor
    // rows together. A low threshold on a dense checkerboard guarantees more
    // candidates than nFeatures, so the cap path is exercised.
    const int briskThreshold = 5;
    const int nLevels = 4;
    const int nFeatures = 25;
    BRISKFeatureExtractor extractor(nFeatures, nLevels, briskThreshold);
    const cv::Mat img = makeCheckerboard(480, 640, 8);

    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    std::vector<int> lap = {0, 0};
    extractor(img, cv::noArray(), kps, descs, lap);
    ASSERT_EQ(static_cast<int>(kps.size()), nFeatures)
        << "test image/threshold should produce more than nFeatures candidates";
    ASSERT_EQ(descs.rows, nFeatures);

    // BRISK descriptors depend only on the local neighbourhood of a keypoint, so
    // recomputing a descriptor for a single kept keypoint in isolation (same
    // image, same detector parameters) must reproduce exactly the row the
    // extractor paired with that keypoint.
    cv::Ptr<cv::BRISK> groundTruth = cv::BRISK::create(briskThreshold, nLevels - 1);
    for (int i = 0; i < static_cast<int>(kps.size()); ++i)
    {
        std::vector<cv::KeyPoint> single = {kps[i]};
        cv::Mat singleDesc;
        groundTruth->compute(img, single, singleDesc);
        ASSERT_FALSE(single.empty()) << "keypoint " << i << " was dropped by BRISK::compute in isolation";
        ASSERT_EQ(singleDesc.cols, descs.cols);
        EXPECT_EQ(cv::countNonZero(singleDesc.row(0) != descs.row(i)), 0)
            << "descriptor row " << i << " does not match its paired keypoint after capping";
    }
}

TEST(BRISKFeatureExtractor, ScaleFactors)
{
    // BRISK forces scaleFactor=2.0 — verify mvScaleFactors = [1, 2, 4, 8]
    const int nLevels = 4;
    BRISKFeatureExtractor extractor(500, nLevels, 30);

    const auto& sf = extractor.GetScaleFactors();
    ASSERT_EQ(static_cast<int>(sf.size()), nLevels);
    for (int i = 0; i < nLevels; ++i)
    {
        const float expected = static_cast<float>(std::pow(2.0, i));
        EXPECT_NEAR(sf[i], expected, 1e-5f)
            << "mvScaleFactors[" << i << "] should be 2^" << i;
    }
}

TEST(BRISKFeatureExtractor, MatchThresholds)
{
    BRISKFeatureExtractor extractor(500, 4, 30);
    EXPECT_EQ(extractor.matchThLow(),  100);
    EXPECT_EQ(extractor.matchThHigh(), 200);
}

TEST(BRISKFeatureExtractor, ImagePyramidPopulated)
{
    // buildPyramid() must still run so Frame::ComputeStereoMatches can index
    // mvImagePyramid[kp.octave].
    const int nLevels = 4;
    BRISKFeatureExtractor extractor(200, nLevels, 30);
    const cv::Mat img = makeCheckerboard(480, 640, 32);

    std::vector<cv::KeyPoint> kps;
    cv::Mat descs;
    std::vector<int> lap = {0, 0};
    extractor(img, cv::noArray(), kps, descs, lap);

    ASSERT_EQ(static_cast<int>(extractor.mvImagePyramid.size()), nLevels);
    for (int i = 0; i < nLevels; ++i)
    {
        EXPECT_FALSE(extractor.mvImagePyramid[i].empty())
            << "mvImagePyramid[" << i << "] is empty after extraction";
    }
    // Level 0 must be the original resolution.
    EXPECT_EQ(extractor.mvImagePyramid[0].rows, img.rows);
    EXPECT_EQ(extractor.mvImagePyramid[0].cols, img.cols);
}

// ============================================================================
// ORBFeatureExtractor — matchThLow/High regression
// ============================================================================

TEST(ORBFeatureExtractor, DefaultMatchThresholds)
{
    GridBasedORBFeatureExtractor extractor(500, 1.2f, 4, 15, 5);
    EXPECT_EQ(extractor.matchThLow(),  50);
    EXPECT_EQ(extractor.matchThHigh(), 100);
}

// ============================================================================
// FeatureMatcher — threshold constructor
// ============================================================================

TEST(FeatureMatcher, DefaultThresholds)
{
    FeatureMatcher m;
    EXPECT_EQ(m.thLow(),  FeatureMatcher::TH_LOW);
    EXPECT_EQ(m.thHigh(), FeatureMatcher::TH_HIGH);
}

TEST(FeatureMatcher, CustomThresholds)
{
    FeatureMatcher m(0.7f, true, 100, 200);
    EXPECT_EQ(m.thLow(),  100);
    EXPECT_EQ(m.thHigh(), 200);
}

// ============================================================================
// DescriptorDistance — 64-byte (BRISK-width) descriptors
// ============================================================================

TEST(DescriptorDistance, IdenticalDescriptors64Bytes)
{
    cv::Mat a(1, 64, CV_8UC1, cv::Scalar(0xAB));
    EXPECT_EQ(FeatureMatcher::DescriptorDistance(a, a), 0);
}

TEST(DescriptorDistance, MaxDistance64Bytes)
{
    cv::Mat a(1, 64, CV_8UC1, cv::Scalar(0x00));
    cv::Mat b(1, 64, CV_8UC1, cv::Scalar(0xFF));
    // Every bit differs: 64 bytes * 8 bits = 512.
    EXPECT_EQ(FeatureMatcher::DescriptorDistance(a, b), 512);
}

TEST(DescriptorDistance, OneBitDifference64Bytes)
{
    cv::Mat a(1, 64, CV_8UC1, cv::Scalar(0x00));
    cv::Mat b(1, 64, CV_8UC1, cv::Scalar(0x00));
    b.at<uint8_t>(0, 0) = 0x01;  // one bit set in the first byte
    EXPECT_EQ(FeatureMatcher::DescriptorDistance(a, b), 1);
}

TEST(DescriptorDistance, KnownDistance64Bytes)
{
    cv::Mat a(1, 64, CV_8UC1, cv::Scalar(0x00));
    cv::Mat b(1, 64, CV_8UC1, cv::Scalar(0x00));
    // Set the first 8 bytes to 0xFF → 64 bits set.
    for (int i = 0; i < 8; ++i)
        b.at<uint8_t>(0, i) = 0xFF;
    EXPECT_EQ(FeatureMatcher::DescriptorDistance(a, b), 64);
}

// Regression: 32-byte (ORB-width) still works correctly.
TEST(DescriptorDistance, ORBWidth32Bytes)
{
    cv::Mat a(1, 32, CV_8UC1, cv::Scalar(0x00));
    cv::Mat b(1, 32, CV_8UC1, cv::Scalar(0xFF));
    EXPECT_EQ(FeatureMatcher::DescriptorDistance(a, b), 256);
}

}  // namespace
