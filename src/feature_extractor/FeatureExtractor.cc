#include "feature_extractor/FeatureExtractor.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <list>

namespace ORB_SLAM3
{

// Padding used when building the pyramid. Matches the original ORBextractor.
static constexpr int EDGE_THRESHOLD = 19;

// ============================================================================
// ExtractorNode
// ============================================================================

void ExtractorNode::DivideNode(ExtractorNode& n1, ExtractorNode& n2, ExtractorNode& n3, ExtractorNode& n4)
{
    const int halfX = static_cast<int>(std::ceil(static_cast<float>(UR.x - UL.x) / 2.f));
    const int halfY = static_cast<int>(std::ceil(static_cast<float>(BR.y - UL.y) / 2.f));

    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x + halfX, UL.y);
    n1.BL = cv::Point2i(UL.x, UL.y + halfY);
    n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x, UL.y + halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x, BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    for (const cv::KeyPoint& kp : vKeys)
    {
        if (kp.pt.x < n1.UR.x)
        {
            if (kp.pt.y < n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else
        {
            if (kp.pt.y < n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }
    }

    if (n1.vKeys.size() == 1)
        n1.bNoMore = true;
    if (n2.vKeys.size() == 1)
        n2.bNoMore = true;
    if (n3.vKeys.size() == 1)
        n3.bNoMore = true;
    if (n4.vKeys.size() == 1)
        n4.bNoMore = true;
}

// ============================================================================
// Local helper
// ============================================================================

static bool compareNodes(std::pair<int, ExtractorNode*>& e1, std::pair<int, ExtractorNode*>& e2)
{
    if (e1.first < e2.first)
        return true;
    if (e1.first > e2.first)
        return false;
    return e1.second->UL.x < e2.second->UL.x;
}

// ============================================================================
// FeatureExtractor
// ============================================================================

FeatureExtractor::FeatureExtractor(int nfeatures, float scaleFactor, int nlevels)
    : mnFeatures(nfeatures), mfScaleFactor(scaleFactor), mnLevels(nlevels)
{
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0] = 1.0f;
    mvLevelSigma2[0] = 1.0f;
    for (int i = 1; i < nlevels; i++)
    {
        mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;
        mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for (int i = 0; i < nlevels; i++)
    {
        mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
        mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
    }

    mvImagePyramid.resize(nlevels);
    computeFeaturesPerLevel();
}

void FeatureExtractor::computeFeaturesPerLevel()
{
    mnFeaturesPerLevel.resize(mnLevels);
    const float factor = 1.0f / mfScaleFactor;
    float nDesiredFeaturesPerScale =
        mnFeatures * (1.f - factor) /
        (1.f - static_cast<float>(std::pow(static_cast<double>(factor), static_cast<double>(mnLevels))));

    int sumFeatures = 0;
    for (int level = 0; level < mnLevels - 1; level++)
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[mnLevels - 1] = std::max(mnFeatures - sumFeatures, 0);
}

// Default pyramid: Gaussian downscale with EDGE_THRESHOLD=19 border padding.
// Identical to ORBextractor::ComputePyramid().
void FeatureExtractor::buildPyramid(const cv::Mat& image)
{
    mvImagePyramid.resize(mnLevels);
    for (int level = 0; level < mnLevels; ++level)
    {
        const float scale = mvInvScaleFactor[level];
        cv::Size sz(cvRound(static_cast<float>(image.cols) * scale), cvRound(static_cast<float>(image.rows) * scale));
        cv::Size wholeSize(sz.width + EDGE_THRESHOLD * 2, sz.height + EDGE_THRESHOLD * 2);
        cv::Mat temp(wholeSize, image.type());
        mvImagePyramid[level] = temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));
        if (level != 0)
        {
            cv::resize(mvImagePyramid[level - 1], mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);
            cv::copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               EDGE_THRESHOLD, cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);
        }
        else
        {
            cv::copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               cv::BORDER_REFLECT_101);
        }
    }
}

int FeatureExtractor::operator()(cv::InputArray _image, cv::InputArray /*_mask*/, std::vector<cv::KeyPoint>& _keypoints,
                                 cv::OutputArray _descriptors, std::vector<int>& vLappingArea)
{
    if (_image.empty())
        return -1;

    cv::Mat image = _image.getMat();
    assert(image.type() == CV_8UC1);

    // 1. Build image pyramid
    buildPyramid(image);

    // 2. Detect keypoints (per level, in level-local coordinates, no angle yet)
    std::vector<std::vector<cv::KeyPoint>> allKeypoints;
    detect(allKeypoints);

    // 3. Compute orientations per level
    for (int level = 0; level < mnLevels; ++level)
        computeOrientation(mvImagePyramid[level], allKeypoints[level]);

    // 4. Gaussian blur + compute descriptors per level
    std::vector<cv::Mat> perLevelDescs(mnLevels);
    int nkeypoints = 0;
    for (int level = 0; level < mnLevels; ++level)
    {
        if (allKeypoints[level].empty())
            continue;

        cv::Mat workingMat = mvImagePyramid[level].clone();
        cv::GaussianBlur(workingMat, workingMat, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);

        computeDescriptors(workingMat, allKeypoints[level], perLevelDescs[level]);
        nkeypoints += static_cast<int>(allKeypoints[level].size());
    }

    if (nkeypoints == 0)
    {
        _descriptors.release();
        _keypoints.clear();
        return 0;
    }

    // 5. Allocate output — derive format from first non-empty level descriptor
    int descCols = -1, descType = -1;
    for (const auto& d : perLevelDescs)
    {
        if (!d.empty())
        {
            descCols = d.cols;
            descType = d.type();
            break;
        }
    }

    _descriptors.create(nkeypoints, descCols, descType);
    cv::Mat descriptors = _descriptors.getMat();
    _keypoints.resize(nkeypoints);

    // 6. Collect keypoints into flat output, scaling coordinates to level-0.
    int idx = 0;
    for (int level = 0; level < mnLevels; ++level)
    {
        std::vector<cv::KeyPoint>& keypoints = allKeypoints[level];
        if (keypoints.empty())
            continue;

        const float scale = mvScaleFactor[level];
        int i = 0;
        for (cv::KeyPoint& kp : keypoints)
        {
            if (level != 0)
                kp.pt *= scale;
            _keypoints[idx] = kp;
            perLevelDescs[level].row(i).copyTo(descriptors.row(idx));
            ++idx;
            ++i;
        }
    }

    // 7. Stereo reordering: mono keypoints to front, stereo to back.
    return reorderForStereo(_keypoints, descriptors, vLappingArea);
}

// ============================================================================
// FeatureExtractor::distributeOctTree
// Migrated from ORBextractor::DistributeOctTree() — logic is unchanged.
// ============================================================================

std::vector<cv::KeyPoint> FeatureExtractor::distributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys,
                                                              int minX, int maxX, int minY, int maxY, int nFeatures,
                                                              int /*level*/) const
{
    const int nIni = static_cast<int>(std::round(static_cast<float>(maxX - minX) / static_cast<float>(maxY - minY)));
    const float hX = static_cast<float>(maxX - minX) / nIni;

    std::list<ExtractorNode> lNodes;
    std::vector<ExtractorNode*> vpIniNodes(nIni);

    for (int i = 0; i < nIni; i++)
    {
        ExtractorNode ni;
        ni.UL = cv::Point2i(static_cast<int>(hX * i), 0);
        ni.UR = cv::Point2i(static_cast<int>(hX * (i + 1)), 0);
        ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
        ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
        ni.vKeys.reserve(vToDistributeKeys.size());
        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    for (const cv::KeyPoint& kp : vToDistributeKeys)
        vpIniNodes[static_cast<int>(kp.pt.x / hX)]->vKeys.push_back(kp);

    {
        auto lit = lNodes.begin();
        while (lit != lNodes.end())
        {
            if (lit->vKeys.size() == 1)
            {
                lit->bNoMore = true;
                ++lit;
            }
            else if (lit->vKeys.empty())
            {
                lit = lNodes.erase(lit);
            }
            else
            {
                ++lit;
            }
        }
    }

    bool bFinish = false;
    std::vector<std::pair<int, ExtractorNode*>> vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size() * 4);

    while (!bFinish)
    {
        const int prevSize = static_cast<int>(lNodes.size());
        auto lit = lNodes.begin();
        int nToExpand = 0;
        vSizeAndPointerToNode.clear();

        while (lit != lNodes.end())
        {
            if (lit->bNoMore)
            {
                ++lit;
                continue;
            }
            ExtractorNode n1, n2, n3, n4;
            lit->DivideNode(n1, n2, n3, n4);

            auto addChild = [&](ExtractorNode& n)
            {
                if (!n.vKeys.empty())
                {
                    lNodes.push_front(n);
                    if (n.vKeys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back({static_cast<int>(n.vKeys.size()), &lNodes.front()});
                        lNodes.front().lit = lNodes.begin();
                    }
                }
            };
            addChild(n1);
            addChild(n2);
            addChild(n3);
            addChild(n4);

            lit = lNodes.erase(lit);
        }

        if (static_cast<int>(lNodes.size()) >= nFeatures || static_cast<int>(lNodes.size()) == prevSize)
        {
            bFinish = true;
        }
        else if ((static_cast<int>(lNodes.size()) + nToExpand * 3) > nFeatures)
        {
            while (!bFinish)
            {
                const int prevSize2 = static_cast<int>(lNodes.size());
                std::vector<std::pair<int, ExtractorNode*>> vPrev = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();
                std::sort(vPrev.begin(), vPrev.end(), compareNodes);

                for (int j = static_cast<int>(vPrev.size()) - 1; j >= 0; j--)
                {
                    ExtractorNode n1, n2, n3, n4;
                    vPrev[j].second->DivideNode(n1, n2, n3, n4);

                    auto addChild2 = [&](ExtractorNode& n)
                    {
                        if (!n.vKeys.empty())
                        {
                            lNodes.push_front(n);
                            if (n.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back({static_cast<int>(n.vKeys.size()), &lNodes.front()});
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                    };
                    addChild2(n1);
                    addChild2(n2);
                    addChild2(n3);
                    addChild2(n4);

                    lNodes.erase(vPrev[j].second->lit);

                    if (static_cast<int>(lNodes.size()) >= nFeatures)
                        break;
                }

                if (static_cast<int>(lNodes.size()) >= nFeatures || static_cast<int>(lNodes.size()) == prevSize2)
                    bFinish = true;
            }
        }
    }

    // Retain the keypoint with the highest FAST response from each leaf node.
    std::vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(nFeatures);
    for (auto& node : lNodes)
    {
        const cv::KeyPoint* pBest = &node.vKeys[0];
        for (size_t k = 1; k < node.vKeys.size(); k++)
        {
            if (node.vKeys[k].response > pBest->response)
                pBest = &node.vKeys[k];
        }
        vResultKeys.push_back(*pBest);
    }
    return vResultKeys;
}

int FeatureExtractor::reorderForStereo(std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors,
                                       const std::vector<int>& vLappingArea) const
{
    const int n = static_cast<int>(keypoints.size());
    std::vector<cv::KeyPoint> reordered(n);
    cv::Mat reorderedDescs(n, descriptors.cols, descriptors.type());

    int monoIndex = 0;
    int stereoIndex = n - 1;

    for (int i = 0; i < n; ++i)
    {
        const cv::KeyPoint& kp = keypoints[i];
        if (kp.pt.x >= vLappingArea[0] && kp.pt.x <= vLappingArea[1])
        {
            reordered[stereoIndex] = kp;
            descriptors.row(i).copyTo(reorderedDescs.row(stereoIndex));
            --stereoIndex;
        }
        else
        {
            reordered[monoIndex] = kp;
            descriptors.row(i).copyTo(reorderedDescs.row(monoIndex));
            ++monoIndex;
        }
    }

    keypoints = std::move(reordered);
    reorderedDescs.copyTo(descriptors);
    return monoIndex;
}

}  // namespace ORB_SLAM3
