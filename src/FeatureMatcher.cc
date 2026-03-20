/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "FeatureMatcher.h"

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Verbose.h"

#include <limits.h>
#include <limits>
#include <utility>

#include <opencv2/core/core.hpp>

#include "DBoW2/FeatureVector.h"

namespace ORB_SLAM3
{
namespace
{
inline float InitRotationHistogram(std::vector<int>* rotHist, const int histoLength)
{
    for (int i = 0; i < histoLength; i++)
    {
        rotHist[i].reserve(500);
    }
    return 1.0f / histoLength;
}

inline void AddRotationToHistogram(std::vector<int>* rotHist, const int histoLength, const float factor,
                                   const float angle1, const float angle2, const int idx)
{
    float rot = angle1 - angle2;
    if (rot < 0.0f)
    {
        rot += 360.0f;
    }
    int bin = round(rot * factor);
    if (bin == histoLength)
    {
        bin = 0;
    }
    assert(bin >= 0 && bin < histoLength);
    rotHist[bin].push_back(idx);
}

template <typename TCompute, typename TRemove>
inline void ApplyRotationConsistency(std::vector<int>* rotHist, const int histoLength, TCompute computeMaxima,
                                     TRemove removeMatch)
{
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    computeMaxima(ind1, ind2, ind3);

    for (int i = 0; i < histoLength; i++)
    {
        if (i == ind1 || i == ind2 || i == ind3)
        {
            continue;
        }
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
        {
            removeMatch(rotHist[i][j]);
        }
    }
}

inline void UpdateBestAndSecond(const int dist, const int idx, int& bestDist, int& bestIdx, int& bestDist2)
{
    if (dist < bestDist)
    {
        bestDist2 = bestDist;
        bestDist = dist;
        bestIdx = idx;
    }
    else if (dist < bestDist2)
    {
        bestDist2 = dist;
    }
}

inline bool PassesBestSecondRatio(const int bestDist, const int bestDist2, const float nnratio)
{
    return (bestDist2 >= std::numeric_limits<int>::max()) || (static_cast<float>(bestDist) < nnratio * bestDist2);
}

enum class ProjectionStatus
{
    kOk,
    kNegativeDepth,
    kOutOfImage,
    kOutOfDistance,
    kBadViewingAngle,
    kNoCandidates
};

struct ProjectionMatchInput
{
    Eigen::Vector2f uv;
    Eigen::Vector3f p3Dc;
    float invz = 0.0f;
    float radius = 0.0f;
    int predictedLevel = -1;
    std::vector<size_t> indices;
};

inline ProjectionStatus TryProjectMapPointToKeyFrame(MapPoint* pMP, KeyFrame* pKF, GeometricCamera* camera,
                                                     const Sophus::SE3f& Tcw, const Eigen::Vector3f& Ow, float th,
                                                     bool useRight, bool checkViewingAngle, ProjectionMatchInput& out)
{
    Eigen::Vector3f p3Dw = pMP->GetWorldPos();
    Eigen::Vector3f p3Dc = Tcw * p3Dw;

    if (p3Dc(2) < 0.0f)
    {
        return ProjectionStatus::kNegativeDepth;
    }

    const Eigen::Vector2f uv = camera->project(p3Dc);

    if (!pKF->IsInImage(uv(0), uv(1)))
    {
        return ProjectionStatus::kOutOfImage;
    }

    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    Eigen::Vector3f PO = p3Dw - Ow;
    const float dist = PO.norm();

    if (dist < minDistance || dist > maxDistance)
    {
        return ProjectionStatus::kOutOfDistance;
    }

    if (checkViewingAngle)
    {
        Eigen::Vector3f Pn = pMP->GetNormal();
        if (PO.dot(Pn) < 0.5f * dist)
        {
            return ProjectionStatus::kBadViewingAngle;
        }
    }

    const int predictedLevel = pMP->PredictScale(dist, pKF);
    const float radius = th * pKF->mvScaleFactors[predictedLevel];
    std::vector<size_t> indices = pKF->GetFeaturesInArea(uv(0), uv(1), radius, useRight);

    if (indices.empty())
    {
        return ProjectionStatus::kNoCandidates;
    }

    out.uv = uv;
    out.p3Dc = p3Dc;
    out.invz = 1.0f / p3Dc(2);
    out.predictedLevel = predictedLevel;
    out.radius = radius;
    out.indices = std::move(indices);

    return ProjectionStatus::kOk;
}

struct BestMatch
{
    int bestDist;
    int bestIdx;

    explicit BestMatch(int initialDist) : bestDist(initialDist), bestIdx(-1) {}

    void Update(int dist, int idx)
    {
        if (dist < bestDist)
        {
            bestDist = dist;
            bestIdx = idx;
        }
    }
};

struct BestTwoMatches
{
    int bestDist;
    int secondBestDist;
    int bestIdx;

    explicit BestTwoMatches(int initialDist) : bestDist(initialDist), secondBestDist(initialDist), bestIdx(-1) {}

    void Update(int dist, int idx)
    {
        if (dist < bestDist)
        {
            secondBestDist = bestDist;
            bestDist = dist;
            bestIdx = idx;
        }
        else if (dist < secondBestDist)
        {
            secondBestDist = dist;
        }
    }

    bool PassesRatio(float ratio) const
    {
        return static_cast<float>(bestDist) < ratio * static_cast<float>(secondBestDist);
    }
};

template <typename TSkip>
inline BestMatch FindBestDescriptorMatch(const std::vector<size_t>& indices, const cv::Mat& descriptors,
                                         const std::vector<cv::KeyPoint>& keys, int predictedLevel, const cv::Mat& dMP,
                                         int initialDist, TSkip shouldSkip)
{
    BestMatch bestMatch(initialDist);

    for (std::vector<size_t>::const_iterator vit = indices.begin(), vend = indices.end(); vit != vend; vit++)
    {
        const size_t idx = *vit;
        if (shouldSkip(idx))
        {
            continue;
        }

        const int& kpLevel = keys[idx].octave;
        if (kpLevel < predictedLevel - 1 || kpLevel > predictedLevel)
        {
            continue;
        }

        const cv::Mat& dKF = descriptors.row(idx);
        const int dist = FeatureMatcher::DescriptorDistance(dMP, dKF);
        bestMatch.Update(dist, idx);
    }

    return bestMatch;
}
}  // namespace

const int FeatureMatcher::TH_HIGH = 100;
const int FeatureMatcher::TH_LOW = 50;
const int FeatureMatcher::HISTO_LENGTH = 30;

int FeatureMatcher::DefaultThLow(DescriptorType descriptorType)
{
    switch (descriptorType)
    {
        case DescriptorType::BINARY:
            return TH_LOW;
        case DescriptorType::FLOAT32:
            // Empirical defaults for OpenCV SIFT descriptors (tune per dataset).
            return 200;
    }
    return TH_LOW;
}

int FeatureMatcher::DefaultThHigh(DescriptorType descriptorType)
{
    switch (descriptorType)
    {
        case DescriptorType::BINARY:
            return TH_HIGH;
        case DescriptorType::FLOAT32:
            // Empirical defaults for OpenCV SIFT descriptors (tune per dataset).
            return 400;
    }
    return TH_HIGH;
}

FeatureMatcher::FeatureMatcher(float nnratio, bool checkOri, DescriptorType descriptorType)
    : mfNNratio(nnratio),
      mbCheckOrientation(checkOri),
      mDescriptorType(descriptorType),
      mThLow(DefaultThLow(descriptorType)),
      mThHigh(DefaultThHigh(descriptorType))
{
}

int FeatureMatcher::SearchByProjection(Frame& F, const std::vector<MapPoint*>& vpMapPoints, const float th,
                                       const bool bFarPoints, const float thFarPoints)
{
    int nmatches = 0, left = 0, right = 0;

    const bool bFactor = th != 1.0;

    for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if (!pMP->mbTrackInView && !pMP->mbTrackInViewR)
        {
            continue;
        }
        if (bFarPoints && pMP->mTrackDepth > thFarPoints)
        {
            continue;
        }
        if (pMP->isBad())
        {
            continue;
        }
        if (pMP->mbTrackInView)
        {
            const int& nPredictedLevel = pMP->mnTrackScaleLevel;

            // The size of the window will depend on the viewing direction
            float r = RadiusByViewingCos(pMP->mTrackViewCos);

            if (bFactor)
            {
                r *= th;
            }
            const std::vector<size_t> vIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY, r * F.mvScaleFactors[nPredictedLevel],
                                    nPredictedLevel - 1, nPredictedLevel);

            if (!vIndices.empty())
            {
                const cv::Mat MPdescriptor = pMP->GetDescriptor();

                int bestDist = std::numeric_limits<int>::max();
                int bestLevel = -1;
                int bestDist2 = std::numeric_limits<int>::max();
                int bestLevel2 = -1;
                int bestIdx = -1;

                // Get best and second matches with near keypoints
                for (std::vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend;
                     vit++)
                {
                    const size_t idx = *vit;

                    if (F.mvpMapPoints[idx])
                    {
                        if (F.mvpMapPoints[idx]->Observations() > 0)
                        {
                            continue;
                        }
                    }
                    if (F.Nleft == -1 && F.mvuRight[idx] > 0)
                    {
                        const float er = fabs(pMP->mTrackProjXR - F.mvuRight[idx]);
                        if (er > r * F.mvScaleFactors[nPredictedLevel])
                        {
                            continue;
                        }
                    }

                    const cv::Mat& d = F.mDescriptors.row(idx);

                    const int dist = DescriptorDistance(MPdescriptor, d);

                    if (dist < bestDist)
                    {
                        bestDist2 = bestDist;
                        bestDist = dist;
                        bestLevel2 = bestLevel;
                        bestLevel = (F.Nleft == -1)   ? F.mvKeysUn[idx].octave
                                    : (idx < F.Nleft) ? F.mvKeys[idx].octave
                                                      : F.mvKeysRight[idx - F.Nleft].octave;
                        bestIdx = idx;
                    }
                    else if (dist < bestDist2)
                    {
                        bestLevel2 = (F.Nleft == -1)   ? F.mvKeysUn[idx].octave
                                     : (idx < F.Nleft) ? F.mvKeys[idx].octave
                                                       : F.mvKeysRight[idx - F.Nleft].octave;
                        bestDist2 = dist;
                    }
                }

                // Apply ratio to second match (only if best and second are in the same scale level)
                if (bestIdx >= 0 && bestDist <= mThHigh)
                {
                    if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                    {
                        continue;
                    }
                    if (bestLevel != bestLevel2 || bestDist <= mfNNratio * bestDist2)
                    {
                        F.mvpMapPoints[bestIdx] = pMP;

                        if (F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] != -1)
                        {  //Also match with the stereo observation at right camera
                            F.mvpMapPoints[F.mvLeftToRightMatch[bestIdx] + F.Nleft] = pMP;
                            nmatches++;
                            right++;
                        }

                        nmatches++;
                        left++;
                    }
                }
            }
        }

        if (F.Nleft != -1 && pMP->mbTrackInViewR)
        {
            const int& nPredictedLevel = pMP->mnTrackScaleLevelR;
            if (nPredictedLevel != -1)
            {
                float r = RadiusByViewingCos(pMP->mTrackViewCosR);

                const std::vector<size_t> vIndices =
                    F.GetFeaturesInArea(pMP->mTrackProjXR, pMP->mTrackProjYR, r * F.mvScaleFactors[nPredictedLevel],
                                        nPredictedLevel - 1, nPredictedLevel, true);

                if (vIndices.empty())
                {
                    continue;
                }
                const cv::Mat MPdescriptor = pMP->GetDescriptor();

                int bestDist = std::numeric_limits<int>::max();
                int bestLevel = -1;
                int bestDist2 = std::numeric_limits<int>::max();
                int bestLevel2 = -1;
                int bestIdx = -1;

                // Get best and second matches with near keypoints
                for (std::vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend;
                     vit++)
                {
                    const size_t idx = *vit;

                    if (F.mvpMapPoints[idx + F.Nleft])
                    {
                        if (F.mvpMapPoints[idx + F.Nleft]->Observations() > 0)
                        {
                            continue;
                        }
                    }
                    const cv::Mat& d = F.mDescriptors.row(idx + F.Nleft);

                    const int dist = DescriptorDistance(MPdescriptor, d);

                    if (dist < bestDist)
                    {
                        bestDist2 = bestDist;
                        bestDist = dist;
                        bestLevel2 = bestLevel;
                        bestLevel = F.mvKeysRight[idx].octave;
                        bestIdx = idx;
                    }
                    else if (dist < bestDist2)
                    {
                        bestLevel2 = F.mvKeysRight[idx].octave;
                        bestDist2 = dist;
                    }
                }

                // Apply ratio to second match (only if best and second are in the same scale level)
                if (bestIdx >= 0 && bestDist <= mThHigh)
                {
                    if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                    {
                        continue;
                    }
                    if (F.Nleft != -1 && F.mvRightToLeftMatch[bestIdx] != -1)
                    {  //Also match with the stereo observation at right camera
                        F.mvpMapPoints[F.mvRightToLeftMatch[bestIdx]] = pMP;
                        nmatches++;
                        left++;
                    }

                    F.mvpMapPoints[bestIdx + F.Nleft] = pMP;
                    nmatches++;
                    right++;
                }
            }
        }
    }
    return nmatches;
}

float FeatureMatcher::RadiusByViewingCos(const float& viewCos)
{
    if (viewCos > 0.998)
    {
        return 2.5;
    }
    else
    {
        return 4.0;
    }
}

int FeatureMatcher::SearchByBoW(KeyFrame* pKF, Frame& F, std::vector<MapPoint*>& vpMapPointMatches)
{
    const std::vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

    vpMapPointMatches = std::vector<MapPoint*>(F.N, static_cast<MapPoint*>(NULL));

    const DBoW2::FeatureVector& vFeatVecKF = pKF->mFeatVec;

    int nmatches = 0;

    std::vector<int> rotHist[HISTO_LENGTH];
    const float factor = InitRotationHistogram(rotHist, HISTO_LENGTH);

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    while (KFit != KFend && Fit != Fend)
    {
        if (KFit->first == Fit->first)
        {
            const std::vector<unsigned int> vIndicesKF = KFit->second;
            const std::vector<unsigned int> vIndicesF = Fit->second;

            for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                MapPoint* pMP = vpMapPointsKF[realIdxKF];

                if (!pMP)
                {
                    continue;
                }
                if (pMP->isBad())
                {
                    continue;
                }
                const cv::Mat& dKF = pKF->mDescriptors.row(realIdxKF);

                BestTwoMatches bestLeft(256);
                BestTwoMatches bestRight(256);

                for (size_t iF = 0; iF < vIndicesF.size(); iF++)
                {
                    if (F.Nleft == -1)
                    {
                        const unsigned int realIdxF = vIndicesF[iF];

                        if (vpMapPointMatches[realIdxF])
                        {
                            continue;
                        }
                        const cv::Mat& dF = F.mDescriptors.row(realIdxF);

                        const int dist = DescriptorDistance(dKF, dF);

                        bestLeft.Update(dist, realIdxF);
                    }
                    else
                    {
                        const unsigned int realIdxF = vIndicesF[iF];

                        if (vpMapPointMatches[realIdxF])
                        {
                            continue;
                        }
                        const cv::Mat& dF = F.mDescriptors.row(realIdxF);

                        const int dist = DescriptorDistance(dKF, dF);

                        if (realIdxF < F.Nleft)
                        {
                            bestLeft.Update(dist, realIdxF);
                        }
                        else
                        {
                            bestRight.Update(dist, realIdxF);
                        }
                    }
                }

                if (bestLeft.bestDist <= mThLow)
                {
                    if (bestLeft.PassesRatio(mfNNratio))
                    {
                        vpMapPointMatches[bestLeft.bestIdx] = pMP;

                        const cv::KeyPoint& kp = (pKF->NLeft == -1)          ? pKF->mvKeysUn[realIdxKF]
                                                 : (realIdxKF >= pKF->NLeft) ? pKF->mvKeysRight[realIdxKF - pKF->NLeft]
                                                                             : pKF->mvKeys[realIdxKF];

                        if (mbCheckOrientation)
                        {
                            cv::KeyPoint& Fkp = (F.Nleft == -1) ? F.mvKeys[bestLeft.bestIdx]
                                                : (bestLeft.bestIdx >= F.Nleft)
                                                    ? F.mvKeysRight[bestLeft.bestIdx - F.Nleft]
                                                    : F.mvKeys[bestLeft.bestIdx];

                            AddRotationToHistogram(rotHist, HISTO_LENGTH, factor, kp.angle, Fkp.angle,
                                                   bestLeft.bestIdx);
                        }
                        nmatches++;
                    }

                    if (bestRight.bestDist <= mThLow)
                    {
                        if (bestRight.PassesRatio(mfNNratio) || true)
                        {
                            vpMapPointMatches[bestRight.bestIdx] = pMP;

                            const cv::KeyPoint& kp = (pKF->NLeft == -1) ? pKF->mvKeysUn[realIdxKF]
                                                     : (realIdxKF >= pKF->NLeft)
                                                         ? pKF->mvKeysRight[realIdxKF - pKF->NLeft]
                                                         : pKF->mvKeys[realIdxKF];

                            if (mbCheckOrientation)
                            {
                                cv::KeyPoint& Fkp = (F.Nleft == -1) ? F.mvKeys[bestRight.bestIdx]
                                                    : (bestRight.bestIdx >= F.Nleft)
                                                        ? F.mvKeysRight[bestRight.bestIdx - F.Nleft]
                                                        : F.mvKeys[bestRight.bestIdx];

                                AddRotationToHistogram(rotHist, HISTO_LENGTH, factor, kp.angle, Fkp.angle,
                                                       bestRight.bestIdx);
                            }
                            nmatches++;
                        }
                    }
                }
            }

            KFit++;
            Fit++;
        }
        else if (KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }

    if (mbCheckOrientation)
    {
        ApplyRotationConsistency(
            rotHist, HISTO_LENGTH,
            [&](int& ind1, int& ind2, int& ind3) { ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3); },
            [&](int idx)
            {
                vpMapPointMatches[idx] = static_cast<MapPoint*>(NULL);
                nmatches--;
            });
    }

    return nmatches;
}

int FeatureMatcher::SearchByProjection(KeyFrame* pKF, Sophus::Sim3f& Scw, const std::vector<MapPoint*>& vpPoints,
                                       std::vector<MapPoint*>& vpMatched, int th, float ratioHamming)
{
    Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    // Set of MapPoints already found in the KeyFrame
    std::set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

    int nmatches = 0;

    // For each Candidate MapPoint Project and Match
    for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if (pMP->isBad() || spAlreadyFound.count(pMP))
        {
            continue;
        }
        ProjectionMatchInput projection;
        if (TryProjectMapPointToKeyFrame(pMP, pKF, pKF->mpCamera, Tcw, Ow, th, false, true, projection) !=
            ProjectionStatus::kOk)
        {
            continue;
        }
        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();
        const BestMatch bestMatch =
            FindBestDescriptorMatch(projection.indices, pKF->mDescriptors, pKF->mvKeysUn, projection.predictedLevel,
                                    dMP, 256, [&](size_t idx) { return vpMatched[idx]; });

        if (bestMatch.bestDist <= mThLow * ratioHamming)
        {
            vpMatched[bestMatch.bestIdx] = pMP;
            nmatches++;
        }
    }

    return nmatches;
}

int FeatureMatcher::SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float>& Scw, const std::vector<MapPoint*>& vpPoints,
                                       const std::vector<KeyFrame*>& vpPointsKFs, std::vector<MapPoint*>& vpMatched,
                                       std::vector<KeyFrame*>& vpMatchedKF, int th, float ratioHamming)
{
    Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    // Set of MapPoints already found in the KeyFrame
    std::set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

    int nmatches = 0;

    // For each Candidate MapPoint Project and Match
    for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];
        KeyFrame* pKFi = vpPointsKFs[iMP];

        // Discard Bad MapPoints and already found
        if (pMP->isBad() || spAlreadyFound.count(pMP))
        {
            continue;
        }
        ProjectionMatchInput projection;
        if (TryProjectMapPointToKeyFrame(pMP, pKF, pKF->mpCamera, Tcw, Ow, th, false, true, projection) !=
            ProjectionStatus::kOk)
        {
            continue;
        }
        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();
        const BestMatch bestMatch =
            FindBestDescriptorMatch(projection.indices, pKF->mDescriptors, pKF->mvKeysUn, projection.predictedLevel,
                                    dMP, 256, [&](size_t idx) { return vpMatched[idx]; });

        if (bestMatch.bestDist <= mThLow * ratioHamming)
        {
            vpMatched[bestMatch.bestIdx] = pMP;
            vpMatchedKF[bestMatch.bestIdx] = pKFi;
            nmatches++;
        }
    }

    return nmatches;
}

int FeatureMatcher::SearchForInitialization(Frame& F1, Frame& F2, std::vector<cv::Point2f>& vbPrevMatched,
                                            std::vector<int>& vnMatches12, int windowSize)
{
    int nmatches = 0;
    vnMatches12 = std::vector<int>(F1.mvKeysUn.size(), -1);

    std::vector<int> rotHist[HISTO_LENGTH];
    const float factor = InitRotationHistogram(rotHist, HISTO_LENGTH);

    std::vector<int> vMatchedDistance(F2.mvKeysUn.size(), INT_MAX);
    std::vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

    for (size_t i1 = 0, iend1 = F1.mvKeysUn.size(); i1 < iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if (level1 > 0)
        {
            continue;
        }
        std::vector<size_t> vIndices2 =
            F2.GetFeaturesInArea(vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize, level1, level1);

        if (vIndices2.empty())
        {
            continue;
        }
        cv::Mat d1 = F1.mDescriptors.row(i1);

        BestTwoMatches bestMatches(INT_MAX);

        for (std::vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1, d2);

            if (vMatchedDistance[i2] <= dist)
            {
                continue;
            }
            bestMatches.Update(dist, i2);
        }

        if (bestMatches.bestDist <= mThLow)
        {
            if (bestMatches.PassesRatio(mfNNratio))
            {
                if (vnMatches21[bestMatches.bestIdx] >= 0)
                {
                    vnMatches12[vnMatches21[bestMatches.bestIdx]] = -1;
                    nmatches--;
                }
                vnMatches12[i1] = bestMatches.bestIdx;
                vnMatches21[bestMatches.bestIdx] = i1;
                vMatchedDistance[bestMatches.bestIdx] = bestMatches.bestDist;
                nmatches++;

                if (mbCheckOrientation)
                {
                    AddRotationToHistogram(rotHist, HISTO_LENGTH, factor, F1.mvKeysUn[i1].angle,
                                           F2.mvKeysUn[bestMatches.bestIdx].angle, i1);
                }
            }
        }
    }

    if (mbCheckOrientation)
    {
        ApplyRotationConsistency(
            rotHist, HISTO_LENGTH,
            [&](int& ind1, int& ind2, int& ind3) { ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3); },
            [&](int idx1)
            {
                if (vnMatches12[idx1] >= 0)
                {
                    vnMatches12[idx1] = -1;
                    nmatches--;
                }
            });
    }

    //Update prev matched
    for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
    {
        if (vnMatches12[i1] >= 0)
        {
            vbPrevMatched[i1] = F2.mvKeysUn[vnMatches12[i1]].pt;
        }
    }
    return nmatches;
}

int FeatureMatcher::SearchByBoW(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches12)
{
    const std::vector<cv::KeyPoint>& vKeysUn1 = pKF1->mvKeysUn;
    const DBoW2::FeatureVector& vFeatVec1 = pKF1->mFeatVec;
    const std::vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const cv::Mat& Descriptors1 = pKF1->mDescriptors;

    const std::vector<cv::KeyPoint>& vKeysUn2 = pKF2->mvKeysUn;
    const DBoW2::FeatureVector& vFeatVec2 = pKF2->mFeatVec;
    const std::vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const cv::Mat& Descriptors2 = pKF2->mDescriptors;

    vpMatches12 = std::vector<MapPoint*>(vpMapPoints1.size(), static_cast<MapPoint*>(NULL));
    std::vector<bool> vbMatched2(vpMapPoints2.size(), false);

    std::vector<int> rotHist[HISTO_LENGTH];
    const float factor = InitRotationHistogram(rotHist, HISTO_LENGTH);

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while (f1it != f1end && f2it != f2end)
    {
        if (f1it->first == f2it->first)
        {
            for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];
                if (pKF1->NLeft != -1 && idx1 >= pKF1->mvKeysUn.size())
                {
                    continue;
                }

                MapPoint* pMP1 = vpMapPoints1[idx1];
                if (!pMP1)
                {
                    continue;
                }
                if (pMP1->isBad())
                {
                    continue;
                }
                const cv::Mat& d1 = Descriptors1.row(idx1);

                BestTwoMatches bestMatches(256);

                for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    if (pKF2->NLeft != -1 && idx2 >= pKF2->mvKeysUn.size())
                    {
                        continue;
                    }

                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    if (vbMatched2[idx2] || !pMP2)
                    {
                        continue;
                    }
                    if (pMP2->isBad())
                    {
                        continue;
                    }
                    const cv::Mat& d2 = Descriptors2.row(idx2);

                    int dist = DescriptorDistance(d1, d2);

                    bestMatches.Update(dist, idx2);
                }

                if (bestMatches.bestDist < mThLow)
                {
                    if (bestMatches.PassesRatio(mfNNratio))
                    {
                        vpMatches12[idx1] = vpMapPoints2[bestMatches.bestIdx];
                        vbMatched2[bestMatches.bestIdx] = true;

                        if (mbCheckOrientation)
                        {
                            AddRotationToHistogram(rotHist, HISTO_LENGTH, factor, vKeysUn1[idx1].angle,
                                                   vKeysUn2[bestMatches.bestIdx].angle, idx1);
                        }
                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if (f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if (mbCheckOrientation)
    {
        ApplyRotationConsistency(
            rotHist, HISTO_LENGTH,
            [&](int& ind1, int& ind2, int& ind3) { ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3); },
            [&](int idx)
            {
                vpMatches12[idx] = static_cast<MapPoint*>(NULL);
                nmatches--;
            });
    }

    return nmatches;
}

int FeatureMatcher::SearchByBruteForce(KeyFrame* pKF, Frame& F)
{
    std::fill(F.mvpMapPoints.begin(), F.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
    const std::vector<MapPoint*> vpRefMapPoints = pKF->GetMapPointMatches();
    const int distThreshold = (mDescriptorType == DescriptorType::FLOAT32) ? mThHigh : mThLow;

    int nmatches = 0;
    for (size_t idx1 = 0; idx1 < vpRefMapPoints.size(); ++idx1)
    {
        MapPoint* pMP = vpRefMapPoints[idx1];
        if (!pMP || pMP->isBad())
        {
            continue;
        }

        const cv::Mat d1 = pKF->mDescriptors.row(static_cast<int>(idx1));
        const int octave1 = pKF->mvKeysUn[idx1].octave;

        int bestDist = distThreshold;
        int bestDist2 = std::numeric_limits<int>::max();
        int bestIdx2 = -1;
        for (int idx2 = 0; idx2 < F.N; ++idx2)
        {
            if (F.mvpMapPoints[idx2])
            {
                continue;
            }
            if (std::abs(octave1 - F.mvKeysUn[idx2].octave) > 1)
            {
                continue;
            }
            const cv::Mat& d2 = F.mDescriptors.row(idx2);
            const int dist = DescriptorDistance(d1, d2);
            if (dist > distThreshold)
            {
                continue;
            }

            UpdateBestAndSecond(dist, idx2, bestDist, bestIdx2, bestDist2);
        }

        if (bestIdx2 < 0)
        {
            continue;
        }
        if (!PassesBestSecondRatio(bestDist, bestDist2, mfNNratio))
        {
            continue;
        }

        F.mvpMapPoints[bestIdx2] = pMP;
        nmatches++;
    }

    return nmatches;
}

int FeatureMatcher::SearchForTriangulationNoBoW(KeyFrame* pKF1, KeyFrame* pKF2,
                                                std::vector<std::pair<size_t, size_t>>& vMatchedPairs,
                                                const bool bOnlyStereo, const bool bCoarse)
{
    Sophus::SE3f T1w = pKF1->GetPose();
    Sophus::SE3f T2w = pKF2->GetPose();
    Sophus::SE3f Tw2 = pKF2->GetPoseInverse();
    Eigen::Vector3f Cw = pKF1->GetCameraCenter();
    Eigen::Vector3f C2 = T2w * Cw;

    Eigen::Vector2f ep = pKF2->mpCamera->project(C2);
    Sophus::SE3f T12;
    Eigen::Matrix3f R12;
    Eigen::Vector3f t12;

    GeometricCamera* pCamera1 = pKF1->mpCamera;
    GeometricCamera* pCamera2 = pKF2->mpCamera;

    T12 = T1w * Tw2;
    R12 = T12.rotationMatrix();
    t12 = T12.translation();

    int nmatches = 0;
    std::vector<bool> vbMatched2(pKF2->N, false);
    std::vector<int> vMatches12(pKF1->N, -1);

    std::vector<int> rotHist[HISTO_LENGTH];
    const float factor = InitRotationHistogram(rotHist, HISTO_LENGTH);

    for (size_t idx1 = 0; idx1 < pKF1->N; ++idx1)
    {
        MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
        if (pMP1)
        {
            continue;
        }

        const bool bStereo1 = (pKF1->mvuRight[idx1] >= 0);
        if (bOnlyStereo && !bStereo1)
        {
            continue;
        }

        const cv::KeyPoint& kp1 = (pKF1->NLeft == -1)    ? pKF1->mvKeysUn[idx1]
                                  : (idx1 < pKF1->NLeft) ? pKF1->mvKeys[idx1]
                                                         : pKF1->mvKeysRight[idx1 - pKF1->NLeft];

        const cv::Mat& d1 = pKF1->mDescriptors.row(idx1);

        const int distThreshold = (mDescriptorType == DescriptorType::FLOAT32) ? mThHigh : mThLow;
        int bestDist = distThreshold;
        int bestDist2 = std::numeric_limits<int>::max();
        int bestIdx2 = -1;

        for (size_t idx2 = 0; idx2 < pKF2->N; ++idx2)
        {
            MapPoint* pMP2 = pKF2->GetMapPoint(idx2);
            if (vbMatched2[idx2] || pMP2)
            {
                continue;
            }
            const bool bStereo2 = (pKF2->mvuRight[idx2] >= 0);
            if (bOnlyStereo && !bStereo2)
            {
                continue;
            }

            const cv::KeyPoint& kp2 = (pKF2->NLeft == -1)    ? pKF2->mvKeysUn[idx2]
                                      : (idx2 < pKF2->NLeft) ? pKF2->mvKeys[idx2]
                                                             : pKF2->mvKeysRight[idx2 - pKF2->NLeft];

            if (std::abs(kp1.octave - kp2.octave) > 1)
            {
                continue;
            }

            const cv::Mat& d2 = pKF2->mDescriptors.row(idx2);
            const int dist = DescriptorDistance(d1, d2);

            if (dist > distThreshold || dist > bestDist)
            {
                continue;
            }
            if (!bStereo1 && !bStereo2)
            {
                const float distex = ep(0) - kp2.pt.x;
                const float distey = ep(1) - kp2.pt.y;
                if (distex * distex + distey * distey < 100 * pKF2->mvScaleFactors[kp2.octave])
                {
                    continue;
                }
            }

            if (bCoarse || pCamera1->epipolarConstrain(pCamera2, kp1, kp2, R12, t12, pKF1->mvLevelSigma2[kp1.octave],
                                                       pKF2->mvLevelSigma2[kp2.octave]))
            {
                UpdateBestAndSecond(dist, static_cast<int>(idx2), bestDist, bestIdx2, bestDist2);
            }
            else
            {
                continue;
            }
        }

        if (bestIdx2 >= 0)
        {
            // Reject ambiguous matches in no-BoW mode.
            if (!PassesBestSecondRatio(bestDist, bestDist2, mfNNratio))
            {
                continue;
            }
            const cv::KeyPoint& kp2 = (pKF2->NLeft == -1)        ? pKF2->mvKeysUn[bestIdx2]
                                      : (bestIdx2 < pKF2->NLeft) ? pKF2->mvKeys[bestIdx2]
                                                                 : pKF2->mvKeysRight[bestIdx2 - pKF2->NLeft];
            vMatches12[idx1] = bestIdx2;
            nmatches++;

            if (mbCheckOrientation)
            {
                AddRotationToHistogram(rotHist, HISTO_LENGTH, factor, kp1.angle, kp2.angle, static_cast<int>(idx1));
            }
        }
    }

    if (mbCheckOrientation)
    {
        ApplyRotationConsistency(
            rotHist, HISTO_LENGTH,
            [&](int& ind1, int& ind2, int& ind3) { ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3); },
            [&](int idx)
            {
                vMatches12[idx] = -1;
                nmatches--;
            });
    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(static_cast<size_t>(nmatches));

    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (vMatches12[i] < 0)
        {
            continue;
        }
        vMatchedPairs.push_back(std::make_pair(i, static_cast<size_t>(vMatches12[i])));
    }

    return nmatches;
}

int FeatureMatcher::SearchForTriangulation(KeyFrame* pKF1, KeyFrame* pKF2,
                                           std::vector<std::pair<size_t, size_t>>& vMatchedPairs,
                                           const bool bOnlyStereo, const bool bCoarse)
{
    // BoW-guided search requires mFeatVec from the binary ORB vocabulary. Float descriptors (SIFT)
    // skip ComputeBoW(), leaving mFeatVec empty — use a non-BoW fallback or triangulation adds 0 points.
    if (pKF1->mFeatVec.empty() || pKF2->mFeatVec.empty())
    {
        return SearchForTriangulationNoBoW(pKF1, pKF2, vMatchedPairs, bOnlyStereo, bCoarse);
    }

    const DBoW2::FeatureVector& vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector& vFeatVec2 = pKF2->mFeatVec;

    //Compute epipole in second image
    Sophus::SE3f T1w = pKF1->GetPose();
    Sophus::SE3f T2w = pKF2->GetPose();
    Sophus::SE3f Tw2 = pKF2->GetPoseInverse();  // for convenience
    Eigen::Vector3f Cw = pKF1->GetCameraCenter();
    Eigen::Vector3f C2 = T2w * Cw;

    Eigen::Vector2f ep = pKF2->mpCamera->project(C2);
    Sophus::SE3f T12;
    Eigen::Matrix3f R12;  // for fastest computation
    Eigen::Vector3f t12;  // for fastest computation

    GeometricCamera *pCamera1 = pKF1->mpCamera, *pCamera2 = pKF2->mpCamera;

    T12 = T1w * Tw2;
    R12 = T12.rotationMatrix();
    t12 = T12.translation();

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node
    int nmatches = 0;
    std::vector<bool> vbMatched2(pKF2->N, false);
    std::vector<int> vMatches12(pKF1->N, -1);

    std::vector<int> rotHist[HISTO_LENGTH];
    const float factor = InitRotationHistogram(rotHist, HISTO_LENGTH);

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while (f1it != f1end && f2it != f2end)
    {
        if (f1it->first == f2it->first)
        {
            for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];

                MapPoint* pMP1 = pKF1->GetMapPoint(idx1);

                // If there is already a MapPoint skip
                if (pMP1)
                {
                    continue;
                }

                const bool bStereo1 = (pKF1->mvuRight[idx1] >= 0);

                if (bOnlyStereo && !bStereo1)
                {
                    continue;
                }
                const cv::KeyPoint& kp1 = (pKF1->NLeft == -1)    ? pKF1->mvKeysUn[idx1]
                                          : (idx1 < pKF1->NLeft) ? pKF1->mvKeys[idx1]
                                                                 : pKF1->mvKeysRight[idx1 - pKF1->NLeft];

                const cv::Mat& d1 = pKF1->mDescriptors.row(idx1);

                int bestDist = mThLow;
                int bestIdx2 = -1;

                for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++)
                {
                    size_t idx2 = f2it->second[i2];

                    MapPoint* pMP2 = pKF2->GetMapPoint(idx2);

                    // If we have already matched or there is a MapPoint skip
                    if (vbMatched2[idx2] || pMP2)
                    {
                        continue;
                    }
                    const bool bStereo2 = (pKF2->mvuRight[idx2] >= 0);

                    if (bOnlyStereo && !bStereo2)
                    {
                        continue;
                    }
                    const cv::Mat& d2 = pKF2->mDescriptors.row(idx2);

                    const int dist = DescriptorDistance(d1, d2);

                    if (dist > mThLow || dist > bestDist)
                    {
                        continue;
                    }
                    const cv::KeyPoint& kp2 = (pKF2->NLeft == -1)    ? pKF2->mvKeysUn[idx2]
                                              : (idx2 < pKF2->NLeft) ? pKF2->mvKeys[idx2]
                                                                     : pKF2->mvKeysRight[idx2 - pKF2->NLeft];
                    if (!bStereo1 && !bStereo2)
                    {
                        const float distex = ep(0) - kp2.pt.x;
                        const float distey = ep(1) - kp2.pt.y;
                        if (distex * distex + distey * distey < 100 * pKF2->mvScaleFactors[kp2.octave])
                        {
                            continue;
                        }
                    }

                    if (bCoarse ||
                        pCamera1->epipolarConstrain(pCamera2, kp1, kp2, R12, t12, pKF1->mvLevelSigma2[kp1.octave],
                                                    pKF2->mvLevelSigma2[kp2.octave]))  // MODIFICATION_2
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }

                if (bestIdx2 >= 0)
                {
                    const cv::KeyPoint& kp2 = (pKF2->NLeft == -1)        ? pKF2->mvKeysUn[bestIdx2]
                                              : (bestIdx2 < pKF2->NLeft) ? pKF2->mvKeys[bestIdx2]
                                                                         : pKF2->mvKeysRight[bestIdx2 - pKF2->NLeft];
                    vMatches12[idx1] = bestIdx2;
                    nmatches++;

                    if (mbCheckOrientation)
                    {
                        AddRotationToHistogram(rotHist, HISTO_LENGTH, factor, kp1.angle, kp2.angle, idx1);
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if (f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if (mbCheckOrientation)
    {
        ApplyRotationConsistency(
            rotHist, HISTO_LENGTH,
            [&](int& ind1, int& ind2, int& ind3) { ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3); },
            [&](int idx)
            {
                vMatches12[idx] = -1;
                nmatches--;
            });
    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
    {
        if (vMatches12[i] < 0)
        {
            continue;
        }
        vMatchedPairs.push_back(std::make_pair(i, vMatches12[i]));
    }

    return nmatches;
}

int FeatureMatcher::Fuse(KeyFrame* pKF, const std::vector<MapPoint*>& vpMapPoints, const float th, const bool bRight)
{
    GeometricCamera* pCamera;
    Sophus::SE3f Tcw;
    Eigen::Vector3f Ow;

    Tcw = pKF->GetPose();
    Ow = pKF->GetCameraCenter();
    pCamera = pKF->mpCamera;

    const float& fx = pKF->fx;
    const float& fy = pKF->fy;
    const float& cx = pKF->cx;
    const float& cy = pKF->cy;
    const float& bf = pKF->mbf;

    int nFused = 0;

    const int nMPs = vpMapPoints.size();

    // For debbuging
    int count_notMP = 0, count_bad = 0, count_isinKF = 0, count_negdepth = 0, count_notinim = 0, count_dist = 0,
        count_normal = 0, count_notidx = 0, count_thcheck = 0;
    for (int i = 0; i < nMPs; i++)
    {
        MapPoint* pMP = vpMapPoints[i];

        if (!pMP)
        {
            count_notMP++;
            continue;
        }

        if (pMP->isBad())
        {
            count_bad++;
            continue;
        }
        else if (pMP->IsInKeyFrame(pKF))
        {
            count_isinKF++;
            continue;
        }

        ProjectionMatchInput projection;
        const ProjectionStatus projectionStatus =
            TryProjectMapPointToKeyFrame(pMP, pKF, pCamera, Tcw, Ow, th, bRight, true, projection);
        if (projectionStatus != ProjectionStatus::kOk)
        {
            switch (projectionStatus)
            {
                case ProjectionStatus::kNegativeDepth:
                    count_negdepth++;
                    break;
                case ProjectionStatus::kOutOfImage:
                    count_notinim++;
                    break;
                case ProjectionStatus::kOutOfDistance:
                    count_dist++;
                    break;
                case ProjectionStatus::kBadViewingAngle:
                    count_normal++;
                    break;
                case ProjectionStatus::kNoCandidates:
                    count_notidx++;
                    break;
                case ProjectionStatus::kOk:
                    break;
            }
            continue;
        }

        const Eigen::Vector2f& uv = projection.uv;
        const float ur = uv(0) - bf * projection.invz;
        const int nPredictedLevel = projection.predictedLevel;
        const std::vector<size_t>& vIndices = projection.indices;

        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->GetDescriptor();

        BestMatch bestMatch(256);
        for (std::vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++)
        {
            size_t idx = *vit;
            const cv::KeyPoint& kp = (pKF->NLeft == -1) ? pKF->mvKeysUn[idx]
                                     : (!bRight)        ? pKF->mvKeys[idx]
                                                        : pKF->mvKeysRight[idx];

            const int& kpLevel = kp.octave;

            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
            {
                continue;
            }
            if (pKF->mvuRight[idx] >= 0)
            {
                // Check reprojection error in stereo
                const float& kpx = kp.pt.x;
                const float& kpy = kp.pt.y;
                const float& kpr = pKF->mvuRight[idx];
                const float ex = uv(0) - kpx;
                const float ey = uv(1) - kpy;
                const float er = ur - kpr;
                const float e2 = ex * ex + ey * ey + er * er;

                if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 7.8)
                {
                    continue;
                }
            }
            else
            {
                const float& kpx = kp.pt.x;
                const float& kpy = kp.pt.y;
                const float ex = uv(0) - kpx;
                const float ey = uv(1) - kpy;
                const float e2 = ex * ex + ey * ey;

                if (e2 * pKF->mvInvLevelSigma2[kpLevel] > 5.99)
                {
                    continue;
                }
            }

            if (bRight)
            {
                idx += pKF->NLeft;
            }
            const cv::Mat& dKF = pKF->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP, dKF);

            bestMatch.Update(dist, idx);
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if (bestMatch.bestDist <= mThLow)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestMatch.bestIdx);
            if (pMPinKF)
            {
                if (!pMPinKF->isBad())
                {
                    if (pMPinKF->Observations() > pMP->Observations())
                    {
                        pMP->Replace(pMPinKF);
                    }
                    else
                    {
                        pMPinKF->Replace(pMP);
                    }
                }
            }
            else
            {
                pMP->AddObservation(pKF, bestMatch.bestIdx);
                pKF->AddMapPoint(pMP, bestMatch.bestIdx);
            }
            nFused++;
        }
        else
        {
            count_thcheck++;
        }
    }

    return nFused;
}

int FeatureMatcher::Fuse(KeyFrame* pKF, Sophus::Sim3f& Scw, const std::vector<MapPoint*>& vpPoints, float th,
                         std::vector<MapPoint*>& vpReplacePoint)
{
    // Get Calibration Parameters for later projection
    const float& fx = pKF->fx;
    const float& fy = pKF->fy;
    const float& cx = pKF->cx;
    const float& cy = pKF->cy;

    // Decompose Scw
    Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    // Set of MapPoints already found in the KeyFrame
    const std::set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

    int nFused = 0;

    const int nPoints = vpPoints.size();

    // For each candidate MapPoint project and match
    for (int iMP = 0; iMP < nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if (pMP->isBad() || spAlreadyFound.count(pMP))
        {
            continue;
        }
        ProjectionMatchInput projection;
        if (TryProjectMapPointToKeyFrame(pMP, pKF, pKF->mpCamera, Tcw, Ow, th, false, true, projection) !=
            ProjectionStatus::kOk)
        {
            continue;
        }
        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->GetDescriptor();
        const BestMatch bestMatch =
            FindBestDescriptorMatch(projection.indices, pKF->mDescriptors, pKF->mvKeysUn, projection.predictedLevel,
                                    dMP, INT_MAX, [&](size_t idx) { return false; });

        // If there is already a MapPoint replace otherwise add new measurement
        if (bestMatch.bestDist <= mThLow)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestMatch.bestIdx);
            if (pMPinKF)
            {
                if (!pMPinKF->isBad())
                {
                    vpReplacePoint[iMP] = pMPinKF;
                }
            }
            else
            {
                pMP->AddObservation(pKF, bestMatch.bestIdx);
                pKF->AddMapPoint(pMP, bestMatch.bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}

int FeatureMatcher::SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches12,
                                 const Sophus::Sim3f& S12, const float th)
{
    const float& fx = pKF1->fx;
    const float& fy = pKF1->fy;
    const float& cx = pKF1->cx;
    const float& cy = pKF1->cy;

    // Camera 1 & 2 from world
    Sophus::SE3f T1w = pKF1->GetPose();
    Sophus::SE3f T2w = pKF2->GetPose();

    //Transformation between cameras
    Sophus::Sim3f S21 = S12.inverse();

    const std::vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();

    const std::vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();

    std::vector<bool> vbAlreadyMatched1(N1, false);
    std::vector<bool> vbAlreadyMatched2(N2, false);

    for (int i = 0; i < N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];
        if (pMP)
        {
            vbAlreadyMatched1[i] = true;
            int idx2 = std::get<0>(pMP->GetIndexInKeyFrame(pKF2));
            if (idx2 >= 0 && idx2 < N2)
            {
                vbAlreadyMatched2[idx2] = true;
            }
        }
    }

    std::vector<int> vnMatch1(N1, -1);
    std::vector<int> vnMatch2(N2, -1);

    // Transform from KF1 to KF2 and search
    for (int i1 = 0; i1 < N1; i1++)
    {
        MapPoint* pMP = vpMapPoints1[i1];

        if (!pMP || vbAlreadyMatched1[i1])
        {
            continue;
        }
        if (pMP->isBad())
        {
            continue;
        }
        Eigen::Vector3f p3Dw = pMP->GetWorldPos();
        Eigen::Vector3f p3Dc1 = T1w * p3Dw;
        Eigen::Vector3f p3Dc2 = S21 * p3Dc1;

        // Depth must be positive
        if (p3Dc2(2) < 0.0)
        {
            continue;
        }
        const float invz = 1.0 / p3Dc2(2);
        const float x = p3Dc2(0) * invz;
        const float y = p3Dc2(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;

        // Point must be inside the image
        if (!pKF2->IsInImage(u, v))
        {
            continue;
        }
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = p3Dc2.norm();

        // Depth must be inside the scale invariance region
        if (dist3D < minDistance || dist3D > maxDistance)
        {
            continue;
        }
        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D, pKF2);

        // Search in a radius
        const float radius = th * pKF2->mvScaleFactors[nPredictedLevel];

        const std::vector<size_t> vIndices = pKF2->GetFeaturesInArea(u, v, radius);

        if (vIndices.empty())
        {
            continue;
        }
        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();
        const BestMatch bestMatch = FindBestDescriptorMatch(
            vIndices, pKF2->mDescriptors, pKF2->mvKeysUn, nPredictedLevel, dMP, INT_MAX, [&](size_t) { return false; });

        if (bestMatch.bestDist <= mThHigh)
        {
            vnMatch1[i1] = bestMatch.bestIdx;
        }
    }

    // Transform from KF2 to KF2 and search
    for (int i2 = 0; i2 < N2; i2++)
    {
        MapPoint* pMP = vpMapPoints2[i2];

        if (!pMP || vbAlreadyMatched2[i2])
        {
            continue;
        }
        if (pMP->isBad())
        {
            continue;
        }
        Eigen::Vector3f p3Dw = pMP->GetWorldPos();
        Eigen::Vector3f p3Dc2 = T2w * p3Dw;
        Eigen::Vector3f p3Dc1 = S12 * p3Dc2;

        // Depth must be positive
        if (p3Dc1(2) < 0.0)
        {
            continue;
        }
        const float invz = 1.0 / p3Dc1(2);
        const float x = p3Dc1(0) * invz;
        const float y = p3Dc1(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;

        // Point must be inside the image
        if (!pKF1->IsInImage(u, v))
        {
            continue;
        }
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = p3Dc1.norm();

        // Depth must be inside the scale pyramid of the image
        if (dist3D < minDistance || dist3D > maxDistance)
        {
            continue;
        }
        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D, pKF1);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th * pKF1->mvScaleFactors[nPredictedLevel];

        const std::vector<size_t> vIndices = pKF1->GetFeaturesInArea(u, v, radius);

        if (vIndices.empty())
        {
            continue;
        }
        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();
        const BestMatch bestMatch = FindBestDescriptorMatch(
            vIndices, pKF1->mDescriptors, pKF1->mvKeysUn, nPredictedLevel, dMP, INT_MAX, [&](size_t) { return false; });

        if (bestMatch.bestDist <= mThHigh)
        {
            vnMatch2[i2] = bestMatch.bestIdx;
        }
    }

    // Check agreement
    int nFound = 0;

    for (int i1 = 0; i1 < N1; i1++)
    {
        int idx2 = vnMatch1[i1];

        if (idx2 >= 0)
        {
            int idx1 = vnMatch2[idx2];
            if (idx1 == i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

int FeatureMatcher::SearchByProjection(Frame& CurrentFrame, const Frame& LastFrame, const float th, const bool bMono)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    std::vector<int> rotHist[HISTO_LENGTH];
    const float factor = InitRotationHistogram(rotHist, HISTO_LENGTH);

    const Sophus::SE3f Tcw = CurrentFrame.GetPose();
    const Eigen::Vector3f twc = Tcw.inverse().translation();

    const Sophus::SE3f Tlw = LastFrame.GetPose();
    const Eigen::Vector3f tlc = Tlw * twc;

    const bool bForward = tlc(2) > CurrentFrame.mb && !bMono;
    const bool bBackward = -tlc(2) > CurrentFrame.mb && !bMono;

    for (int i = 0; i < LastFrame.N; i++)
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];
        if (pMP)
        {
            if (!LastFrame.mvbOutlier[i])
            {
                // Project
                Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                Eigen::Vector3f x3Dc = Tcw * x3Dw;

                const float xc = x3Dc(0);
                const float yc = x3Dc(1);
                const float invzc = 1.0 / x3Dc(2);

                if (invzc < 0)
                {
                    continue;
                }
                Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

                if (uv(0) < CurrentFrame.mnMinX || uv(0) > CurrentFrame.mnMaxX)
                {
                    continue;
                }
                if (uv(1) < CurrentFrame.mnMinY || uv(1) > CurrentFrame.mnMaxY)
                {
                    continue;
                }
                int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft)
                                      ? LastFrame.mvKeys[i].octave
                                      : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

                // Search in a window. Size depends on scale
                float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

                std::vector<size_t> vIndices2;

                if (bForward)
                {
                    vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave);
                }
                else if (bBackward)
                {
                    vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, 0, nLastOctave);
                }
                else
                {
                    vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave - 1, nLastOctave + 1);
                }
                if (vIndices2.empty())
                {
                    continue;
                }
                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = std::numeric_limits<int>::max();
                int bestIdx2 = -1;

                for (std::vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end(); vit != vend;
                     vit++)
                {
                    const size_t i2 = *vit;

                    if (CurrentFrame.mvpMapPoints[i2])
                    {
                        if (CurrentFrame.mvpMapPoints[i2]->Observations() > 0)
                        {
                            continue;
                        }
                    }
                    if (CurrentFrame.Nleft == -1 && CurrentFrame.mvuRight[i2] > 0)
                    {
                        const float ur = uv(0) - CurrentFrame.mbf * invzc;
                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                        if (er > radius)
                        {
                            continue;
                        }
                    }

                    const cv::Mat& d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP, d);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdx2 = i2;
                    }
                }

                if (bestIdx2 >= 0 && bestDist <= mThHigh)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
                    nmatches++;

                    if (mbCheckOrientation)
                    {
                        cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                                            : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
                                                                    : LastFrame.mvKeysRight[i - LastFrame.Nleft];

                        cv::KeyPoint kpCF = (CurrentFrame.Nleft == -1) ? CurrentFrame.mvKeysUn[bestIdx2]
                                            : (bestIdx2 < CurrentFrame.Nleft)
                                                ? CurrentFrame.mvKeys[bestIdx2]
                                                : CurrentFrame.mvKeysRight[bestIdx2 - CurrentFrame.Nleft];
                        AddRotationToHistogram(rotHist, HISTO_LENGTH, factor, kpLF.angle, kpCF.angle, bestIdx2);
                    }
                }
                if (CurrentFrame.Nleft != -1)
                {
                    Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

                    int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft)
                                          ? LastFrame.mvKeys[i].octave
                                          : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

                    // Search in a window. Size depends on scale
                    float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

                    std::vector<size_t> vIndices2;

                    if (bForward)
                    {
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave, -1, true);
                    }
                    else if (bBackward)
                    {
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, 0, nLastOctave, true);
                    }
                    else
                    {
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave - 1,
                                                                   nLastOctave + 1, true);
                    }
                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = std::numeric_limits<int>::max();
                    int bestIdx2 = -1;

                    for (std::vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end();
                         vit != vend; vit++)
                    {
                        const size_t i2 = *vit;
                        if (CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft])
                        {
                            if (CurrentFrame.mvpMapPoints[i2 + CurrentFrame.Nleft]->Observations() > 0)
                            {
                                continue;
                            }
                        }
                        const cv::Mat& d = CurrentFrame.mDescriptors.row(i2 + CurrentFrame.Nleft);

                        const int dist = DescriptorDistance(dMP, d);

                        if (dist < bestDist)
                        {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    if (bestIdx2 >= 0 && bestDist <= mThHigh)
                    {
                        CurrentFrame.mvpMapPoints[bestIdx2 + CurrentFrame.Nleft] = pMP;
                        nmatches++;
                        if (mbCheckOrientation)
                        {
                            cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
                                                : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
                                                                        : LastFrame.mvKeysRight[i - LastFrame.Nleft];

                            cv::KeyPoint kpCF = CurrentFrame.mvKeysRight[bestIdx2];

                            AddRotationToHistogram(rotHist, HISTO_LENGTH, factor, kpLF.angle, kpCF.angle,
                                                   bestIdx2 + CurrentFrame.Nleft);
                        }
                    }
                }
            }
        }
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << CurrentFrame.mnId << "] " << "SEARCH_BY_PROJECTION: nmatches=" << nmatches << std::endl;

    //Apply rotation consistency
    if (mbCheckOrientation)
    {
        ApplyRotationConsistency(
            rotHist, HISTO_LENGTH,
            [&](int& ind1, int& ind2, int& ind3) { ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3); },
            [&](int idx)
            {
                CurrentFrame.mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
                nmatches--;
            });
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << CurrentFrame.mnId << "] "
        << "SEARCH_BY_PROJECTION: after rotation consistency check nmatches=" << nmatches << std::endl;

    return nmatches;
}

int FeatureMatcher::SearchByProjection(Frame& CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*>& sAlreadyFound,
                                       const float th, const int ORBdist)
{
    int nmatches = 0;

    const Sophus::SE3f Tcw = CurrentFrame.GetPose();
    Eigen::Vector3f Ow = Tcw.inverse().translation();

    // Rotation Histogram (to check rotation consistency)
    std::vector<int> rotHist[HISTO_LENGTH];
    const float factor = InitRotationHistogram(rotHist, HISTO_LENGTH);

    const std::vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if (pMP)
        {
            if (!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //Project
                Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                Eigen::Vector3f x3Dc = Tcw * x3Dw;

                const Eigen::Vector2f uv = CurrentFrame.mpCamera->project(x3Dc);

                if (uv(0) < CurrentFrame.mnMinX || uv(0) > CurrentFrame.mnMaxX)
                {
                    continue;
                }
                if (uv(1) < CurrentFrame.mnMinY || uv(1) > CurrentFrame.mnMaxY)
                {
                    continue;
                }
                // Compute predicted scale level
                Eigen::Vector3f PO = x3Dw - Ow;
                float dist3D = PO.norm();

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if (dist3D < minDistance || dist3D > maxDistance)
                {
                    continue;
                }
                int nPredictedLevel = pMP->PredictScale(dist3D, &CurrentFrame);

                // Search in a window
                const float radius = th * CurrentFrame.mvScaleFactors[nPredictedLevel];

                const std::vector<size_t> vIndices2 =
                    CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nPredictedLevel - 1, nPredictedLevel + 1);

                if (vIndices2.empty())
                {
                    continue;
                }
                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = std::numeric_limits<int>::max();
                int bestIdx2 = -1;

                for (std::vector<size_t>::const_iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if (CurrentFrame.mvpMapPoints[i2])
                    {
                        continue;
                    }
                    const cv::Mat& d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP, d);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdx2 = i2;
                    }
                }

                if (bestIdx2 >= 0 && bestDist <= ORBdist)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
                    nmatches++;

                    if (mbCheckOrientation)
                    {
                        AddRotationToHistogram(rotHist, HISTO_LENGTH, factor, pKF->mvKeysUn[i].angle,
                                               CurrentFrame.mvKeysUn[bestIdx2].angle, bestIdx2);
                    }
                }
            }
        }
    }

    if (mbCheckOrientation)
    {
        ApplyRotationConsistency(
            rotHist, HISTO_LENGTH,
            [&](int& ind1, int& ind2, int& ind3) { ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3); },
            [&](int idx)
            {
                CurrentFrame.mvpMapPoints[idx] = NULL;
                nmatches--;
            });
    }

    return nmatches;
}

void FeatureMatcher::ComputeThreeMaxima(std::vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3)
{
    int max1 = 0;
    int max2 = 0;
    int max3 = 0;

    for (int i = 0; i < L; i++)
    {
        const int s = histo[i].size();
        if (s > max1)
        {
            max3 = max2;
            max2 = max1;
            max1 = s;
            ind3 = ind2;
            ind2 = ind1;
            ind1 = i;
        }
        else if (s > max2)
        {
            max3 = max2;
            max2 = s;
            ind3 = ind2;
            ind2 = i;
        }
        else if (s > max3)
        {
            max3 = s;
            ind3 = i;
        }
    }

    if (max2 < 0.1f * (float)max1)
    {
        ind2 = -1;
        ind3 = -1;
    }
    else if (max3 < 0.1f * (float)max1)
    {
        ind3 = -1;
    }
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int FeatureMatcher::DescriptorDistance(const cv::Mat& a, const cv::Mat& b)
{
    // Binary ORB descriptor: 32 bytes = 8x int32_t words.
    if (a.type() == CV_8UC1 && b.type() == CV_8UC1)
    {
        const int* pa = a.ptr<int32_t>();
        const int* pb = b.ptr<int32_t>();

        int dist = 0;

        for (int i = 0; i < 8; i++, pa++, pb++)
        {
            unsigned int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

    // Float32 descriptors (e.g., SIFT): use L2 distance.
    // Return an int for compatibility with existing thresholding code.
    return cvRound(cv::norm(a, b, cv::NORM_L2));
}

}  // namespace ORB_SLAM3