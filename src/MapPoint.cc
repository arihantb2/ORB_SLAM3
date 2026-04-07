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

#include "MapPoint.h"
#include "FeatureMatcher.h"

#include <algorithm>
#include <mutex>

namespace ORB_SLAM3
{

long unsigned int MapPoint::nNextId = 0;

MapPoint::MapPoint()
    : mnFirstKFid(0),
      mnFirstFrame(0),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr)
{
}

MapPoint::MapPoint(const Eigen::Vector3f& Pos, KeyFrame* pRefKF, Map* pMap)
    : mnFirstKFid(pRefKF->mnId),
      mnFirstFrame(pRefKF->mnFrameId),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(pRefKF),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr),
      mfMinDistance(0),
      mfMaxDistance(0),
      mpMap(pMap),
      mnOriginMapId(pMap->GetId())
{
    SetWorldPos(Pos);

    mNormalVector.setZero();

    mbTrackInViewR = false;
    mbTrackInView = false;

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
}

MapPoint::MapPoint(const double invDepth, cv::Point2f uv_init, KeyFrame* pRefKF, KeyFrame* pHostKF, Map* pMap)
    : mnFirstKFid(pRefKF->mnId),
      mnFirstFrame(pRefKF->mnFrameId),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(pRefKF),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr),
      mfMinDistance(0),
      mfMaxDistance(0),
      mpMap(pMap),
      mnOriginMapId(pMap->GetId())
{
    mInvDepth = invDepth;
    mInitU = (double)uv_init.x;
    mInitV = (double)uv_init.y;
    mpHostKF = pHostKF;

    mNormalVector.setZero();

    // Worldpos is not set
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
}

MapPoint::MapPoint(const Eigen::Vector3f& Pos, Map* pMap, Frame* pFrame, const int& idxF)
    : mnFirstKFid(-1),
      mnFirstFrame(pFrame->mnId),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(nullptr),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(nullptr),
      mpMap(pMap),
      mnOriginMapId(pMap->GetId())
{
    SetWorldPos(Pos);

    Eigen::Vector3f Ow;
    Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector / mNormalVector.norm();

    Eigen::Vector3f PC = mWorldPos - Ow;
    const float dist = PC.norm();
    const int level = (pFrame->Nleft == -1)    ? pFrame->mvKeysUn[idxF].octave
                      : (idxF < pFrame->Nleft) ? pFrame->mvKeys[idxF].octave
                                               : pFrame->mvKeysRight[idxF].octave;
    const float levelScaleFactor = pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist * levelScaleFactor;
    mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
    mnId = nNextId++;
}

void MapPoint::SetWorldPos(const Eigen::Vector3f& Pos)
{
    std::lock_guard<std::mutex> lock(mMutexPos);
    mWorldPos = Pos;
}

Eigen::Vector3f MapPoint::GetWorldPos()
{
    std::lock_guard<std::mutex> lock(mMutexPos);
    return mWorldPos;
}

Eigen::Vector3f MapPoint::GetNormal()
{
    std::lock_guard<std::mutex> lock(mMutexPos);
    return mNormalVector;
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, int idx)
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    std::tuple<int, int> indexes;

    if (mObservations.count(pKF))
    {
        indexes = mObservations[pKF];
    }
    else
    {
        indexes = std::tuple<int, int>(-1, -1);
    }

    if (pKF->NLeft != -1 && idx >= pKF->NLeft)
    {
        std::get<1>(indexes) = idx;
    }
    else
    {
        std::get<0>(indexes) = idx;
    }

    mObservations[pKF] = indexes;

    if (pKF->mvuRight[idx] >= 0)
    {
        nObs += 2;
    }
    else
    {
        nObs++;
    }
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad = false;
    {
        std::lock_guard<std::mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
        {
            std::tuple<int, int> indexes = mObservations[pKF];
            int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

            if (leftIndex != -1)
            {
                if (pKF->mvuRight[leftIndex] >= 0)
                {
                    nObs -= 2;
                }
                else
                {
                    nObs--;
                }
            }
            if (rightIndex != -1)
            {
                nObs--;
            }

            mObservations.erase(pKF);

            if (mpRefKF == pKF)
            {
                // Pick the observation with the lowest KF mnId as the new reference for
                // deterministic behaviour regardless of container or pointer ordering.
                mpRefKF = std::min_element(mObservations.begin(), mObservations.end(),
                                           [](const auto& a, const auto& b) { return a.first->mnId < b.first->mnId; })
                              ->first;
            }
            // If only 2 observations or less, discard point
            if (nObs <= 2)
            {
                bBad = true;
            }
        }
    }

    if (bBad)
    {
        SetBadFlag();
    }
}

std::unordered_map<KeyFrame*, std::tuple<int, int>> MapPoint::GetObservations()
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    std::unordered_map<KeyFrame*, std::tuple<int, int>> obs;
    {
        std::lock_guard<std::mutex> lock1(mMutexFeatures);
        std::lock_guard<std::mutex> lock2(mMutexPos);
        mbBad = true;
        obs = mObservations;
        mObservations.clear();
    }
    for (auto& [pKF, indexes] : obs)
    {
        int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);
        if (leftIndex != -1)
        {
            pKF->EraseMapPointMatch(leftIndex);
        }
        if (rightIndex != -1)
        {
            pKF->EraseMapPointMatch(rightIndex);
        }
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    std::scoped_lock lock(mMutexFeatures, mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if (pMP->mnId == this->mnId)
    {
        return;
    }
    int nvisible, nfound;
    std::unordered_map<KeyFrame*, std::tuple<int, int>> obs;
    {
        std::lock_guard<std::mutex> lock1(mMutexFeatures);
        std::lock_guard<std::mutex> lock2(mMutexPos);
        obs = mObservations;
        mObservations.clear();
        mbBad = true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for (auto& [pKF, indexes] : obs)
    {
        int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

        if (!pMP->IsInKeyFrame(pKF))
        {
            if (leftIndex != -1)
            {
                pKF->ReplaceMapPointMatch(leftIndex, pMP);
                pMP->AddObservation(pKF, leftIndex);
            }
            if (rightIndex != -1)
            {
                pKF->ReplaceMapPointMatch(rightIndex, pMP);
                pMP->AddObservation(pKF, rightIndex);
            }
        }
        else
        {
            if (leftIndex != -1)
            {
                pKF->EraseMapPointMatch(leftIndex);
            }
            if (rightIndex != -1)
            {
                pKF->EraseMapPointMatch(rightIndex);
            }
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    std::scoped_lock lock(mMutexFeatures, mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    mnVisible += n;
}

void MapPoint::IncreaseFound(int n)
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    mnFound += n;
}

float MapPoint::GetFoundRatio()
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound) / mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    std::vector<cv::Mat> vDescriptors;

    std::unordered_map<KeyFrame*, std::tuple<int, int>> observations;

    {
        std::lock_guard<std::mutex> lock1(mMutexFeatures);
        if (mbBad)
        {
            return;
        }
        observations = mObservations;
    }

    if (observations.empty())
    {
        return;
    }
    vDescriptors.reserve(observations.size());

    // Sort by KF mnId so the collected descriptor set — and therefore the
    // median selection — is deterministic regardless of pointer address (ASLR).
    std::vector<std::pair<KeyFrame*, std::tuple<int, int>>> vSortedObs(observations.begin(), observations.end());
    std::sort(vSortedObs.begin(), vSortedObs.end(),
              [](const std::pair<KeyFrame*, std::tuple<int, int>>& a,
                 const std::pair<KeyFrame*, std::tuple<int, int>>& b) { return a.first->mnId < b.first->mnId; });

    for (const auto& [pKF, indexes] : vSortedObs)
    {
        if (!pKF->isBad())
        {
            int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

            if (leftIndex != -1)
            {
                vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
            }
            if (rightIndex != -1)
            {
                vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
            }
        }
    }

    if (vDescriptors.empty())
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "COMPUTE_DISTINCTIVE_DESCRIPTORS: MapPoint " << mnId
            << " has no valid observations to compute descriptor. This should not happen.";
        return;
    }

    // Compute distances between them.
    // Use a flat heap-allocated array to avoid non-standard VLA.
    const size_t N = vDescriptors.size();
    std::vector<int> distances(N * N, 0);
    for (size_t i = 0; i < N; i++)
    {
        for (size_t j = i + 1; j < N; j++)
        {
            int distij = FeatureMatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
            distances[i * N + j] = distij;
            distances[j * N + i] = distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for (size_t i = 0; i < N; i++)
    {
        std::vector<int> vDists(distances.begin() + i * N, distances.begin() + (i + 1) * N);
        sort(vDists.begin(), vDists.end());
        int median = vDists[static_cast<size_t>(0.5 * (N - 1))];

        if (median < BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        std::lock_guard<std::mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

std::tuple<int, int> MapPoint::GetIndexInKeyFrame(KeyFrame* pKF)
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF))
    {
        return mObservations[pKF];
    }
    else
    {
        return std::tuple<int, int>(-1, -1);
    }
}

bool MapPoint::IsInKeyFrame(KeyFrame* pKF)
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    std::unordered_map<KeyFrame*, std::tuple<int, int>> observations;
    KeyFrame* pRefKF;
    Eigen::Vector3f Pos;
    {
        std::lock_guard<std::mutex> lock1(mMutexFeatures);
        std::lock_guard<std::mutex> lock2(mMutexPos);
        if (mbBad)
        {
            return;
        }
        observations = mObservations;
        pRefKF = mpRefKF;
        Pos = mWorldPos;
    }

    if (observations.empty())
    {
        return;
    }
    Eigen::Vector3f normal;
    normal.setZero();
    int n = 0;
    for (auto& [pKF, indexes] : observations)
    {
        int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);

        if (leftIndex != -1)
        {
            Eigen::Vector3f Owi = pKF->GetCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
        if (rightIndex != -1)
        {
            Eigen::Vector3f Owi = pKF->GetCameraCenter();
            Eigen::Vector3f normali = Pos - Owi;
            normal = normal + normali / normali.norm();
            n++;
        }
    }

    Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
    const float dist = PC.norm();

    std::tuple<int, int> indexes = observations[pRefKF];
    int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);
    int level;
    if (pRefKF->NLeft == -1)
    {
        level = pRefKF->mvKeysUn[leftIndex].octave;
    }
    else if (leftIndex != -1)
    {
        level = pRefKF->mvKeys[leftIndex].octave;
    }
    else
    {
        level = pRefKF->mvKeysRight[rightIndex - pRefKF->NLeft].octave;
    }

    const float levelScaleFactor = pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        std::lock_guard<std::mutex> lock3(mMutexPos);
        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
        mNormalVector = normal / n;
    }
}

void MapPoint::SetNormalVector(const Eigen::Vector3f& normal)
{
    std::lock_guard<std::mutex> lock3(mMutexPos);
    mNormalVector = normal;
}

float MapPoint::GetMinDistanceInvariance()
{
    std::lock_guard<std::mutex> lock(mMutexPos);
    return 0.8f * mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    std::lock_guard<std::mutex> lock(mMutexPos);
    return 1.2f * mfMaxDistance;
}

int MapPoint::PredictScale(const float& currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        std::lock_guard<std::mutex> lock(mMutexPos);
        ratio = mfMaxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
    if (nScale < 0)
    {
        nScale = 0;
    }
    else if (nScale >= pKF->mnScaleLevels)
    {
        nScale = pKF->mnScaleLevels - 1;
    }
    return nScale;
}

int MapPoint::PredictScale(const float& currentDist, Frame* pF)
{
    float ratio;
    {
        std::lock_guard<std::mutex> lock(mMutexPos);
        ratio = mfMaxDistance / currentDist;
    }

    int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
    if (nScale < 0)
    {
        nScale = 0;
    }
    else if (nScale >= pF->mnScaleLevels)
    {
        nScale = pF->mnScaleLevels - 1;
    }
    return nScale;
}

void MapPoint::PrintObservations()
{
    std::lock_guard<std::mutex> lock(mMutexFeatures);
    for (const auto& [pKFi, indexes] : mObservations)
    {
        int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);
        (void)pKFi;
        (void)leftIndex;
        (void)rightIndex;
    }
}

Map* MapPoint::GetMap()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mpMap;
}

void MapPoint::UpdateMap(Map* pMap)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mpMap = pMap;
}

}  // namespace ORB_SLAM3
