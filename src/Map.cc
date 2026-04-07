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

#include "Map.h"

#include "KeyFrame.h"
#include "MapPoint.h"

#include <algorithm>
#include <mutex>

namespace ORB_SLAM3
{

long unsigned int Map::nNextId = 0;

Map::Map()
    : mnMaxKFid(0),
      mnBigChangeIdx(0),
      mnMapChange(0),
      mpFirstRegionKF(nullptr),
      mnMapChangeNotified(0)
{
    mnId = nNextId++;
}

Map::Map(int initKFid)
    : mnInitKFid(initKFid),
      mnMaxKFid(initKFid),
      mnBigChangeIdx(0),
      mpFirstRegionKF(nullptr),
      mnMapChange(0),
      mnMapChangeNotified(0)
{
    mnId = nNextId++;
}

Map::~Map()
{
    for (MapPoint* pMP : mspMapPoints)
        delete pMP;
    mspMapPoints.clear();

    for (KeyFrame* pKF : mspKeyFrames)
        delete pKF;
    mspKeyFrames.clear();

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame* pKF)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    if (mspKeyFrames.empty())
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << std::endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    if (pKF->mnId > mnMaxKFid)
    {
        mnMaxKFid = pKF->mnId;
    }
    if (pKF->mnId < mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
}

void Map::AddMapPoint(MapPoint* pMP)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint* pMP)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    // Object lifetime: pMP may still be referenced by callers after SetBadFlag().
    // Deletion is deferred to Map::~Map() which deletes all remaining objects.
}

void Map::EraseKeyFrame(KeyFrame* pKF)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if (!mspKeyFrames.empty())
    {
        if (pKF->mnId == mpKFlowerID->mnId)
        {
            mpKFlowerID = *std::min_element(mspKeyFrames.begin(), mspKeyFrames.end(),
                                             KeyFrame::lId);
        }
    }
    else
    {
        mpKFlowerID = nullptr;
    }
    // Object lifetime: deferred to Map::~Map().
}

void Map::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    std::vector<KeyFrame*> v(mspKeyFrames.begin(), mspKeyFrames.end());
    std::sort(v.begin(), v.end(), KeyFrame::lId);
    return v;
}

std::vector<MapPoint*> Map::GetAllMapPoints()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    std::vector<MapPoint*> v(mspMapPoints.begin(), mspMapPoints.end());
    std::sort(v.begin(), v.end(), [](MapPoint* a, MapPoint* b) { return a->mnId < b->mnId; });
    return v;
}

long unsigned int Map::MapPointsInMap()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

std::vector<MapPoint*> Map::GetReferenceMapPoints()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}

long unsigned int Map::GetInitKFid()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    for (KeyFrame* pKF : mspKeyFrames)
        pKF->UpdateMap(nullptr);

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}

void Map::ApplyScaledRotation(const Sophus::SE3f& T, const float s, const bool bScaledVel)
{
    std::lock_guard<std::mutex> lock(mMutexMap);

    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for (KeyFrame* pKF : mspKeyFrames)
    {
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw * Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if (!bScaledVel)
        {
            pKF->SetVelocity(Ryw * Vw);
        }
        else
        {
            pKF->SetVelocity(Ryw * Vw * s);
        }
    }
    for (MapPoint* pMP : mspMapPoints)
    {
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::ChangeId(long unsigned int nId)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    if (mpKFlowerID)
    {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

}  //namespace ORB_SLAM3
