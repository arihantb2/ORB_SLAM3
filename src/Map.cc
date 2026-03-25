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

#include <mutex>

namespace ORB_SLAM3
{

long unsigned int Map::nNextId = 0;

Map::Map()
    : mnMaxKFid(0),
      mnBigChangeIdx(0),
      mnMapChange(0),
      mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
      mbFail(false),
      mIsInUse(false),
      mbBad(false),
      mnMapChangeNotified(0)
{
    mnId = nNextId++;
}

Map::Map(int initKFid)
    : mnInitKFid(initKFid),
      mnMaxKFid(initKFid),
      /*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0),
      mIsInUse(false),
      mbBad(false),
      mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
      mnMapChange(0),
      mbFail(false),
      mnMapChangeNotified(0)
{
    mnId = nNextId++;
}

Map::~Map()
{
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame* pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
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
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint* pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame* pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if (mspKeyFrames.size() > 0)
    {
        if (pKF->mnId == mpKFlowerID->mnId)
        {
            std::vector<KeyFrame*> vpKFs = std::vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
            sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
}

std::vector<MapPoint*> Map::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<MapPoint*>(mspMapPoints.begin(), mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

std::vector<MapPoint*> Map::GetReferenceMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF()
{
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
    //    for(std::set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
    //        delete *sit;

    for (std::set<KeyFrame*>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
    {
        KeyFrame* pKF = *sit;
        pKF->UpdateMap(static_cast<Map*>(NULL));
        //        delete *sit;
    }

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
    std::unique_lock<std::mutex> lock(mMutexMap);

    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for (std::set<KeyFrame*>::iterator sit = mspKeyFrames.begin(); sit != mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
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
    for (std::set<MapPoint*>::iterator sit = mspMapPoints.begin(); sit != mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    if (mpKFlowerID)
    {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

}  //namespace ORB_SLAM3
