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

#include "Atlas.h"

#include <mutex>
#include <set>
#include <vector>

namespace ORB_SLAM3
{

Atlas::Atlas()
{
    mpCurrentMap = static_cast<Map*>(NULL);
}

Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid)
{
    mpCurrentMap = static_cast<Map*>(NULL);
    CreateNewMap();
}

Atlas::~Atlas()
{
    if (mpCurrentMap)
    {
        delete mpCurrentMap;
        mpCurrentMap = static_cast<Map*>(NULL);
    }
}

void Atlas::CreateNewMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Creation of new map with id: " << Map::nNextId << std::endl;
    if (mpCurrentMap)
    {
        if (mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
        {
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1;  // The init KF is the next of current maximum
        }
        delete mpCurrentMap;
        mpCurrentMap = static_cast<Map*>(NULL);
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "Creation of new map with last KF id: " << mnLastInitKFidMap << std::endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::AddKeyFrame(KeyFrame* pKF)
{
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP)
{
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
{
    //Check if the camera already exists
    bool bAlreadyInMap = false;
    int index_cam = -1;
    for (size_t i = 0; i < mvpCameras.size(); ++i)
    {
        GeometricCamera* pCam_i = mvpCameras[i];
        if (pCam->GetType() != pCam_i->GetType())
        {
            continue;
        }
        if (pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            if (((Pinhole*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
        else if (pCam->GetType() == GeometricCamera::CAM_METASHAPE)
        {
            if (((Metashape*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
    }

    if (bAlreadyInMap)
    {
        return mvpCameras[index_cam];
    }
    else
    {
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

std::vector<GeometricCamera*> Atlas::GetAllCameras()
{
    return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs)
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

void Atlas::clearMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    if (mpCurrentMap)
    {
        delete mpCurrentMap;
        mpCurrentMap = static_cast<Map*>(NULL);
    }
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    if (!mpCurrentMap)
    {
        CreateNewMap();
    }
    return mpCurrentMap;
}

long unsigned int Atlas::GetNumLivedKF()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap ? mpCurrentMap->GetAllKeyFrames().size() : 0;
}

long unsigned int Atlas::GetNumLivedMP()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap ? mpCurrentMap->GetAllMapPoints().size() : 0;
}

}  //namespace ORB_SLAM3
