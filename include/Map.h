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

#ifndef MAP_H
#define MAP_H

#include <list>
#include <map>
#include <mutex>
#include <set>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "MapPointPool.h"

namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;
class Atlas;
class GeometricCamera;

class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);

    // Factory methods: allocate a slot from the pool and construct a MapPoint.
    // Use these instead of `new MapPoint(...)` for all map-owned points so that
    // memory is managed by the pool and released when the point is retired.
    MapPoint* CreateMapPoint(const Eigen::Vector3f& Pos, KeyFrame* pRefKF);
    MapPoint* CreateMapPoint(const double invDepth, cv::Point2f uv_init,
                             KeyFrame* pRefKF, KeyFrame* pHostKF);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFrame* GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void ApplyScaledRotation(const Sophus::SE3f& T, const float s, const bool bScaledVel = false);

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void printReprojectionError(std::list<KeyFrame*>& lpLocalWindowKFs, KeyFrame* mpCurrentKF, std::string& name,
                                std::string& name_folder);

    std::vector<KeyFrame*> mvpKeyFrameOrigins;
    KeyFrame* mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:
    long unsigned int mnId;

    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    //long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    bool mIsInUse;
    bool mbBad = false;

    // Pool allocator for map-owned MapPoints.
    MapPointPool mMapPointPool;

    // Mutex
    std::mutex mMutexMap;
};

}  //namespace ORB_SLAM3

#endif  // MAP_H
