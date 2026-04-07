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

#include <atomic>
#include <list>
#include <map>
#include <mutex>
#include <set>
#include <sophus/se3.hpp>
#include <string>
#include <unordered_set>
#include <vector>

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
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs);
    void InformNewBigChange();
    [[nodiscard]] int GetLastBigChangeIdx();

    [[nodiscard]] std::vector<KeyFrame*> GetAllKeyFrames();
    [[nodiscard]] std::vector<MapPoint*> GetAllMapPoints();
    [[nodiscard]] std::vector<MapPoint*> GetReferenceMapPoints();

    [[nodiscard]] long unsigned int MapPointsInMap();
    [[nodiscard]] long unsigned KeyFramesInMap();

    [[nodiscard]] long unsigned int GetId();

    [[nodiscard]] long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    [[nodiscard]] long unsigned int GetMaxKFid();

    [[nodiscard]] KeyFrame* GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    [[nodiscard]] bool IsInUse();

    void SetBad();
    [[nodiscard]] bool IsBad();

    void clear();

    [[nodiscard]] int GetMapChangeIndex();
    void IncreaseChangeIndex();
    [[nodiscard]] int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void ApplyScaledRotation(const Sophus::SE3f& T, const float s, const bool bScaledVel = false);

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    [[nodiscard]] unsigned int GetLowerKFID();

    void printReprojectionError(std::list<KeyFrame*>& lpLocalWindowKFs, KeyFrame* mpCurrentKF, std::string& name,
                                std::string& name_folder);

    std::vector<KeyFrame*> mvpKeyFrameOrigins;
    KeyFrame* mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    std::atomic<bool> mbFail{false};

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:
    long unsigned int mnId;

    std::unordered_set<MapPoint*> mspMapPoints;
    std::unordered_set<KeyFrame*> mspKeyFrames;

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

    std::atomic<bool> mIsInUse{false};
    std::atomic<bool> mbBad{false};

    // Mutex
    std::mutex mMutexMap;
};

}  //namespace ORB_SLAM3

#endif  // MAP_H
