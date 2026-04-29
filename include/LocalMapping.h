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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "LocalMappingResult.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <list>
#include <mutex>

namespace ORB_SLAM3
{

class System;
class Settings;
class Tracking;
class Atlas;
class KeyFrame;
class Map;
class MapPoint;

class LocalMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, Settings* settings);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    // Returns true if outer loop needs to break
    bool RunLoop();

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Synchronous mode: when enabled, InsertKeyFrame() blocks until the full
    // LocalMapping iteration for the inserted frame completes before returning.
    // No separate LocalMapping thread is used; the caller's thread drives mapping.
    void SetSynchronousMode(bool bSync) { mbSynchronousMode = bSync; }
    bool IsSynchronousMode() const { return mbSynchronousMode; }

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue()
    {
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    /// Register a callback invoked from the LocalMapping thread at the end of
    /// every iteration that processes a KeyFrame. Pass nullptr to clear.
    /// Thread-safe: may be called from any thread.
    void SetCallback(LocalMappingCallback cb);

    /// In synchronous mode, block the caller until LocalMapping has finished
    /// processing the last inserted KeyFrame (culling, descriptors, covisibility).
    /// Returns immediately in async mode or if LocalMapping is shutting down.
    void WaitForMappingComplete();

    double mFirstTs;
    int mnMatchesInliers;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

    // Descriptor-type-appropriate matching thresholds (read from Settings)
    int mMatchThLow  = 50;
    int mMatchThHigh = 100;

    // LBA throttling: min interval between optimizations
    double mOptimizeEveryTSeconds = 0.0;

    // RunLoop
    int mMinKeyframesForLBA = 2;

    // MapPointCulling
    int mMPCullingMinObs = 2;  // 3 for stereo
    int mMPCullingMinKFAgeForObsCheck = 2;
    int mMPCullingMaxKFAgeInRecent = 3;
    float mMPCullingMinFoundRatio = 0.25f;

    // CreateNewMapPoints
    int mCreateNewMapPointsCovisibility = 30;  // 10 for stereo
    float mCreateNewMapPointsMatchRatio = 0.6f;
    float mCreateNewMapPointsMinBaselineDepthRatio = 0.01f;
    float mCreateNewMapPointsMaxCosParallax = 0.9998f;
    float mCreateNewMapPointsScaleConsistencyFactor = 1.5f;

    // SearchInNeighbors
    int mSearchInNeighborsNumNeighborKFs = 30;
    int mSearchInNeighborsNumSecondNeighbors = 20;
    int mSearchInNeighborsMaxTemporalNeighbors = 20;

    // KeyFrameCulling
    float mKeyFrameCullingRedundantRatio = 0.9f;
    int mKeyFrameCullingMinObsInOthers = 3;
    int mKeyFrameCullingMaxKeyframesToCheck = 100;
    int mKeyFrameCullingEarlyExitAfterAbort = 20;

protected:
    void loadFromSettings(Settings* settings);

    void SetNewKeyFrame();
    bool CheckNewKeyFrames();
    ProcessNewKeyFrameResult ProcessNewKeyFrame();
    CreateNewMapPointsResult CreateNewMapPoints();

    MapPointCullingResult MapPointCulling();
    SearchInNeighborsResult SearchInNeighbors();
    KeyFrameCullingResult KeyFrameCulling();

    System* mpSystem;

    bool mbMonocular;
    bool mbSynchronousMode = false;

    // Synchronous-mode coordination.
    // mCVNewKF    — wakes the LocalMapping thread when a KF is enqueued
    //               (guarded by mMutexNewKFs, which already protects the queue).
    // mCVSyncIterDone / mMutexSyncIterDone / mbSyncIterDone
    //             — wakes the Tracking thread when LocalMapping finishes the
    //               iteration triggered by the last InsertKeyFrame call.
    std::condition_variable mCVNewKF;
    std::mutex mMutexSyncIterDone;
    std::condition_variable mCVSyncIterDone;
    bool mbSyncIterDone = true;

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    double prevOptimizedKFTimestamp = -1.0;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    bool bInitializing;

    LocalMappingCallback mCallback;
    std::mutex mMutexCallback;
    std::atomic<uint64_t> mIterationCounter{0};

    //DEBUG
    std::ofstream f_lm;
};

}  // namespace ORB_SLAM3

#endif  // LOCALMAPPING_H
