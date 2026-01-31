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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdlib.h>
#include <unistd.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ImuTypes.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM3
{

class Viewer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;
class KeyFrame;
class Map;
class MapPoint;
class KeyFrameDatabase;

using std::string;
using std::vector;

class System
{
public:
    // Input sensor
    enum eSensor
    {
        MONOCULAR = 0,
        STEREO = 1,
        IMU_MONOCULAR = 3,
        IMU_STEREO = 4,
    };

    // File type
    enum FileType
    {
        TEXT_FILE = 0,
        BINARY_FILE = 1,
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string& strVocFile, const string& strSettingsFile, const eSensor sensor, const bool bUseViewer = true,
           const bool bTurnOffLC = false);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp,
                             const vector<IMU::Point>& vImuMeas = vector<IMU::Point>());

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat& im, const double& timestamp,
                                const vector<IMU::Point>& vImuMeas = vector<IMU::Point>());

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();
    bool isShutDown();

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
    // Detected keypoints in the last processed frame (Frame::mvKeys).
    // These are in the same pixel coordinates as the image fed into Track* (before undistortion),
    // so they are suitable for direct overlay.
    std::vector<cv::KeyPoint> GetDetectedKeyPoints();
    std::vector<cv::KeyPoint> GetDetectedKeyPointsRight();

    // Subset of detected keypoints that are currently tracked (i.e., have a MapPoint association)
    // and are not flagged as outliers. Useful if you only want "successful tracks".
    std::vector<cv::KeyPoint> GetInlierKeyPoints();
    std::vector<cv::KeyPoint> GetInlierKeyPointsRight();

    // Subset of detected keypoints that are currently tracked (i.e., have a MapPoint association)
    // and are flagged as outliers. Useful if you only want "failed tracks".
    std::vector<cv::KeyPoint> GetOutlierKeyPoints();
    std::vector<cv::KeyPoint> GetOutlierKeyPointsRight();
 
    // Keyframe trajectory in the world frame.
    std::vector<Sophus::SE3f> GetKeyframeTrajectory();

    Tracking* GetTracker() const;
    int GetLastBigChangeIdx();
    vector<KeyFrame*> GetKeyFrames();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    float GetImageScale();

private:
    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Shutdown flag
    bool mbShutDown;

    // Tracking state
    int mTrackingState;

    std::vector<MapPoint*> mTrackedMapPoints;

    // Undistorted keypoints (mvKeysUn) of last frame
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
 
    // Original/detected keypoints (mvKeys) of last frame, suitable for overlay on the input image
    std::vector<cv::KeyPoint> mDetectedKeyPoints;
    std::vector<cv::KeyPoint> mDetectedKeyPointsRight;

    // Outlier flags aligned with mDetectedKeyPoints / mTrackedMapPoints
    std::vector<bool> mTrackedOutliers;

    std::mutex mMutexState;

    string mStrVocabularyFilePath;

    Settings* settings_;
};

}  // namespace ORB_SLAM3

#endif  // SYSTEM_H
