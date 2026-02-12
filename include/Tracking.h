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

#ifndef TRACKING_H
#define TRACKING_H

#include <list>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <string>
#include <vector>

#include "Frame.h"
#include "ImuTypes.h"
#include "ORBVocabulary.h"
#include "StereoDebug.h"

namespace ORB_SLAM3
{

class Viewer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;
class KeyFrame;
class KeyFrameDatabase;
class Map;
class MapDrawer;
class MapPoint;
class ORBextractor;
class GeometricCamera;

class Tracking
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System* pSys, ORBVocabulary* pVoc, MapDrawer* pMapDrawer, Atlas* pAtlas, KeyFrameDatabase* pKFDB,
             const std::string& strSettingPath, const int sensor, Settings* settings, const bool newMaps);

    ~Tracking();

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    Sophus::SE3f GrabImageStereo(const cv::Mat& imRectLeft, const cv::Mat& imRectRight, const double& timestamp);
    Sophus::SE3f GrabImageMonocular(const cv::Mat& im, const double& timestamp);

    void GrabImuData(const IMU::Point& imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    bool isLastFrameKeyframe();

    void UpdateFrameIMU(const float s, const IMU::Bias& b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame() { return mpLastKeyFrame; }

    void CreateMapInAtlas();

    int GetMatchesInliers();
    MonocularDebugFrame GetMonocularDebugFrame() const;
    StereoDebugFrame GetStereoDebugFrame() const;

    float GetImageScale();

    // Tracking states
    enum eTrackingState
    {
        NO_IMAGES_YET = 0,
        NOT_INITIALIZED = 1,
        OK = 2,
        LOST = 3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;
    int mMonocularInitMinKeypoints = 100;
    float mMonocularInitNNRatio = 0.9f;
    int mMonocularInitSearchWindowSize = 100;
    int mMonocularInitMinMatches = 100;

    int mStereoInitMinKeypoints = 500;
    float mReferenceKeyframeNNRatio = 0.7f;
    int mReferenceKeyframeMinBoWMatches = 15;
    int mReferenceKeyframeMinOptimizedMapMatches = 10;
    float mMotionModelNNRatio = 0.9f;
    int mMotionModelProjectionSearchThStereo = 7;
    int mMotionModelProjectionSearchThMono = 30;
    int mMotionModelMinInitialMatches = 20;
    int mMotionModelRetryProjectionSearchThStereo = 14;
    int mMotionModelRetryProjectionSearchThMono = 60;
    int mMotionModelMinRetryMatches = 20;
    int mMotionModelMinOptimizedMapMatches = 10;
    int mLocalMapGenericMinInliers = 10;
    int mLocalMapVisualMinInliers = 30;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    std::list<Sophus::SE3f> mlRelativeFramePoses;
    std::list<KeyFrame*> mlpReferences;
    std::list<double> mlFrameTimes;
    std::list<bool> mlbLost;

    // frames with estimated pose
    int mTrackedFr;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0;     // time-stamp of first read frame
    double t0vis;  // time-stamp of first inserted keyframe
    double t0IMU;  // time-stamp of IMU initialization
    bool mFastInit = false;

    std::vector<MapPoint*> GetLocalMapMPS();

    bool mbWriteStats;

protected:
    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Stereo Initialization
    void StereoInitialization();

    // Monocular Initialization
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool PredictStateIMU();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();

    bool mbMapUpdated;

    bool mbAtlasNewMaps;

    // Imu preintegration from last frame
    IMU::Preintegrated* mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib* mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    // ORB
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    // BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    // Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    // System
    System* mpSystem;

    // Drawers
    Viewer* mpViewer;
    MapDrawer* mpMapDrawer;

    // Atlas
    Atlas* mpAtlas;

    // Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    float mImuFreq;
    double mImuPer;
    bool mInsertKFsLost;

    // New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;

    // Threshold close/far points
    // Points seen as close by the stereo sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    // Current matches in frame
    int mnMatchesInliers;

    // Last Frame, KeyFrame Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    double mTimeStampLost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    // Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    std::list<MapPoint*> mlpTemporalPoints;

    GeometricCamera *mpCamera, *mpCamera2;

    int initID, lastID;

    Sophus::SE3f mTlr;

    void loadFromSettings(Settings* settings);

    MonocularDebugFrame BuildMonocularDebugFrame(const Frame& frame, const cv::Mat& image) const;
    void UpdateMonocularDebugFrame(const cv::Mat& image);

    StereoDebugFrame BuildStereoDebugFrameMetashapePinhole(const Frame& frame, const cv::Mat& leftRectified,
                                                           const cv::Mat& rightRectified) const;
    StereoDebugFrame BuildStereoDebugFrameFisheye(const Frame& frame, const cv::Mat& leftRectified,
                                                  const cv::Mat& rightRectified) const;
    void UpdateStereoDebugFrame(const cv::Mat& leftRectified, const cv::Mat& rightRectified);

    mutable std::mutex mMutexMonocularDebugFrame;
    MonocularDebugFrame mLastMonocularDebugFrame;

    mutable std::mutex mMutexStereoDebugFrame;
    StereoDebugFrame mLastStereoDebugFrame;
};

}  // namespace ORB_SLAM3

#endif  // TRACKING_H
