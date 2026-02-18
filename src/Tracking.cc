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

#include "Tracking.h"
#include "Verbose.h"

#include "Atlas.h"
#include "CameraModels/GeometricCamera.h"
#include "G2oTypes.h"
#include "GeometricTools.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Settings.h"
#include "System.h"
#include "Viewer.h"

#include <algorithm>
#include <utility>
#include <unordered_set>

#include <mutex>

namespace ORB_SLAM3
{

Tracking::Tracking(System* pSys, ORBVocabulary* pVoc, MapDrawer* pMapDrawer, Atlas* pAtlas, KeyFrameDatabase* pKFDB,
                   const std::string& strSettingPath, const int sensor, Settings* settings, const bool newMaps)
    : mState(NO_IMAGES_YET),
      mSensor(sensor),
      mbMapUpdated(false),
      mpORBVocabulary(pVoc),
      mpKeyFrameDB(pKFDB),
      mbReadyToInitializate(false),
      mpSystem(pSys),
      mpViewer(NULL),
      mpMapDrawer(pMapDrawer),
      mpAtlas(pAtlas),
      mnInitialFrameId(0),
      mbCreatedMap(false),
      mnFirstFrameId(0),
      mpLastKeyFrame(static_cast<KeyFrame*>(NULL)),
      mbAtlasNewMaps(newMaps)
{
    // Load camera parameters from settings file
    if (!settings)
    {
        throw std::runtime_error("Settings must be provided (File.version \"1.0\" format only).");
    }
    loadFromSettings(settings);

    initID = 0;
    lastID = 0;

    std::vector<GeometricCamera*> vpCams = mpAtlas->GetAllCameras();
    Verbose::Print(Verbose::VERBOSITY_QUIET) << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    for (GeometricCamera* pCam : vpCams)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET) << "Camera " << pCam->GetId();
        if (pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET) << " is pinhole" << std::endl;
        }
        else if (pCam->GetType() == GeometricCamera::CAM_METASHAPE)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET) << " is metashape" << std::endl;
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET) << " is unknown" << std::endl;
        }
    }
}

Tracking::~Tracking() {}

void Tracking::loadFromSettings(Settings* settings)
{
    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    if (settings->needToUndistort())
    {
        mDistCoef = settings->camera1DistortionCoef();
    }
    else
    {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    }

    //TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK = mpCamera->toK();
    mK_ = mpCamera->toK_();

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    //ORB parameters
    int nFeatures = settings->nFeatures();
    int nInitFeatures = settings->nInitFeatures();
    int nLevels = settings->nLevels();
    int fIniThFAST = settings->initThFAST();
    int fMinThFAST = settings->minThFAST();
    float fScaleFactor = settings->scaleFactor();

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }
    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        mpIniORBextractor = new ORBextractor(nInitFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }

    // Monocular initialization thresholds
    mMonocularInitSearchWindowSize = settings->monocularInitSearchWindowSize();
    mMonocularInitMinKeypoints = settings->monocularInitMinKeypoints();
    mMonocularInitNNRatio = settings->monocularInitNNRatio();
    mMonocularInitMinMatches = settings->monocularInitMinMatches();

    // Stereo initialization thresholds
    mStereoInitMinKeypoints = settings->stereoInitMinKeypoints();

    // Reference keyframe tracking thresholds
    mReferenceKeyframeNNRatio = settings->referenceKeyframeNNRatio();
    mReferenceKeyframeMinBoWMatches = settings->referenceKeyframeMinBoWMatches();
    mReferenceKeyframeMinOptimizedMapMatches = settings->referenceKeyframeMinOptimizedMapMatches();
    mReferenceKeyframeQuadSearchWindowSize = settings->referenceKeyframeQuadSearchWindowSize();
    mbUseQuadMatchingReferenceKeyFrame = settings->stereoUseQuadMatchingReferenceKeyFrame();

    // Motion model tracking thresholds
    mMotionModelNNRatio = settings->motionModelNNRatio();
    mMotionModelProjectionSearchThStereo = settings->motionModelProjectionSearchThStereo();
    mMotionModelProjectionSearchThMono = settings->motionModelProjectionSearchThMono();
    mMotionModelMinInitialMatches = settings->motionModelMinInitialMatches();
    mMotionModelQuadSearchWindowSize = settings->motionModelQuadSearchWindowSize();
    mbUseQuadMatchingMotionModel = settings->stereoUseQuadMatchingMotionModel();
    mMotionModelRetryProjectionSearchThStereo = settings->motionModelRetryProjectionSearchThStereo();
    mMotionModelRetryProjectionSearchThMono = settings->motionModelRetryProjectionSearchThMono();
    mMotionModelMinRetryMatches = settings->motionModelMinRetryMatches();
    mMotionModelQuadSearchWindowSizeRetry = settings->motionModelQuadSearchWindowSizeRetry();
    mMotionModelMinOptimizedMapMatches = settings->motionModelMinOptimizedMapMatches();

    // Local map tracking success thresholds
    mLocalMapGenericMinInliers = settings->localMapGenericMinInliers();
    mLocalMapVisualMinInliers = settings->localMapVisualMinInliers();

    // New keyframe decision thresholds
    mNewKFMinTrackedClosePoints = settings->newKFMinTrackedClosePoints();
    mNewKFMinNonTrackedClosePoints = settings->newKFMinNonTrackedClosePoints();
    mNewKFRefRatioMono = settings->newKFRefRatioMono();
    mNewKFRefRatioStereoFewKFs = settings->newKFRefRatioStereoFewKFs();
    mNewKFRefRatioStereo = settings->newKFRefRatioStereo();
    mNewKFWeakTrackingRatio = settings->newKFWeakTrackingRatio();
    mNewKFMinInliers = settings->newKFMinInliers();
    mNewKFMaxKFsInQueue = settings->newKFMaxKFsInQueue();
    mLostResetMinKFs = settings->lostResetMinKFs();

    // IMU parameters
    Sophus::SE3f Tbc = settings->Tbc();
    mInsertKFsLost = settings->insertKFsWhenLost();
    mImuFreq = settings->imuFrequency();
    mImuPer = 0.001;  //1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
    float Ng = settings->noiseGyro();
    float Na = settings->noiseAcc();
    float Ngw = settings->gyroWalk();
    float Naw = settings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
}

void Tracking::SetLocalMapper(LocalMapping* pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing* pLoopClosing)
{
    mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
}

Sophus::SE3f Tracking::GrabImageStereo(const cv::Mat& imageLeft, const cv::Mat& imageRight, const double& timestamp)
{
    if (imageLeft.channels() != 1)
    {
        throw std::runtime_error("[Tracking::GrabImageStereo]: Input image must be grayscale");
    }
    if (imageRight.channels() != 1)
    {
        throw std::runtime_error("[Tracking::GrabImageStereo]: Input image must be grayscale");
    }

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;
    Verbose::Print(Verbose::VERBOSITY_QUIET) << "[-] TRACKING_STEREO" << std::endl;
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;
    if (mSensor == System::STEREO)
    {
        mCurrentFrame = Frame(imageLeft, imageRight, timestamp, mpORBextractorLeft, mpORBextractorRight,
                              mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);
    }
    else if (mSensor == System::IMU_STEREO)
    {
        mCurrentFrame = Frame(imageLeft, imageRight, timestamp, mpORBextractorLeft, mpORBextractorRight,
                              mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera, &mLastFrame, *mpImuCalib);
    }

    Track();
    UpdateStereoDebugFrame(imageLeft, imageRight);
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    return mCurrentFrame.GetPose();
}

Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat& image, const double& timestamp)
{
    if (image.channels() != 1)
    {
        throw std::runtime_error("[Tracking::GrabImageMonocular]: Input image must be grayscale");
    }

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;
    Verbose::Print(Verbose::VERBOSITY_QUIET) << "[-] TRACKING_MONOCULAR" << std::endl;
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;
    if (mSensor == System::MONOCULAR)
    {
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET || (lastID - initID) < mMaxFrames)
        {
            mCurrentFrame =
                Frame(image, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
        }
        else
        {
            mCurrentFrame =
                Frame(image, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
        }
    }
    else if (mSensor == System::IMU_MONOCULAR)
    {
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
        {
            mCurrentFrame = Frame(image, timestamp, mpIniORBextractor, mpORBVocabulary, mpCamera, mDistCoef, mbf,
                                  mThDepth, &mLastFrame, *mpImuCalib);
        }
        else
        {
            mCurrentFrame = Frame(image, timestamp, mpORBextractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf,
                                  mThDepth, &mLastFrame, *mpImuCalib);
        }
    }

    lastID = mCurrentFrame.mnId;
    Track();
    UpdateMonocularDebugFrame(image);
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    return mCurrentFrame.GetPose();
}

void Tracking::GrabImuData(const IMU::Point& imuMeasurement)
{
    std::unique_lock<std::mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

void Tracking::PreintegrateIMU()
{
    if (!mCurrentFrame.mpPrevFrame)
    {
        mCurrentFrame.setIntegrated();
        return;
    }

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    if (mlQueueImuData.size() == 0)
    {
        mCurrentFrame.setIntegrated();
        return;
    }

    while (true)
    {
        std::unique_lock<std::mutex> lock(mMutexImuQueue);
        if (!mlQueueImuData.empty())
        {
            IMU::Point* m = &mlQueueImuData.front();
            if (m->t < mCurrentFrame.mpPrevFrame->mTimeStamp - mImuPer)
            {
                mlQueueImuData.pop_front();
            }
            else if (m->t < mCurrentFrame.mTimeStamp - mImuPer)
            {
                mvImuFromLastFrame.push_back(*m);
                mlQueueImuData.pop_front();
            }
            else
            {
                mvImuFromLastFrame.push_back(*m);
                break;
            }
        }
        else
        {
            break;
        }
    }

    const int n = mvImuFromLastFrame.size() - 1;
    if (n == 0)
    {
        return;
    }

    IMU::Preintegrated* pImuPreintegratedFromLastFrame =
        new IMU::Preintegrated(mLastFrame.mImuBias, mCurrentFrame.mImuCalib);

    for (int i = 0; i < n; i++)
    {
        float tstep;
        Eigen::Vector3f acc, angVel;
        if ((i == 0) && (i < (n - 1)))
        {
            float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            float tini = mvImuFromLastFrame[i].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                   (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tini / tab)) *
                  0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                      (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tini / tab)) *
                     0.5f;
            tstep = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        else if (i < (n - 1))
        {
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a) * 0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w) * 0.5f;
            tstep = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
        }
        else if ((i > 0) && (i == (n - 1)))
        {
            float tab = mvImuFromLastFrame[i + 1].t - mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i + 1].t - mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a + mvImuFromLastFrame[i + 1].a -
                   (mvImuFromLastFrame[i + 1].a - mvImuFromLastFrame[i].a) * (tend / tab)) *
                  0.5f;
            angVel = (mvImuFromLastFrame[i].w + mvImuFromLastFrame[i + 1].w -
                      (mvImuFromLastFrame[i + 1].w - mvImuFromLastFrame[i].w) * (tend / tab)) *
                     0.5f;
            tstep = mCurrentFrame.mTimeStamp - mvImuFromLastFrame[i].t;
        }
        else if ((i == 0) && (i == (n - 1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp - mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc, angVel, tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc, angVel, tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();
}

bool Tracking::PredictStateIMU()
{
    if (!mCurrentFrame.mpPrevFrame)
    {
        return false;
    }

    if (mbMapUpdated && mpLastKeyFrame)
    {
        const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        Eigen::Matrix3f Rwb2 =
            IMU::NormalizeRotation(Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                               Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz + Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else if (!mbMapUpdated)
    {
        const Eigen::Vector3f twb1 = mLastFrame.GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 =
            IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                               Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz + Rwb1 * mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

        mCurrentFrame.mImuBias = mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }

    return false;
}

void Tracking::ResetFrameIMU()
{
    // TODO To implement...
}

void Tracking::PrepareFrameForTracking()
{
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mpLastKeyFrame)
    {
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());
    }

    if (mState == NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && !mbCreatedMap)
    {
        PreintegrateIMU();
    }
    mbCreatedMap = false;
}

void Tracking::UpdateMapChangeState(Map* pCurrentMap)
{
    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if (nCurMapChangeIndex > nMapChangeIndex)
    {
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }
}

bool Tracking::Initialize()
{
    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        StereoInitialization();
    }
    else
    {
        MonocularInitialization();
    }

    if (mState != OK)  // If rightly initialized, mState=OK
    {
        mLastFrame = Frame(mCurrentFrame);
        return false;
    }

    if (mpAtlas->GetAllMaps().size() == 1)
    {
        mnFirstFrameId = mCurrentFrame.mnId;
    }

    return true;
}

void Tracking::UpdateAfterTracking(bool bOK)
{
    // Update drawer
    if (mCurrentFrame.isSet())
    {
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());
    }

    if (!bOK)
    {
        return;
    }

    // Update motion model
    if (mLastFrame.isSet() && mCurrentFrame.isSet())
    {
        const Sophus::SE3f& Tcw_last = mLastFrame.GetPose();
        const Sophus::SE3f& Tcw_cur = mCurrentFrame.GetPose();

        // Relative motion (last camera -> current camera) in Tcw convention
        mVelocity = Tcw_cur * Tcw_last.inverse();

        // Delta position in world frame
        Eigen::Vector3f twc_last = Tcw_last.inverse().translation();
        Eigen::Vector3f twc_cur = Tcw_cur.inverse().translation();
        Eigen::Vector3f delta_w = twc_cur - twc_last;

        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] " << "Velocity (delta position): " << delta_w.transpose()
            << " m, dt: " << std::fixed << std::setprecision(6) << mCurrentFrame.mTimeStamp - mLastFrame.mTimeStamp
            << " s" << std::endl;
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] " << "Velocity norm: " << delta_w.norm() << " m" << std::endl;
        mbVelocity = true;
    }
    else
    {
        mbVelocity = false;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());
    }

    // Clean VO matches
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
        {
            if (pMP->Observations() < 1)
            {
                mCurrentFrame.mvbOutlier[i] = false;
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            }
        }
    }

    // Delete temporal MapPoints
    for (std::list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend;
         lit++)
    {
        MapPoint* pMP = *lit;
        delete pMP;
    }
    mlpTemporalPoints.clear();

    bool bNeedKF = NeedNewKeyFrame();

    // Check if we need to insert a new keyframe
    if (bNeedKF && (bOK || (mInsertKFsLost && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))))
    {
        CreateNewKeyFrame();
    }

    // We allow points with high innovation (considererd outliers by the Huber Function)
    // pass to the new keyframe, so that bundle adjustment will finally decide
    // if they are outliers or not. We don't want next frame to estimate its position
    // with those points so we discard them in the frame. Only has effect if lastframe is tracked
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }
}

bool Tracking::TrackStereo()
{
    Map* pCurrentMap = mpAtlas->GetCurrentMap();
    bool bOK = false;
    if (!mbVelocity && !pCurrentMap->isImuInitialized())
    {
        bOK = mbUseQuadMatchingReferenceKeyFrame ? TrackQuadReferenceKeyFrame() : TrackReferenceKeyFrameWithBoW();
        if (!bOK)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF failed." << std::endl;
        }
    }
    else
    {
        bOK = mbUseQuadMatchingMotionModel ? TrackQuadWithMotionModel() : TrackWithMotionModel();
        if (!bOK)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL failed." << std::endl;
            bOK = mbUseQuadMatchingReferenceKeyFrame ? TrackQuadReferenceKeyFrame() : TrackReferenceKeyFrameWithBoW();
            if (!bOK)
            {
                Verbose::Print(Verbose::VERBOSITY_QUIET)
                    << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF failed (fallback)." << std::endl;
            }
        }
    }

    return bOK;
}

bool Tracking::TrackMonocular()
{
    Map* pCurrentMap = mpAtlas->GetCurrentMap();
    bool bOK = false;
    if (!mbVelocity && !pCurrentMap->isImuInitialized())
    {
        bOK = TrackReferenceKeyFrameWithBoW();
        if (!bOK)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF failed." << std::endl;
        }
    }
    else
    {
        bOK = TrackWithMotionModel();
        if (!bOK)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL failed." << std::endl;
            bOK = TrackReferenceKeyFrameWithBoW();
            if (!bOK)
            {
                Verbose::Print(Verbose::VERBOSITY_QUIET)
                    << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF failed (fallback)." << std::endl;
            }
        }
    }

    return bOK;
}

void Tracking::Track()
{
    if (mpLocalMapper->mbBadImu)
    {
        mpSystem->ResetActiveMap();
        return;
    }

    Map* pCurrentMap = mpAtlas->GetCurrentMap();
    if (!pCurrentMap)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET) << "ERROR: There is not an active map in the atlas" << std::endl;
    }

    if (mState != NO_IMAGES_YET)
    {
        if (mLastFrame.mTimeStamp > mCurrentFrame.mTimeStamp)
        {
            std::unique_lock<std::mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        }
        else if (mCurrentFrame.mTimeStamp > mLastFrame.mTimeStamp + 1.0)
        {
            if (mpAtlas->isInertial())
            {

                if (mpAtlas->isImuInitialized())
                {
                    Verbose::Print(Verbose::VERBOSITY_QUIET)
                        << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << std::endl;
                    mpSystem->ResetActiveMap();
                }
                else
                {
                    Verbose::Print(Verbose::VERBOSITY_QUIET)
                        << "Timestamp jump detected, before IMU initialization. Reseting..." << std::endl;
                    mpSystem->ResetActiveMap();
                }
                return;
            }
        }
    }

    PrepareFrameForTracking();

    // Get Map Mutex -> Map cannot be changed
    std::unique_lock<std::mutex> lock(pCurrentMap->mMutexMapUpdate);

    UpdateMapChangeState(pCurrentMap);

    if (mState == NOT_INITIALIZED)
    {
        Initialize();
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK = false;
        if (mState == OK)
        {
            // Local Mapping might have changed some MapPoints tracked in last frame
            CheckReplacedInLastFrame();

            if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
            {
                bOK = TrackStereo();
            }
            else if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
            {
                bOK = TrackMonocular();
            }

            if (!bOK)
            {
                mState = LOST;
                mTimeStampLost = mCurrentFrame.mTimeStamp;
                Verbose::Print(Verbose::VERBOSITY_QUIET)
                    << "[" << mCurrentFrame.mnId << "] TRACK_LOST. Pose estimation failed" << std::endl;
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_QUIET)
                    << "[" << mCurrentFrame.mnId << "] TRACK_OK. Pose estimation succeeded" << std::endl;
            }
        }
        else if (mState == LOST)
        {
            mpSystem->ResetActiveMap();
            if (mpLastKeyFrame)
            {
                mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
            }
            return;
        }

        if (!mCurrentFrame.mpReferenceKF)
        {
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if (bOK)
        {
            bOK = TrackLocalMap();
            if (!bOK)
            {
                Verbose::Print(Verbose::VERBOSITY_QUIET)
                    << "[" << mCurrentFrame.mnId << "] TRACK_LOCAL_MAP failed." << std::endl;
            }
            else
            {
                mState = OK;
                Verbose::Print(Verbose::VERBOSITY_QUIET)
                    << "[" << mCurrentFrame.mnId << "] TRACK_LOCAL_MAP ok: inliers=" << mnMatchesInliers << std::endl;
            }
        }

        if (!bOK and mState == OK)
        {
            if (mSensor == System::STEREO || mSensor == System::MONOCULAR)
            {
                Verbose::Print(Verbose::VERBOSITY_QUIET)
                    << "[" << mCurrentFrame.mnId
                    << "] Tracking LOST (frames_since_last_kf=" << (mCurrentFrame.mnId - mnLastKeyFrameId) << ")."
                    << std::endl;
            }
            mState = LOST;
            mTimeStampLost = mCurrentFrame.mTimeStamp;
        }

        // Save frame for IMU reset (copy made once mCurrentFrame is fully updated).
        if ((mCurrentFrame.mnId > mnFramesToResetIMU) &&
            (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }

        UpdateAfterTracking(bOK);

        // Reset if the camera get lost
        if (mState == LOST)
        {
            mpSystem->ResetActiveMap();
            return;
        }

        if (!mCurrentFrame.mpReferenceKF)
        {
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
        mLastFrame = Frame(mCurrentFrame);
    }

    if (mState == OK)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        if (mCurrentFrame.isSet())
        {
            Sophus::SE3f Tcr_ = mCurrentFrame.GetPose() * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState == LOST);
        }
        else
        {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }
    }

    // Populate debug correspondences on the current frame for visualization (monocular only).
    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        mCurrentFrame.mDebugFrame2FrameMatches.clear();
        mCurrentFrame.mDebugFrame2RefKfMatches.clear();
        mCurrentFrame.mDebugFrame2LocalMapMatches.clear();

        if (mState == OK)
        {
            // Frame-to-frame (previous vs current) using temporal matches from motion model, if available.
            if (mvTemporalMatches.size() == static_cast<size_t>(mCurrentFrame.N))
            {
                for (int i = 0; i < mCurrentFrame.N; ++i)
                {
                    if (i < 0 || i >= static_cast<int>(mCurrentFrame.mvbOutlier.size()))
                    {
                        continue;
                    }
                    const int j = mvTemporalMatches[i];
                    if (j < 0 || j >= mLastFrame.N)
                    {
                        continue;
                    }
                    if (mCurrentFrame.mvbOutlier[i])
                    {
                        continue;
                    }
                    const cv::Point2f last_pt = mLastFrame.mvKeys[j].pt;
                    const cv::Point2f curr_pt = mCurrentFrame.mvKeys[i].pt;
                    mCurrentFrame.mDebugFrame2FrameMatches.emplace_back(last_pt, curr_pt);
                }
            }

            // Build set for fast membership testing of local map points.
            std::unordered_set<MapPoint*> local_set;
            local_set.reserve(mvpLocalMapPoints.size());
            for (MapPoint* pMP : mvpLocalMapPoints)
            {
                if (pMP)
                {
                    local_set.insert(pMP);
                }
            }

            // Classify remaining inlier map-point matches into reference-keyframe vs local-map.
            for (int i = 0; i < mCurrentFrame.N; ++i)
            {
                if (i < 0 || i >= static_cast<int>(mCurrentFrame.mvpMapPoints.size()) ||
                    i >= static_cast<int>(mCurrentFrame.mvbOutlier.size()))
                {
                    continue;
                }
                if (mCurrentFrame.mvbOutlier[i])
                {
                    continue;
                }

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP)
                {
                    continue;
                }

                // Skip if this feature already has a frame-to-frame temporal match.
                bool is_frame2frame = false;
                if (mvTemporalMatches.size() == static_cast<size_t>(mCurrentFrame.N) && mvTemporalMatches[i] >= 0)
                {
                    is_frame2frame = true;
                }
                if (is_frame2frame)
                {
                    continue;
                }

                const cv::Point2f curr_pt = mCurrentFrame.mvKeys[i].pt;

                // Matches that are also observed in the reference keyframe.
                if (mpReferenceKF && pMP->IsInKeyFrame(mpReferenceKF))
                {
                    auto idx_tuple = pMP->GetIndexInKeyFrame(mpReferenceKF);
                    const int idxKF = std::get<0>(idx_tuple);
                    if (idxKF >= 0 && idxKF < mpReferenceKF->N)
                    {
                        const cv::Point2f ref_pt = mpReferenceKF->mvKeys[idxKF].pt;
                        mCurrentFrame.mDebugFrame2RefKfMatches.emplace_back(ref_pt, curr_pt);
                    }
                    continue;
                }

                // Matches that come from the local map (but not ref-KF / frame-to-frame).
                if (!local_set.empty() && local_set.find(pMP) != local_set.end())
                {
                    mCurrentFrame.mDebugFrame2LocalMapMatches.emplace_back(curr_pt, curr_pt);
                }
            }
        }
    }

    mvKeysLastFrame = mCurrentFrame.mvKeys;
}

void Tracking::StereoInitialization()
{
    if (mCurrentFrame.N < mStereoInitMinKeypoints)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] STEREO_INIT failed: keypoints=" << mCurrentFrame.N
            << " < MinKeypoints=" << mStereoInitMinKeypoints << "." << std::endl;
        return;
    }

    if (mSensor == System::IMU_STEREO)
    {
        if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
        {
            return;
        }

        if ((mCurrentFrame.mpImuPreintegratedFrame->avgA - mLastFrame.mpImuPreintegratedFrame->avgA).norm() < 0.5)
        {
            return;
        }

        if (mpImuPreintegratedFromLastKF)
        {
            delete mpImuPreintegratedFromLastKF;
        }
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    }

    // Set Frame pose to the origin (In case of inertial SLAM to imu)
    if (mSensor == System::IMU_STEREO)
    {
        Eigen::Matrix3f Rwb0 = mCurrentFrame.mImuCalib.mTcb.rotationMatrix();
        Eigen::Vector3f twb0 = mCurrentFrame.mImuCalib.mTcb.translation();
        Eigen::Vector3f Vwb0;
        Vwb0.setZero();
        mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);
    }
    else
    {
        mCurrentFrame.SetPose(Sophus::SE3f());
    }
    // Create KeyFrame
    KeyFrame* pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    // Insert KeyFrame in the map
    mpAtlas->AddKeyFrame(pKFini);

    // Create MapPoints and asscoiate to KeyFrame
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        float z = mCurrentFrame.mvDepth[i];
        if (z > 0)
        {
            Eigen::Vector3f x3D;
            mCurrentFrame.UnprojectStereo(i, x3D);
            MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
            pNewMP->AddObservation(pKFini, i);
            pKFini->AddMapPoint(pNewMP, i);
            pNewMP->ComputeDistinctiveDescriptors();
            pNewMP->UpdateNormalAndDepth();
            mpAtlas->AddMapPoint(pNewMP);

            mCurrentFrame.mvpMapPoints[i] = pNewMP;
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);

    mLastFrame = Frame(mCurrentFrame);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFini;

    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFini;
    mCurrentFrame.mpReferenceKF = pKFini;

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());

    mState = OK;
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] STEREO_INIT ok: keypoints=" << mCurrentFrame.N
        << " map_points=" << mpAtlas->MapPointsInMap() << "." << std::endl;
}

void Tracking::MonocularInitialization()
{
    if (!mbReadyToInitializate)
    {
        // Set Reference Frame
        if (mCurrentFrame.mvKeys.size() > mMonocularInitMinKeypoints)
        {

            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
            {
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;
            }
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            if (mSensor == System::IMU_MONOCULAR)
            {
                if (mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
            }

            mbReadyToInitializate = true;

            return;
        }
    }
    else
    {
        if (((int)mCurrentFrame.mvKeys.size() <= mMonocularInitMinKeypoints) ||
            ((mSensor == System::IMU_MONOCULAR) && (mLastFrame.mTimeStamp - mInitialFrame.mTimeStamp > 1.0)))
        {
            mbReadyToInitializate = false;
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION: Not enough detected features ["
                << mCurrentFrame.mvKeys.size() << "] to initialize." << std::endl;
            return;
        }

        // Find correspondences
        ORBmatcher matcher(mMonocularInitNNRatio, true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches,
                                                       mMonocularInitSearchWindowSize);

        // Check if there are enough correspondences
        if (nmatches < mMonocularInitMinMatches)
        {
            mbReadyToInitializate = false;
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION: Not enough correspondences [" << nmatches
                << "] to initialize." << std::endl;
            return;
        }

        Sophus::SE3f Tcw;
        std::vector<bool> vbTriangulated;  // Triangulated Correspondences (mvIniMatches)

        if (mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn, mCurrentFrame.mvKeysUn, mvIniMatches, Tcw,
                                              mvIniP3D, vbTriangulated))
        {
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
            {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
                {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(Sophus::SE3f());
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);
    }

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    for (size_t i = 0; i < mvIniMatches.size(); i++)
    {
        if (mvIniMatches[i] < 0)
        {
            continue;
        }

        //Create MapPoint.
        Eigen::Vector3f worldPos;
        worldPos << mvIniP3D[i].x, mvIniP3D[i].y, mvIniP3D[i].z;
        MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpAtlas->GetCurrentMap());

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpAtlas->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if (mSensor == System::IMU_MONOCULAR)
    {
        invMedianDepth = 4.0f / medianDepth;  // 4.0f
    }
    else
    {
        invMedianDepth = 1.0f / medianDepth;
    }

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION: Median depth [" << medianDepth << "]."
        << std::endl;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50)  // TODO Check, originally 100 tracks
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION: Wrong initialization, reseting..."
            << std::endl;
        mpSystem->ResetActiveMap();
        return;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->GetPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    std::vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
        if (vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF =
            new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(), pKFcur->mImuCalib);
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs = pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    mbVelocity = false;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;

    initID = pKFcur->mnId;

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION ok: New Map created with "
        << mpAtlas->MapPointsInMap() << " points." << std::endl;
}

void Tracking::CreateMapInAtlas()
{
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
    {
        mpAtlas->SetInertialSensor();
    }
    mbSetInit = false;

    mnInitialFrameId = mCurrentFrame.mnId + 1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mbVelocity = false;
    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        mbReadyToInitializate = false;
    }

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mpImuPreintegratedFromLastKF)
    {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
    }

    if (mpLastKeyFrame)
    {
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    }

    if (mpReferenceKF)
    {
        mpReferenceKF = static_cast<KeyFrame*>(NULL);
    }

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;
}

void Tracking::CheckReplacedInLastFrame()
{
    for (int i = 0; i < mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if (pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if (pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

void Tracking::UpdateRefKeyFrame(std::vector<MapPoint*>& vpMapPointsKF)
{
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    std::vector<std::pair<float, int>> vDepthIdx;
    vDepthIdx.reserve(mpReferenceKF->N);
    for (int i = 0; i < mpReferenceKF->N; i++)
    {
        float z = mpReferenceKF->mvDepth[i];
        if (z > 0)
        {
            vDepthIdx.push_back(std::make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
    {
        return;
    }
    std::sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert 1000 points sorted by depth
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++)
    {
        int i = vDepthIdx[j].second;

        MapPoint* pMP = vpMapPointsKF[i];
        if (!pMP || pMP->Observations() < 1)
        {
            Eigen::Vector3f x3D;
            mpReferenceKF->UnprojectStereo(i, x3D);
            MapPoint* pNewMP = new MapPoint(x3D, mpReferenceKF, mpAtlas->GetCurrentMap());

            vpMapPointsKF[i] = pNewMP;
            mlpTemporalPoints.push_back(pNewMP);

            nPoints++;
        }

        if (nPoints > 1000)
        {  // 1000 for underwater dataset
            break;
        }
    }
}

bool Tracking::TrackQuadReferenceKeyFrame()
{

    // cout << "Track With Reference Keyframe...(RF)" << endl;
    mbFrame2Frame = false;
    ORBmatcher matcher(mReferenceKeyframeNNRatio, true);

    std::vector<MapPoint*> vpMapPointsKF = mpReferenceKF->GetMapPointMatches();

    UpdateRefKeyFrame(vpMapPointsKF);

    int nmatches = matcher.SearchByQuadKeyFrame(mpReferenceKF, mCurrentFrame, vpMapPointsKF,
                                                mReferenceKeyframeQuadSearchWindowSize);

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] TRACK_QUAD_REF_KF: nmatches=" << nmatches << std::endl;

    if (nmatches < mReferenceKeyframeMinBoWMatches)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] TRACK_QUAD_REF_KF failed: nmatches=" << nmatches
            << " < MinBoWMatches=" << mReferenceKeyframeMinBoWMatches << std::endl;
        return false;
    }

    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = DiscardOutliersAndCountInliers(mCurrentFrame, nmatches, true);

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] TRACK_QUAD_REF_KF: nmatchesMap=" << nmatchesMap << std::endl;

    if (nmatchesMap >= mReferenceKeyframeMinOptimizedMapMatches)
    {
        return true;
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] TRACK_QUAD_REF_KF failed: nmatchesMap=" << nmatchesMap
            << " < MinOptimizedMapMatches=" << mReferenceKeyframeMinOptimizedMapMatches << std::endl;
        return false;
    }
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    Sophus::SE3f Tlr = mlRelativeFramePoses.back();
    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        return;
    }

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    std::vector<std::pair<float, int>> vDepthIdx;
    const int Nfeat = mLastFrame.Nleft == -1 ? mLastFrame.N : mLastFrame.Nleft;
    BuildDepthIndex(mLastFrame, Nfeat, vDepthIdx);

    if (vDepthIdx.empty())
    {
        return;
    }

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if (!pMP || pMP->Observations() < 1)
        {
            bCreateNew = true;
        }

        if (bCreateNew)
        {
            Eigen::Vector3f x3D;

            mLastFrame.UnprojectStereo(i, x3D);

            MapPoint* pNewMP = new MapPoint(x3D, mpAtlas->GetCurrentMap(), &mLastFrame, i);
            mLastFrame.mvpMapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > 100)
        {
            break;
        }
    }
}

bool Tracking::TrackQuadWithMotionModel()
{
    mbFrame2Frame = true;
    ORBmatcher matcher(mMotionModelNNRatio, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    if (mpAtlas->isImuInitialized())
    {
        PredictStateIMU();
        return true;
    }

    // Save temperal matches for visualization
    mvTemporalMatches = std::vector<int>(mCurrentFrame.N, -1);

    Sophus::SE3f iniTcw = mVelocity * mLastFrame.GetPose();

    mCurrentFrame.SetPose(iniTcw);

    std::fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

    // Search matches by quad matching, i.e., matches must fit to
    // last left, right and current left, right frames simultaneously
    int nmatches = matcher.SearchByQuad(mCurrentFrame, mLastFrame, mvTemporalMatches, mMotionModelQuadSearchWindowSize);

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] TRACK_QUAD_WITH_MOTION_MODEL: nmatches=" << nmatches << std::endl;

    if (nmatches < mMotionModelMinInitialMatches)
    {
        // Retry with wider window: reset match state so retry is consistent
        std::fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
        std::fill(mvTemporalMatches.begin(), mvTemporalMatches.end(), -1);
        nmatches =
            matcher.SearchByQuad(mCurrentFrame, mLastFrame, mvTemporalMatches, mMotionModelQuadSearchWindowSizeRetry);

        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] TRACK_QUAD_WITH_MOTION_MODEL: nmatches (retry)=" << nmatches
            << std::endl;

        if (nmatches < mMotionModelMinRetryMatches)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] TRACK_QUAD_WITH_MOTION_MODEL failed: nmatches=" << nmatches
                << " < MinRetryMatches=" << mMotionModelMinRetryMatches << std::endl;
            return false;
        }
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    int nmatchesMap = DiscardOutliersAndCountInliers(mCurrentFrame, nmatches, false);

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] TRACK_QUAD_WITH_MOTION_MODEL: nmatchesMap=" << nmatchesMap << std::endl;

    if (nmatchesMap >= mMotionModelMinOptimizedMapMatches)
    {
        return true;
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] TRACK_QUAD_WITH_MOTION_MODEL failed: nmatchesMap=" << nmatchesMap
            << " < MinOptimizedMapMatches=" << mMotionModelMinOptimizedMapMatches << std::endl;
        return false;
    }
}

bool Tracking::TrackReferenceKeyFrameWithBoW()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(mReferenceKeyframeNNRatio, true);
    std::vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_WITH_BOW: nmatches=" << nmatches << std::endl;

    if (nmatches < mReferenceKeyframeMinBoWMatches)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF failed: nmatches=" << nmatches
            << " < MinBoWMatches=" << mReferenceKeyframeMinBoWMatches << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.GetPose());

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = DiscardOutliersAndCountInliers(mCurrentFrame, nmatches, true);

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_WITH_BOW: nmatchesMap=" << nmatchesMap << std::endl;

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        return true;
    }
    else
    {
        if (nmatchesMap >= mReferenceKeyframeMinOptimizedMapMatches)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_WITH_BOW ok: nmatches=" << nmatches
                << " nmatchesMap=" << nmatchesMap << std::endl;
            return true;
        }
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_WITH_BOW failed: nmatchesMap=" << nmatchesMap
            << " < MinOptimizedMapMatches=" << mReferenceKeyframeMinOptimizedMapMatches << std::endl;
        return false;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(mMotionModelNNRatio, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    if (mpAtlas->isImuInitialized())
    {
        // Predict state with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();
        return true;
    }
    else
    {
        mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());
    }

    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if (mSensor == System::STEREO)
    {
        th = mMotionModelProjectionSearchThStereo;
    }
    else
    {
        th = mMotionModelProjectionSearchThMono;
    }

    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th,
                                              mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: nmatches=" << nmatches << std::endl;

    // If few matches, uses a wider window search
    int thRetry = (mSensor == System::STEREO) ? mMotionModelRetryProjectionSearchThStereo
                                              : mMotionModelRetryProjectionSearchThMono;
    if (nmatches < mMotionModelMinInitialMatches)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: Not enough matches [" << nmatches
            << "] < MinInitialMatches=" << mMotionModelMinInitialMatches << "." << std::endl;
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, thRetry,
                                              mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR);

        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: nmatches=" << nmatches << std::endl;
    }

    if (nmatches < mMotionModelMinRetryMatches)
    {
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        {
            return true;
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: Not enough matches [" << nmatches
                << "] with wider search < MinRetryMatches=" << mMotionModelMinRetryMatches << "." << std::endl;
            return false;
        }
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = DiscardOutliersAndCountInliers(mCurrentFrame, nmatches, false);

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: nmatchesMap=" << nmatchesMap << std::endl;

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        return true;
    }
    else
    {
        if (nmatchesMap < mMotionModelMinOptimizedMapMatches)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId
                << "] TRACK_WITH_MOTION_MODEL: Not enough matches after pose optimization [" << nmatchesMap
                << "] < MinOptimizedMapMatches=" << mMotionModelMinOptimizedMapMatches << "." << std::endl;
            return false;
        }
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL ok: nmatchesMap=" << nmatchesMap << std::endl;
        return true;
    }
}

bool Tracking::TrackLocalMap()
{

    UpdateLocalMap();
    SearchLocalPoints();

    if (!mpAtlas->isImuInitialized())
    {
        Optimizer::PoseOptimization(&mCurrentFrame);
    }
    else
    {
        // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
        if (!mbMapUpdated)  //  && (mnMatchesInliers>30))
        {
            Optimizer::PoseInertialOptimizationLastFrame(
                &mCurrentFrame);  // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
        }
        else
        {
            Optimizer::PoseInertialOptimizationLastKeyFrame(
                &mCurrentFrame);  // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
        }
    }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            if (!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                {
                    mnMatchesInliers++;
                }
            }
            else if (mSensor == System::STEREO)
            {
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            }
        }
    }

    // Decide if the tracking was succesful
    // Inlier count is passed to LocalMapping for keyframe/point culling decisions.
    mpLocalMapper->mnMatchesInliers = mnMatchesInliers;

    if ((mnMatchesInliers > mLocalMapGenericMinInliers))
    {
        return true;
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        if ((mnMatchesInliers < 15 && mpAtlas->isImuInitialized()) ||
            (mnMatchesInliers < 50 && !mpAtlas->isImuInitialized()))
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else if (mSensor == System::IMU_STEREO)
    {
        if (mnMatchesInliers < 15)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        if (mnMatchesInliers < mLocalMapVisualMinInliers)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] TRACK_LOCAL_MAP failed: inliers=" << mnMatchesInliers
                << " < VisualMinInliers=" << mLocalMapVisualMinInliers << "." << std::endl;
            return false;
        }
        else
        {
            return true;
        }
    }
}

int Tracking::DiscardOutliersAndCountInliers(Frame& frame, int& nmatches, bool clearTrackInViewFlag)
{
    int nmatchesMap = 0;
    for (int i = 0; i < frame.N; i++)
    {
        if (frame.mvpMapPoints[i])
        {
            if (frame.mvbOutlier[i])
            {
                MapPoint* pMP = frame.mvpMapPoints[i];

                frame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                frame.mvbOutlier[i] = false;
                if (i < frame.Nleft)
                {
                    pMP->mbTrackInView = false;
                }
                else
                {
                    pMP->mbTrackInViewR = false;
                }
                if (clearTrackInViewFlag)
                {
                    pMP->mbTrackInView = false;
                }
                pMP->mnLastFrameSeen = frame.mnId;
                nmatches--;
            }
            else if (frame.mvpMapPoints[i]->Observations() > 0)
            {
                nmatchesMap++;
            }
        }
    }

    return nmatchesMap;
}

void Tracking::BuildDepthIndex(const Frame& frame, int N, std::vector<std::pair<float, int>>& vDepthIdx) const
{
    vDepthIdx.clear();
    vDepthIdx.reserve(N);
    for (int i = 0; i < N; i++)
    {
        float z = frame.mvDepth[i];
        if (z > 0)
        {
            vDepthIdx.push_back(std::make_pair(z, i));
        }
    }

    if (!vDepthIdx.empty())
    {
        sort(vDepthIdx.begin(), vDepthIdx.end());
    }
}

bool Tracking::NeedNewKeyFrame()
{
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) &&
        !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        const double dt = mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp;
        const bool needKF = (dt >= 0.25);
        if (needKF)
        {
            Verbose::Print(Verbose::VERBOSITY_QUIET)
                << "[" << mCurrentFrame.mnId << "] NEED_NEW_KEYFRAME: IMU not initialized, dt=" << dt << " >= 0.25."
                << std::endl;
        }
        return needKF;
    }

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    {
        return false;
    }

    const int nKFs = mpAtlas->KeyFramesInMap();

    // Tracked MapPoints in the reference keyframe
    const int nMinObs = (nKFs <= 2) ? 2 : 3;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;

    if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for (int i = 0; i < N; i++)
        {
            if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
            {
                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                {
                    nTrackedClose++;
                }
                else
                {
                    nNonTrackedClose++;
                }
            }
        }
    }

    const bool bNeedToInsertClose =
        (nTrackedClose < mNewKFMinTrackedClosePoints) && (nNonTrackedClose > mNewKFMinNonTrackedClosePoints);

    // Thresholds
    float thRefRatio;
    if (mSensor == System::IMU_MONOCULAR)
    {
        thRefRatio = (mnMatchesInliers > 350) ? 0.75f : 0.90f;  // Points tracked from the local map
    }
    else if (mSensor == System::MONOCULAR)
    {
        thRefRatio = mNewKFRefRatioMono;
    }
    else
    {
        thRefRatio = (nKFs < 2) ? mNewKFRefRatioStereoFewKFs : mNewKFRefRatioStereo;
    }

    // More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;

    // More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = ((mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames) && bLocalMappingIdle);

    // Tracking is weak
    const bool c1c = mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR &&
                     mSensor != System::IMU_STEREO &&
                     (mnMatchesInliers < nRefMatches * mNewKFWeakTrackingRatio || bNeedToInsertClose);

    // Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 =
        (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) && mnMatchesInliers > mNewKFMinInliers);

    // Temporal condition for Inertial cases
    const bool c3 = (mpLastKeyFrame) && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) &&
                    (mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp >= 0.5);

    const bool c4 = (((mnMatchesInliers < 75) && (mnMatchesInliers > 15))) && (mSensor == System::IMU_MONOCULAR);

    const bool needKeyFrame = (((c1a || c1b || c1c) && c2) || c3 || c4);
    if (!needKeyFrame)
    {
        return false;
    }

    std::string kfReason;
    if (c3)
    {
        kfReason = "IMU temporal condition (c3)";
    }
    else if (c4)
    {
        kfReason = "IMU monocular low-inlier condition (c4)";
    }
    else if (c1a)
    {
        kfReason = "too many frames since last keyframe (c1a)";
    }
    else if (c1b)
    {
        kfReason = "enough frames since last keyframe and local mapper idle (c1b)";
    }
    else if (c1c)
    {
        kfReason = "weak tracking / need more close points (c1c)";
    }
    else if (c2)
    {
        kfReason = "few tracked points compared to reference keyframe (c2)";
    }
    else
    {
        kfReason = "NeedNewKeyFrame boolean expression true";
    }

    // If the mapping accepts keyframes, insert keyframe.
    // Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle || mpLocalMapper->IsInitializing())
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] NEED_NEW_KEYFRAME: reason=" << kfReason << std::endl;
        return true;
    }

    mpLocalMapper->InterruptBA();
    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        return false;
    }
    const int queueSize = mpLocalMapper->KeyframesInQueue();
    const bool canInsert = (queueSize < mNewKFMaxKFsInQueue);
    if (canInsert)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "[" << mCurrentFrame.mnId << "] NEED_NEW_KEYFRAME: reason=" << kfReason << std::endl;
    }
    return canInsert;
}

void Tracking::CreateNewKeyFrame()
{
    if (mpLocalMapper->IsInitializing() && !mpAtlas->isImuInitialized())
    {
        return;
    }
    if (!mpLocalMapper->SetNotStop(true))
    {
        return;
    }
    KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

    if (mpAtlas->isImuInitialized())
    {
        pKF->bImu = true;
    }
    pKF->SetNewBias(mCurrentFrame.mImuBias);
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if (mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);
    }

    if (mSensor != System::MONOCULAR && mSensor != System::IMU_MONOCULAR)  // TODO check if incluide imu_stereo
    {
        mCurrentFrame.UpdatePoseMatrices();
        // We sort points by the measured depth by the stereo sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;
        std::vector<std::pair<float, int>> vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        BuildDepthIndex(mCurrentFrame, N, vDepthIdx);

        if (!vDepthIdx.empty())
        {
            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP)
                {
                    bCreateNew = true;
                }
                else if (pMP->Observations() < 1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if (bCreateNew)
                {
                    Eigen::Vector3f x3D;

                    mCurrentFrame.UnprojectStereo(i, x3D);

                    MapPoint* pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                    pNewMP->AddObservation(pKF, i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if (mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0)
                    {
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]] = pNewMP;
                        pNewMP->AddObservation(pKF, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP, mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth && nPoints > maxPoint)
                {
                    break;
                }
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << mCurrentFrame.mnId << "] TRACKING_CREATED_NEW_KEYFRAME" << std::endl;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for (std::vector<MapPoint*>::iterator vit = mCurrentFrame.mvpMapPoints.begin(),
                                          vend = mCurrentFrame.mvpMapPoints.end();
         vit != vend; vit++)
    {
        MapPoint* pMP = *vit;
        if (pMP)
        {
            if (pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (std::vector<MapPoint*>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend;
         vit++)
    {
        MapPoint* pMP = *vit;

        if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
        {
            continue;
        }
        if (pMP->isBad())
        {
            continue;
        }  // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        if (pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    if (nToMatch > 0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if (mpAtlas->isImuInitialized())
        {
            if (mpAtlas->GetCurrentMap()->GetIniertialBA2())
            {
                th = 2;
            }
            else
            {
                th = 6;
            }
        }
        else if (!mpAtlas->isImuInitialized() && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))
        {
            th = 10;
        }

        if (mState == LOST)
        {
            th = 15;  // 15
        }
        matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints,
                                   mpLocalMapper->mThFarPoints);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for (std::vector<KeyFrame*>::const_reverse_iterator itKF = mvpLocalKeyFrames.rbegin(),
                                                        itEndKF = mvpLocalKeyFrames.rend();
         itKF != itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const std::vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for (std::vector<MapPoint*>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP;
             itMP++)
        {

            MapPoint* pMP = *itMP;
            if (!pMP)
            {
                continue;
            }
            if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
            {
                continue;
            }
            if (!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    std::map<KeyFrame*, int> keyframeCounter;
    if (!mpAtlas->isImuInitialized())
    {
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();
                    for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator it = observations.begin(),
                                                                                   itend = observations.end();
                         it != itend; it++)
                    {
                        keyframeCounter[it->first]++;
                    }
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if (mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if (!pMP)
                {
                    continue;
                }
                if (!pMP->isBad())
                {
                    const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();
                    for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator it = observations.begin(),
                                                                                   itend = observations.end();
                         it != itend; it++)
                    {
                        keyframeCounter[it->first]++;
                    }
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i] = NULL;
                }
            }
        }
    }

    int max = 0;
    KeyFrame* pKFmax = static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for (std::map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end();
         it != itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if (pKF->isBad())
        {
            continue;
        }
        if (it->second > max)
        {
            max = it->second;
            pKFmax = pKF;
        }

        mvpLocalKeyFrames.push_back(pKF);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for (std::vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if (mvpLocalKeyFrames.size() > 80)  // 80
        {
            break;
        }
        KeyFrame* pKF = *itKF;

        const std::vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for (std::vector<KeyFrame*>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end();
             itNeighKF != itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad())
            {
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const std::set<KeyFrame*> spChilds = pKF->GetChilds();
        for (std::set<KeyFrame*>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if (!pChildKF->isBad())
            {
                if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if (pParent)
        {
            if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mvpLocalKeyFrames.size() < 80)
    {
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for (int i = 0; i < Nd; i++)
        {
            if (!tempKeyFrame)
            {
                break;
            }
            if (tempKeyFrame->mnTrackReferenceForFrame != mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                tempKeyFrame = tempKeyFrame->mPrevKF;
            }
        }
    }

    if (pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

void Tracking::Reset(bool bLocMap)
{
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_DEBUG);
    if (mpViewer)
    {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
        {
            usleep(3000);
        }
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        mpLocalMapper->RequestReset();
    }

    // Reset Loop Closing
    mpLoopClosing->RequestReset();

    // Clear BoW Database
    mpKeyFrameDB->clear();

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor == System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
    {
        mpAtlas->SetInertialSensor();
    }
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit = false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if (mpViewer)
    {
        mpViewer->Release();
    }
}

void Tracking::ResetActiveMap(bool bLocMap)
{
    if (mpViewer)
    {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
        {
            usleep(3000);
        }
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap)
    {
        mpLocalMapper->RequestResetActiveMap(pMap);
    }

    // Reset Loop Closing
    mpLoopClosing->RequestResetActiveMap(pMap);

    // Clear BoW Database
    mpKeyFrameDB->clearMap(pMap);  // Only clear the active map references

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();

    mnLastInitFrameId = Frame::nNextId;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;

    std::list<bool> lbLost;
    unsigned int index = mnFirstFrameId;
    for (Map* pMap : mpAtlas->GetAllMaps())
    {
        if (pMap->GetAllKeyFrames().size() > 0)
        {
            if (index > pMap->GetLowerKFID())
            {
                index = pMap->GetLowerKFID();
            }
        }
    }

    int num_lost = 0;

    for (std::list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
    {
        if (index < mnInitialFrameId)
        {
            lbLost.push_back(*ilbL);
        }
        else
        {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    mbVelocity = false;

    if (mpViewer)
    {
        mpViewer->Release();
    }
}

std::vector<MapPoint*> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

bool Tracking::isLastFrameKeyframe()
{
    return mnLastKeyFrameId == mLastFrame.mnId;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias& b, KeyFrame* pCurrentKeyFrame)
{
    Map* pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    std::list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
    std::list<bool>::iterator lbL = mlbLost.begin();
    for (auto lit = mlRelativeFramePoses.begin(), lend = mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lbL++)
    {
        if (*lbL)
        {
            continue;
        }
        KeyFrame* pKF = *lRit;

        while (pKF->isBad())
        {
            pKF = pKF->GetParent();
        }

        if (pKF->GetMap() == pMap)
        {
            (*lit).translation() *= s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    while (!mCurrentFrame.imuIsPreintegrated())
    {
        usleep(500);
    }

    if (mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(
            IMU::NormalizeRotation(Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 + Rwb1 * mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated)
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        const Eigen::Matrix3f Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        const Eigen::Vector3f Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        float t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(
            IMU::NormalizeRotation(Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 + Rwb1 * mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}

MonocularDebugFrame Tracking::GetMonocularDebugFrame() const
{
    std::unique_lock<std::mutex> lock(mMutexMonocularDebugFrame);
    return mLastMonocularDebugFrame;
}

MonocularDebugFrame Tracking::BuildMonocularDebugFrame(const Frame& frame, const cv::Mat& image) const
{
    MonocularDebugFrame debugFrame;
    debugFrame.image = image.clone();
    debugFrame.keypoints_detected = frame.mvKeys;
    const size_t n = frame.mvKeys.size();
    if (frame.mvpMapPoints.size() != n || frame.mvbOutlier.size() != n)
    {
        return debugFrame;
    }
    debugFrame.keypoints_inlier.reserve(n);
    debugFrame.keypoints_outlier.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        if (frame.mvpMapPoints[i])
        {
            if (frame.mvbOutlier[i])
            {
                debugFrame.keypoints_outlier.push_back(frame.mvKeys[i]);
            }
            else
            {
                debugFrame.keypoints_inlier.push_back(frame.mvKeys[i]);
            }
        }
    }
    // Copy precomputed debug correspondences from the frame.
    debugFrame.frame_to_frame_matches = frame.mDebugFrame2FrameMatches;
    debugFrame.frame_to_ref_kf_matches = frame.mDebugFrame2RefKfMatches;
    debugFrame.frame_to_local_map_matches = frame.mDebugFrame2LocalMapMatches;

    return debugFrame;
}

void Tracking::UpdateMonocularDebugFrame(const cv::Mat& image)
{
    MonocularDebugFrame debugFrame = BuildMonocularDebugFrame(mCurrentFrame, image);
    std::unique_lock<std::mutex> lock(mMutexMonocularDebugFrame);
    mLastMonocularDebugFrame = std::move(debugFrame);
}

StereoDebugFrame Tracking::GetStereoDebugFrame() const
{
    std::unique_lock<std::mutex> lock(mMutexStereoDebugFrame);
    return mLastStereoDebugFrame;
}

StereoDebugFrame Tracking::BuildStereoDebugFrameMetashapePinhole(const Frame& frame, const cv::Mat& leftRectified,
                                                                 const cv::Mat& rightRectified, const Frame* pLastFrame,
                                                                 const cv::Mat* pLastLeftRectified,
                                                                 const cv::Mat* pLastRightRectified) const
{
    StereoDebugFrame debugFrame;
    debugFrame.mode = StereoDebugMode::METASHAPE_PINHOLE;
    debugFrame.left_rectified = leftRectified.clone();
    debugFrame.right_rectified = rightRectified.clone();
    debugFrame.left_keypoints = frame.mvKeys;
    debugFrame.right_keypoints = frame.mvKeysRight;

    const size_t n = std::min(frame.mvKeys.size(), frame.mvuRight.size());
    debugFrame.matches.reserve(n);
    debugFrame.match_lines.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        const float uRight = frame.mvuRight[i];
        if (uRight < 0.0f)
        {
            continue;
        }

        StereoMatchDebug match;
        match.left_idx = static_cast<int>(i);
        match.right_idx = -1;
        match.left_point = frame.mvKeys[i].pt;
        match.right_point = cv::Point2f(uRight, frame.mvKeys[i].pt.y);
        match.disparity = match.left_point.x - match.right_point.x;
        if (i < frame.mvDepth.size() && frame.mvDepth[i] > 0.0f)
        {
            match.depth = frame.mvDepth[i];
            match.has_depth = true;
        }
        debugFrame.matches.push_back(match);
        debugFrame.match_lines.emplace_back(match.left_point.x, match.left_point.y, match.right_point.x,
                                            match.right_point.y);
    }

    if (pLastLeftRectified && !pLastLeftRectified->empty())
    {
        debugFrame.last_left_rectified = pLastLeftRectified->clone();
    }
    if (pLastRightRectified && !pLastRightRectified->empty())
    {
        debugFrame.last_right_rectified = pLastRightRectified->clone();
    }

    if (pLastFrame && !frame.mvpMapPoints.empty())
    {
        debugFrame.frame_to_frame_matches.reserve(frame.mvpMapPoints.size());
        for (size_t i = 0; i < frame.mvpMapPoints.size(); ++i)
        {
            MapPoint* pMP = frame.mvpMapPoints[i];
            if (!pMP)
            {
                continue;
            }
            for (size_t j = 0; j < pLastFrame->mvpMapPoints.size(); ++j)
            {
                if (pLastFrame->mvpMapPoints[j] == pMP)
                {
                    cv::Point2f lastPt = pLastFrame->mvKeys[j].pt;
                    cv::Point2f currPt = frame.mvKeys[i].pt;
                    debugFrame.frame_to_frame_matches.push_back(std::make_pair(lastPt, currPt));
                    break;
                }
            }
        }
    }

    if (pLastFrame)
    {
        debugFrame.last_left_keypoints = pLastFrame->mvKeys;
        debugFrame.last_right_keypoints = pLastFrame->mvKeysRight;
    }

    return debugFrame;
}

void Tracking::UpdateStereoDebugFrame(const cv::Mat& leftRectified, const cv::Mat& rightRectified)
{
    const cv::Mat* pLastLeft = nullptr;
    const cv::Mat* pLastRight = nullptr;
    {
        std::unique_lock<std::mutex> lock(mMutexStereoDebugFrame);
        if (!mLastStereoDebugFrame.left_rectified.empty())
        {
            pLastLeft = &mLastStereoDebugFrame.left_rectified;
        }
        if (!mLastStereoDebugFrame.right_rectified.empty())
        {
            pLastRight = &mLastStereoDebugFrame.right_rectified;
        }
    }

    const Frame* pLastFrame = (mLastFrame.isSet() && mLastFrame.N > 0) ? &mLastFrame : nullptr;
    StereoDebugFrame debugFrame = BuildStereoDebugFrameMetashapePinhole(mCurrentFrame, leftRectified, rightRectified,
                                                                        pLastFrame, pLastLeft, pLastRight);

    std::unique_lock<std::mutex> lock(mMutexStereoDebugFrame);
    mLastStereoDebugFrame = std::move(debugFrame);
}

float Tracking::GetImageScale()
{
    return mImageScale;
}
}  // namespace ORB_SLAM3
