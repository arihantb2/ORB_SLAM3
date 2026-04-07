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
#include "FeatureMatcher.h"
#include "GeometricTools.h"
#include "LocalMapping.h"
#include "Optimizer.h"
#include "Settings.h"
#include "System.h"
#include "feature_extractor/GridBasedORBFeatureExtractor.h"
#include "feature_extractor/SIFTFeatureExtractor.h"
#include "feature_extractor/VanillaORBFeatureExtractor.h"

#include <algorithm>
#include <mutex>
#include <optional>
#include <utility>

namespace ORB_SLAM3
{

Tracking::Tracking(System* pSys, IBowVocabulary* pVoc, Atlas* pAtlas, const std::string& strSettingPath,
                   const int sensor, Settings* settings, const bool newMaps)
    : mState(NO_IMAGES_YET),
      mSensor(sensor),
      mbMapUpdated(false),
      mpORBVocabulary(pVoc),
      mbReadyToInitializate(false),
      mpSystem(pSys),
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
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
    for (GeometricCamera* pCam : vpCams)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Camera " << pCam->GetId();
        if (pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << " is pinhole" << std::endl;
        }
        else if (pCam->GetType() == GeometricCamera::CAM_METASHAPE)
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << " is metashape" << std::endl;
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << " is unknown" << std::endl;
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

    mK = mpCamera->toK();
    mK_ = mpCamera->toK_();

    if (mSensor == System::STEREO)
    {
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    // Feature extractor instantiation — type selected by FeatureExtractor.type in config
    const int nFeatures = settings->nFeatures();
    const int nInitFeatures = settings->nInitFeatures();
    const int nLevels = settings->nLevels();

    const std::string extractorType = settings->featureExtractorType();

    if (extractorType == "SIFT")
    {
        const int nOctaveLayers = settings->siftNOctaveLayers();
        const double contrastTh = settings->siftContrastThreshold();
        const double edgeTh = settings->siftEdgeThreshold();
        const double sigma = settings->siftSigma();

        mpFeatureExtractorLeft = new SIFTFeatureExtractor(nFeatures, nOctaveLayers, contrastTh, edgeTh, sigma, nLevels);
        if (mSensor == System::STEREO)
        {
            mpFeatureextractorRight =
                new SIFTFeatureExtractor(nFeatures, nOctaveLayers, contrastTh, edgeTh, sigma, nLevels);
        }
        if (mSensor == System::MONOCULAR)
        {
            mpIniFeatureExtractor =
                new SIFTFeatureExtractor(nInitFeatures, nOctaveLayers, contrastTh, edgeTh, sigma, nLevels);
        }
    }
    else if (extractorType == "ORB")
    {
        const float fScaleFactor = settings->scaleFactor();
        const int fastThreshold = settings->initThFAST();
        const auto scoreType = static_cast<cv::ORB::ScoreType>(settings->orbScoreType());

        mpFeatureExtractorLeft =
            new VanillaORBFeatureExtractor(nFeatures, fScaleFactor, nLevels, fastThreshold, scoreType);
        if (mSensor == System::STEREO)
        {
            mpFeatureextractorRight =
                new VanillaORBFeatureExtractor(nFeatures, fScaleFactor, nLevels, fastThreshold, scoreType);
        }
        if (mSensor == System::MONOCULAR)
        {
            mpIniFeatureExtractor =
                new VanillaORBFeatureExtractor(nInitFeatures, fScaleFactor, nLevels, fastThreshold, scoreType);
        }
    }
    else  // "GridORB" (default)
    {
        const float fScaleFactor = settings->scaleFactor();
        const int fIniThFAST = settings->initThFAST();
        const int fMinThFAST = settings->minThFAST();

        mpFeatureExtractorLeft =
            new GridBasedORBFeatureExtractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        if (mSensor == System::STEREO)
        {
            mpFeatureextractorRight =
                new GridBasedORBFeatureExtractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        }
        if (mSensor == System::MONOCULAR)
        {
            mpIniFeatureExtractor =
                new GridBasedORBFeatureExtractor(nInitFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        }
    }

    // Monocular initialization thresholds
    mMonocularInitSearchWindowSize = settings->monocularInitSearchWindowSize();
    mMonocularInitMinKeypoints = settings->monocularInitMinKeypoints();
    mMonocularInitNNRatio = settings->monocularInitNNRatio();
    mMonocularInitMinMatches = settings->monocularInitMinMatches();

    // Stereo initialization thresholds
    mStereoInitMinKeypoints = settings->stereoInitMinKeypoints();
    mStereoInitMinMapPoints = settings->stereoInitMinMapPoints();

    // Reference keyframe tracking thresholds
    mReferenceKeyframeNNRatio = settings->referenceKeyframeNNRatio();
    mReferenceKeyframeMinBoWMatches = settings->referenceKeyframeMinBoWMatches();
    mReferenceKeyframeMinOptimizedMapMatches = settings->referenceKeyframeMinOptimizedMapMatches();
    // SIFT mode: disable BoW-based reference keyframe tracking.
    mUseBoWReferenceKeyframeTracking = (extractorType != "SIFT");

    // Motion model tracking thresholds
    mMotionModelNNRatio = settings->motionModelNNRatio();
    mMotionModelProjectionSearchTh = settings->motionModelProjectionSearchTh();
    mMotionModelMinInitialMatches = settings->motionModelMinInitialMatches();
    mMotionModelRetryProjectionSearchTh = settings->motionModelRetryProjectionSearchTh();
    mMotionModelMinRetryMatches = settings->motionModelMinRetryMatches();
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
}

void Tracking::SetLocalMapper(LocalMapping* pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

TrackingResult Tracking::GrabImageStereo(const cv::Mat& imageLeft, const cv::Mat& imageRight, const double& timestamp,
                                         const std::optional<Sophus::SE3f>& posePrior)
{
    if (imageLeft.channels() != 1)
    {
        throw std::runtime_error("[Tracking::GrabImageStereo]: Input image must be grayscale");
    }
    if (imageRight.channels() != 1)
    {
        throw std::runtime_error("[Tracking::GrabImageStereo]: Input image must be grayscale");
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "[-] TRACKING_STEREO" << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    // Create the current frame, extracting features and computing stereo matches.
    mCurrentFrame = Frame(imageLeft, imageRight, timestamp, mpFeatureExtractorLeft, mpFeatureextractorRight,
                          mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, mpCamera);

    if (posePrior.has_value())
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "GRAB_IMAGE_STEREO: TwcPrior" << mCurrentFrame.mnId << ": "
            << posePrior.value().translation().transpose() << std::endl;
        mCurrentFrame.setPosePrior(posePrior.value());
    }

    TrackingResult result = Track();

    // Attach the rectified images so the result is self-contained.
    // .clone() is mandatory: imageLeft/imageRight are const refs to temporaries
    // in System.cc that go out of scope immediately after this function returns.
    result.image_left = imageLeft.clone();
    result.image_right = imageRight.clone();

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    return result;
}

TrackingResult Tracking::GrabImageMonocular(const cv::Mat& image, const double& timestamp,
                                            const std::optional<Sophus::SE3f>& posePrior)
{
    if (image.channels() != 1)
    {
        throw std::runtime_error("[Tracking::GrabImageMonocular]: Input image must be grayscale");
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "[-] TRACKING_MONOCULAR" << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    // Create the current frame, extracting features.
    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET || (lastID - initID) < mMaxFrames)
    {
        mCurrentFrame =
            Frame(image, timestamp, mpIniFeatureExtractor, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
    }
    else
    {
        mCurrentFrame =
            Frame(image, timestamp, mpFeatureExtractorLeft, mpORBVocabulary, mpCamera, mDistCoef, mbf, mThDepth);
    }

    if (posePrior.has_value())
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "GRAB_IMAGE_MONOCULAR: TwcPrior" << mCurrentFrame.mnId << ": "
            << posePrior.value().translation().transpose() << std::endl;
        mCurrentFrame.setPosePrior(posePrior.value());
    }

    lastID = mCurrentFrame.mnId;
    TrackingResult result = Track();

    // Attach the (undistorted) image so the result is self-contained.
    // image_right is left default-constructed (empty) for monocular.
    result.image_left = image.clone();

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "----------------------------------------------------------------------------------------------------"
        << std::endl;

    return result;
}

void Tracking::PrepareFrameForTracking()
{
    if (mState == NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;

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
    if (mSensor == System::STEREO)
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

    mnFirstFrameId = mCurrentFrame.mnId;

    return true;
}

void Tracking::UpdateAfterTracking(bool tracking_success)
{
    if (!tracking_success)
    {
        return;
    }

    // Update motion model
    if (mLastFrame.isSet() && mCurrentFrame.isSet())
    {
        const Sophus::SE3f& T_cLastw = mLastFrame.GetPose();
        const Sophus::SE3f& T_cCurrw = mCurrentFrame.GetPose();

        // takes points in cLast frame and moves them to the cCurr frame
        const Sophus::SE3f& T_cCurrcLast = T_cCurrw * T_cLastw.inverse();

        // Relative motion (last camera -> current camera) in Tcw convention
        mVelocity = T_cCurrcLast;

        // Delta position in world frame
        const Eigen::Vector3f& p_wcLast = T_cLastw.inverse().translation();
        const Eigen::Vector3f& p_wcCurr = T_cCurrw.inverse().translation();

        // motion from frame cLast to frame cCurr as seen in the world frame
        const Eigen::Vector3f& p_cLastcCurr_w = p_wcCurr - p_wcLast;

        // motion from frane cLast to frame cCurr as seen in the cLast frame
        const auto R_cLastw = T_cLastw.rotationMatrix();
        const Eigen::Vector3f& p_cLastcCurr_cLast = R_cLastw * p_cLastcCurr_w;
        const Eigen::Vector3f& p_cLastcCurr_cLast_norm = p_cLastcCurr_cLast.normalized();

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "UPDATE_AFTER_TRACKING: p_c" << mLastFrame.mnId << "c"
            << mCurrentFrame.mnId << "_w: " << p_cLastcCurr_w.transpose() << " m" << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "UPDATE_AFTER_TRACKING: p_c" << mLastFrame.mnId << "c"
            << mCurrentFrame.mnId << "_c" << mLastFrame.mnId << ": " << p_cLastcCurr_cLast.transpose() << " m"
            << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "UPDATE_AFTER_TRACKING: p_c" << mLastFrame.mnId << "c"
            << mCurrentFrame.mnId << "_c" << mLastFrame.mnId << "_norm: " << p_cLastcCurr_cLast_norm.transpose() << " m"
            << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "UPDATE_AFTER_TRACKING: Motion ||p||_c" << mLastFrame.mnId << "c"
            << mCurrentFrame.mnId << ": " << p_cLastcCurr_w.norm() << " m" << std::endl;

        mbVelocity = true;
    }
    else
    {
        mbVelocity = false;
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
}

void Tracking::ComputeVelocityFromPriors()
{
    auto compute_velocity = [](const Sophus::SE3f& T_wcCurr, const Sophus::SE3f& T_wcLast) -> Sophus::SE3f
    {
        const auto& T_cCurrw = T_wcCurr.inverse();
        const auto& T_cCurrcLast = T_cCurrw * T_wcLast;

        return T_cCurrcLast;
    };

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] COMPUTE_VELOCITY_FROM_PRIORS: mCurrentFrame.hasPosePrior: " << std::boolalpha
        << mCurrentFrame.hasPosePrior() << std::endl;

    if (mState == NOT_INITIALIZED)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId
            << "] COMPUTE_VELOCITY_FROM_PRIORS: mInitialFrame.hasPosePrior: " << std::boolalpha
            << mInitialFrame.hasPosePrior() << std::endl;
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId
            << "] COMPUTE_VELOCITY_FROM_PRIORS: mLastFrame.hasPosePrior: " << std::boolalpha
            << mLastFrame.hasPosePrior() << std::endl;
    }

    if (!mCurrentFrame.hasPosePrior())
    {
        return;
    }

    if (mState == OK && mLastFrame.hasPosePrior())
    {
        mVelocity = compute_velocity(*mCurrentFrame.mPosePrior, *mLastFrame.mPosePrior);
        mbVelocity = true;

        const Eigen::Vector3f& p_cLastPriorcCurrPrior_w = mVelocity.inverse().translation();
        const Eigen::Vector3f& p_cLastPriorcCurrPrior_cLast =
            mLastFrame.mPosePrior->inverse().rotationMatrix() * p_cLastPriorcCurrPrior_w;

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "COMPUTE_VELOCITY_FROM_PRIORS: p_c" << mLastFrame.mnId << "Priorc"
            << mCurrentFrame.mnId << "Prior_w: " << p_cLastPriorcCurrPrior_w.transpose() << " m" << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "COMPUTE_VELOCITY_FROM_PRIORS: p_c" << mLastFrame.mnId << "Priorc"
            << mCurrentFrame.mnId << "Prior_c" << mLastFrame.mnId << ": " << p_cLastPriorcCurrPrior_cLast.transpose()
            << " m" << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "COMPUTE_VELOCITY_FROM_PRIORS: Motion ||p||_c" << mLastFrame.mnId
            << "Priorc" << mCurrentFrame.mnId << "Prior: " << p_cLastPriorcCurrPrior_w.norm() << " m" << std::endl;
    }
    else if (mInitialFrame.hasPosePrior())
    {
        mVelocity = compute_velocity(*mCurrentFrame.mPosePrior, *mInitialFrame.mPosePrior);
        mbVelocity = true;

        const Eigen::Vector3f& p_cInitialPriorcCurrPrior_w = mVelocity.inverse().translation();
        const Eigen::Vector3f& p_cInitialPriorcCurrPrior_cInitial =
            mInitialFrame.mPosePrior->inverse().rotationMatrix() * p_cInitialPriorcCurrPrior_w;

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "COMPUTE_VELOCITY_FROM_PRIORS: p_c" << mInitialFrame.mnId
            << "Priorc" << mCurrentFrame.mnId << "Prior_w: " << p_cInitialPriorcCurrPrior_w.transpose() << " m"
            << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "COMPUTE_VELOCITY_FROM_PRIORS: p_c" << mInitialFrame.mnId
            << "Priorc" << mCurrentFrame.mnId << "Prior_c" << mInitialFrame.mnId << ": "
            << p_cInitialPriorcCurrPrior_cInitial.transpose() << " m" << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "COMPUTE_VELOCITY_FROM_PRIORS: Motion ||p||_c" << mInitialFrame.mnId
            << "Priorc" << mCurrentFrame.mnId << "Prior: " << p_cInitialPriorcCurrPrior_w.norm() << " m" << std::endl;
    }
}

TrackingResult Tracking::Track()
{
    // In synchronous mode, wait for LocalMapping to finish processing the last inserted
    // KeyFrame before touching any map state.  Feature detection (Frame constructor) runs
    // before this call, so it overlaps freely with LocalMapping's iteration.
    mpLocalMapper->WaitForMappingComplete();

    Map* pCurrentMap = mpAtlas->GetCurrentMap();
    if (!pCurrentMap)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "ERROR: There is not an active map in the atlas. Creating a new map..." << std::endl;
        CreateMapInAtlas();
        pCurrentMap = mpAtlas->GetCurrentMap();
    }

    PrepareFrameForTracking();

    // Get Map Mutex -> Map cannot be changed
    std::unique_lock<std::mutex> lock(pCurrentMap->mMutexMapUpdate);

    UpdateMapChangeState(pCurrentMap);
    ComputeVelocityFromPriors();

    TrackingResult tracking_result;

    // Populate keypoint_data from mCurrentFrame immediately after frame construction.
    // mvuRight is initialised to -1 for every keypoint in mono (Frame constructor),
    // so this block is safe to run unconditionally for all sensor types.
    tracking_result.keypoint_data.left_keypoints = mCurrentFrame.mvKeysUn;
    tracking_result.keypoint_data.right_keypoints = mCurrentFrame.mvKeysRight;
    tracking_result.keypoint_data.stereo_right_u = mCurrentFrame.mvuRight;
    tracking_result.keypoint_data.stereo_depth = mCurrentFrame.mvDepth;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvuRight[i] >= 0.f)
        {
            cv::KeyPoint right_kp = mCurrentFrame.mvKeysUn[i];
            right_kp.pt.x = mCurrentFrame.mvuRight[i];
            tracking_result.keypoint_data.stereo_match_pairs.push_back({i, right_kp});
        }
    }

    if (mState == NOT_INITIALIZED)
    {
        Initialize();
    }
    else if (mState == OK)
    {
        // System is initialized. Track Frame.
        // Local Mapping might have changed some MapPoints tracked in last frame
        CheckReplacedInLastFrame();

        Map* pCurrentMap = mpAtlas->GetCurrentMap();
        auto trackReferenceKF = [&]() -> RefKeyFrameTrackingResult
        {
            if (mUseBoWReferenceKeyframeTracking)
            {
                mCurrentFrame.ComputeBoW();
                if (mCurrentFrame.mFeatVec.empty() || !mpReferenceKF || mpReferenceKF->mFeatVec.empty())
                {
                    return TrackReferenceKeyFrameNoBoW();
                }
            }
            return mUseBoWReferenceKeyframeTracking ? TrackReferenceKeyFrameWithBoW() : TrackReferenceKeyFrameNoBoW();
        };
        if (!mbVelocity)
        {
            tracking_result.ref_key_frame_result = trackReferenceKF();
            tracking_result.ref_keyframe_tracking_primary = true;
            if (!tracking_result.ref_key_frame_result.success)
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF failed." << std::endl;
            }
        }
        else
        {
            tracking_result.motion_model_result = TrackWithMotionModel();
            tracking_result.motion_model_tracking_primary = true;
            if (!tracking_result.motion_model_result.success)
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL failed." << std::endl;
                tracking_result.ref_key_frame_result = trackReferenceKF();
                tracking_result.ref_keyframe_tracking_fallback = true;
                if (!tracking_result.ref_key_frame_result.success)
                {
                    Verbose::Print(Verbose::VERBOSITY_DEBUG)
                        << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF failed (fallback)." << std::endl;
                }
            }
        }

        auto frame_tracking_success =
            tracking_result.ref_key_frame_result.success || tracking_result.motion_model_result.success;

        if (!frame_tracking_success)
        {
            mState = LOST;
            mTimeStampLost = mCurrentFrame.mTimeStamp;
            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "[" << mCurrentFrame.mnId << "] TRACK_LOST. Pose estimation failed" << std::endl;
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "[" << mCurrentFrame.mnId << "] TRACK_OK. Pose estimation succeeded" << std::endl;
        }

        if (frame_tracking_success)
        {
            // If we have an initial estimation of the camera pose and matching. Track the local map.
            tracking_result.local_map_result = TrackLocalMap();
            tracking_result.success = tracking_result.local_map_result.success;

            if (!tracking_result.local_map_result.success)
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << mCurrentFrame.mnId << "] TRACK_LOCAL_MAP failed." << std::endl;
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << mCurrentFrame.mnId << "] TRACK_LOCAL_MAP ok: inliers=" << mnMatchesInliers << std::endl;
            }
        }
        else
        {
            tracking_result.success = false;
        }

        if (!tracking_result.success)
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "[" << mCurrentFrame.mnId
                << "] Tracking LOST (frames_since_last_kf=" << (mCurrentFrame.mnId - mnLastKeyFrameId) << ")."
                << std::endl;

            mState = LOST;
            mTimeStampLost = mCurrentFrame.mTimeStamp;
        }
        else
        {
            mState = OK;
        }

        // Update tracking result pose
        tracking_result.pose = mCurrentFrame.GetPose().inverse();

        // Set reference keyframe for current frame
        if (!mCurrentFrame.mpReferenceKF)
        {
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }

        UpdateAfterTracking(tracking_result.success);

        if (tracking_result.success && NeedNewKeyFrame())
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

        if (mCurrentFrame.isSet())
        {
            // Populate all_tracked_map_points: every inlier map point visible in this frame
            // after the complete tracking pipeline.
            const Sophus::SE3f Tcw = mCurrentFrame.GetPose();
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                if (!mCurrentFrame.mvpMapPoints[i] || mCurrentFrame.mvbOutlier[i])
                {
                    continue;
                }

                MapPointObservation obs;
                obs.keypoint_idx = i;
                obs.keypoint = mCurrentFrame.mvKeysUn[i];
                obs.pos_world = mCurrentFrame.mvpMapPoints[i]->GetWorldPos();
                obs.pos_camera = Tcw * obs.pos_world;
                obs.map_point_id = mCurrentFrame.mvpMapPoints[i]->mnId;
                obs.is_inlier = true;
                tracking_result.all_tracked_map_points.push_back(obs);
            }
        }

        if (mSensor == System::STEREO && mCurrentFrame.isSet())
        {
            // Populate new_map_point_candidates: stereo keypoints with valid depth that
            // are NOT currently tracked as map points. LocalMapping will create new
            // MapPoints from these when this frame becomes a KeyFrame.
            const Sophus::SE3f Tcw = mCurrentFrame.GetPose();
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                if (mCurrentFrame.mvDepth[i] <= 0.f)
                {
                    continue;
                }
                if (mCurrentFrame.mvpMapPoints[i])
                {
                    continue;
                }

                NewMapPointCandidate c;
                c.keypoint_idx = i;
                c.left_kp = mCurrentFrame.mvKeysUn[i];
                c.depth = mCurrentFrame.mvDepth[i];
                c.right_kp = mCurrentFrame.mvKeysUn[i];
                c.right_kp.pt.x = mCurrentFrame.mvuRight[i];

                Eigen::Vector3f x3D;
                if (mCurrentFrame.UnprojectStereo(i, x3D))
                {
                    c.pos_world = x3D;
                    c.pos_camera = Tcw * x3D;
                }

                tracking_result.new_map_point_candidates.push_back(c);
            }
        }

        // Reset if tracking failed
        if (!tracking_result.success)
        {
            mpSystem->ResetActiveMap();
            return tracking_result;  // early return if tracking failed
        }

        mLastFrame = Frame(mCurrentFrame);
    }
    else if (mState == LOST)
    {
        mpSystem->ResetActiveMap();
        if (mpLastKeyFrame)
        {
            mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
        }
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] Tracking state is not valid: " << mState << std::endl;

        throw std::runtime_error("Tracking state is not valid: " + std::to_string(mState));
    }

    return tracking_result;
}

void Tracking::StereoInitialization()
{
    if (mCurrentFrame.N < mStereoInitMinKeypoints)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] STEREO_INIT failed: keypoints=" << mCurrentFrame.N
            << " < MinKeypoints=" << mStereoInitMinKeypoints << "." << std::endl;
        return;
    }

    // Set Frame pose to the origin
    mCurrentFrame.SetPose(Sophus::SE3f());
    // Create KeyFrame
    KeyFrame* pKFini = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap());

    // Insert KeyFrame in the map
    mpAtlas->AddKeyFrame(pKFini);

    std::vector<std::pair<int, MapPoint*>> new_map_points;
    // Create MapPoints and asscoiate to KeyFrame
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        float z = mCurrentFrame.mvDepth[i];
        if (z > 0)
        {
            Eigen::Vector3f x3D;
            mCurrentFrame.UnprojectStereo(i, x3D);
            MapPoint* pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
            new_map_points.push_back({i, pNewMP});
        }
    }

    if (new_map_points.size() < mStereoInitMinMapPoints)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] STEREO_INIT failed: map_points=" << new_map_points.size()
            << " < MinMapPoints=" << mStereoInitMinMapPoints << "." << std::endl;
        return;
    }

    for (size_t i = 0; i < new_map_points.size(); i++)
    {
        int idx = new_map_points[i].first;
        MapPoint* pMP = new_map_points[i].second;
        pMP->AddObservation(pKFini, idx);
        pKFini->AddMapPoint(pMP, idx);
        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();
        mpAtlas->AddMapPoint(pMP);

        mCurrentFrame.mvpMapPoints[idx] = pMP;
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

    mState = OK;

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
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

            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "[" << mCurrentFrame.mnId
                << "] MONOCULAR_INITIALIZATION: Set mInitialFrame to id: " << mInitialFrame.mnId << std::endl;

            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
            {
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;
            }
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            mbReadyToInitializate = true;

            return;
        }
    }
    else if (mCurrentFrame.mvKeys.size() <= mMonocularInitMinKeypoints)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION: Not enough detected features ["
            << mCurrentFrame.mvKeys.size() << "] to initialize. Dropping this frame." << std::endl;
        return;
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION: mInitialFrame[" << mInitialFrame.mnId
            << "].hasPosePrior: " << std::boolalpha << mInitialFrame.hasPosePrior() << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION: mCurrentFrame[" << mCurrentFrame.mnId
            << "].hasPosePrior: " << std::boolalpha << mCurrentFrame.hasPosePrior() << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION: mbVelocity: " << std::boolalpha << mbVelocity
            << std::endl;

        // Find correspondences
        const DescriptorType descriptorType =
            (mInitialFrame.mDescriptors.type() == CV_32FC1) ? DescriptorType::FLOAT32 : DescriptorType::BINARY;
        FeatureMatcher matcher(mMonocularInitNNRatio, true, descriptorType);
        int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches,
                                                       mMonocularInitSearchWindowSize);

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] " << "MONOCULAR_INITIALIZATION: nmatches: " << nmatches << std::endl;

        // Check if there are enough correspondences
        if (nmatches < mMonocularInitMinMatches)
        {
            mbReadyToInitializate = false;
            Verbose::Print(Verbose::VERBOSITY_DEBUG)
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

            const auto& Twc = Tcw.inverse();

            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "[" << mCurrentFrame.mnId << "] "
                << "MONOCULAR_INITIALIZATION: Twc: " << Twc.translation().transpose() << std::endl;

            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "[" << mCurrentFrame.mnId << "] "
                << "MONOCULAR_INITIALIZATION: Motion ||p||_cInitialcCurr: " << Twc.translation().norm() << " m"
                << std::endl;

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpAtlas->GetCurrentMap());
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap());

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

    // Bundle Adjustment
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(), 20);

    float scalingFactor;
    Sophus::SE3f Tc2w = pKFcur->GetPose();

    if (mInitialFrame.hasPosePrior() && mCurrentFrame.hasPosePrior() && mbVelocity && Tc2w.translation().norm() > 0.0f)
    {
        scalingFactor = mVelocity.translation().norm() / Tc2w.translation().norm();
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] CREATE_INITIAL_MAP_MONOCULAR: Scaling factor from velocity ["
            << scalingFactor << "]." << std::endl;
    }
    else
    {
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        scalingFactor = 1.0f / medianDepth;

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] CREATE_INITIAL_MAP_MONOCULAR: Median depth [" << medianDepth << "]."
            << std::endl;

        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50)  // TODO Check, originally 100 tracks
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "[" << mCurrentFrame.mnId << "] CREATE_INITIAL_MAP_MONOCULAR: Wrong initialization, reseting..."
                << std::endl;
            mpSystem->ResetActiveMap();
            return;
        }

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] CREATE_INITIAL_MAP_MONOCULAR: Scaling factor from median depth ["
            << scalingFactor << "]." << std::endl;
    }

    // Scale initial baseline
    Tc2w.translation() *= scalingFactor;
    pKFcur->SetPose(Tc2w);

    // Set initial pose for the current frame
    mCurrentFrame.SetPose(Tc2w);

    const auto& Twc = Tc2w.inverse();

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] "
        << "CREATE_INITIAL_MAP_MONOCULAR: After scaling Twc: " << Twc.translation().transpose() << std::endl;

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] "
        << "CREATE_INITIAL_MAP_MONOCULAR: After scaling Motion ||p||_cInitialcCurr: " << Twc.translation().norm()
        << " m" << std::endl;

    // Scale points
    std::vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
        if (vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * scalingFactor);
            pMP->UpdateNormalAndDepth();
        }
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

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;

    initID = pKFcur->mnId;

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] MONOCULAR_INITIALIZATION ok: New Map created with "
        << mpAtlas->MapPointsInMap() << " points." << std::endl;
}

void Tracking::CreateMapInAtlas()
{
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    mbSetInit = false;

    mnInitialFrameId = mCurrentFrame.mnId + 1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mbVelocity = false;
    if (mSensor == System::MONOCULAR)
    {
        mbReadyToInitializate = false;
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

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;

    if (!pRef)
    {
        return;
    }

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR)
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

RefKeyFrameTrackingResult Tracking::TrackReferenceKeyFrameWithBoW()
{
    RefKeyFrameTrackingResult result;

    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();
    if (mCurrentFrame.mFeatVec.empty() || !mpReferenceKF || mpReferenceKF->mFeatVec.empty())
    {
        return TrackReferenceKeyFrameNoBoW();
    }

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    // BoW matching is only valid for ORB/binary descriptors in this codebase.
    FeatureMatcher matcher(mReferenceKeyframeNNRatio, true, DescriptorType::BINARY);
    std::vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);
    result.num_matches = nmatches;

    // vpMapPointMatches[i] is the MapPoint matched to current-frame keypoint i.
    // Resolve the reference-KF keypoint via the MapPoint's observation list.
    // GetObservations() returns std::map<KeyFrame*, std::tuple<int,int>> where
    // std::get<0>(value) is the left-image keypoint index in that KeyFrame.
    for (int i = 0; i < static_cast<int>(vpMapPointMatches.size()); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if (!pMP || pMP->isBad())
        {
            continue;
        }

        MatchedKeypoint m;
        m.current_kp_idx = i;
        m.current_kp = mCurrentFrame.mvKeysUn[i];

        auto obs = pMP->GetObservations();
        auto it = obs.find(mpReferenceKF);
        if (it != obs.end())
        {
            int refIdx = std::get<0>(it->second);  // left-image kp index in ref KF
            m.source_kp_idx = refIdx;
            m.source_kp = mpReferenceKF->mvKeysUn[refIdx];
        }

        result.kf_matches.push_back(m);
        result.keypoints_matches.push_back({m.current_kp, m.source_kp});  // legacy field
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_WITH_BOW: nmatches=" << nmatches << std::endl;

    if (nmatches < mReferenceKeyframeMinBoWMatches)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF failed: nmatches=" << nmatches
            << " < MinBoWMatches=" << mReferenceKeyframeMinBoWMatches << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "TRACK_REF_KF: Less than 15 matches!!\n";
        return result;
    }

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.GetPose());

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = DiscardOutliersAndCountInliers(mCurrentFrame, nmatches, true);

    // After DiscardOutliersAndCountInliers(), outlier mvpMapPoints entries are nullptr.
    for (auto& m : result.kf_matches)
    {
        m.is_inlier = (m.current_kp_idx >= 0 && m.current_kp_idx < mCurrentFrame.N &&
                       mCurrentFrame.mvpMapPoints[m.current_kp_idx] != nullptr);
        if (m.is_inlier)
        {
            result.kf_matches_optimized.push_back(m);
            result.keypoints_inliers_optimized.push_back(m.current_kp);
            result.keypoints_matches_optimized.push_back({m.current_kp, m.source_kp});
        }
        else
        {
            result.keypoints_outliers_optimized.push_back(m.current_kp);
        }
    }

    result.num_matches_optimized = nmatchesMap;
    result.pose = mCurrentFrame.GetPose().inverse();

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_WITH_BOW: nmatchesMap=" << nmatchesMap << std::endl;

    if (nmatchesMap >= mReferenceKeyframeMinOptimizedMapMatches)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_WITH_BOW ok: nmatches=" << nmatches
            << " nmatchesMap=" << nmatchesMap << std::endl;
        result.success = true;
        return result;
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_WITH_BOW failed: nmatchesMap=" << nmatchesMap
        << " < MinOptimizedMapMatches=" << mReferenceKeyframeMinOptimizedMapMatches << std::endl;
    return result;
}

RefKeyFrameTrackingResult Tracking::TrackReferenceKeyFrameNoBoW()
{
    RefKeyFrameTrackingResult result;

    const DescriptorType descriptorType =
        (mCurrentFrame.mDescriptors.type() == CV_32FC1) ? DescriptorType::FLOAT32 : DescriptorType::BINARY;
    FeatureMatcher matcher(mReferenceKeyframeNNRatio, true, descriptorType);

    const int nmatches = matcher.SearchByBruteForce(mpReferenceKF, mCurrentFrame);
    result.num_matches = nmatches;

    // Build match list for introspection (best-effort; source index resolved via MapPoint observations).
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (!pMP || pMP->isBad())
        {
            continue;
        }

        MatchedKeypoint m;
        m.current_kp_idx = i;
        m.current_kp = mCurrentFrame.mvKeysUn[i];

        auto obs = pMP->GetObservations();
        auto it = obs.find(mpReferenceKF);
        if (it != obs.end())
        {
            const int refIdx = std::get<0>(it->second);
            if (refIdx >= 0 && refIdx < static_cast<int>(mpReferenceKF->mvKeysUn.size()))
            {
                m.source_kp_idx = refIdx;
                m.source_kp = mpReferenceKF->mvKeysUn[refIdx];
            }
        }

        result.kf_matches.push_back(m);
        result.keypoints_matches.push_back({m.current_kp, m.source_kp});
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_NO_BOW: nmatches=" << nmatches << std::endl;

    if (nmatches < mReferenceKeyframeMinBoWMatches)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF failed: nmatches=" << nmatches
            << " < MinMatches=" << mReferenceKeyframeMinBoWMatches << std::endl;
        return result;
    }

    mCurrentFrame.SetPose(mLastFrame.GetPose());
    Optimizer::PoseOptimization(&mCurrentFrame);

    int nmatchesMap = result.num_matches;
    nmatchesMap = DiscardOutliersAndCountInliers(mCurrentFrame, nmatchesMap, true);

    for (auto& m : result.kf_matches)
    {
        m.is_inlier = (m.current_kp_idx >= 0 && m.current_kp_idx < mCurrentFrame.N &&
                       mCurrentFrame.mvpMapPoints[m.current_kp_idx] != nullptr);
        if (m.is_inlier)
        {
            result.kf_matches_optimized.push_back(m);
            result.keypoints_inliers_optimized.push_back(m.current_kp);
            result.keypoints_matches_optimized.push_back({m.current_kp, m.source_kp});
        }
        else
        {
            result.keypoints_outliers_optimized.push_back(m.current_kp);
        }
    }

    result.num_matches_optimized = nmatchesMap;
    result.pose = mCurrentFrame.GetPose().inverse();

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] TRACK_REF_KF_NO_BOW: nmatchesMap=" << nmatchesMap << std::endl;

    if (nmatchesMap >= mReferenceKeyframeMinOptimizedMapMatches)
    {
        result.success = true;
    }
    return result;
}

MotionModelTrackingResult Tracking::TrackWithMotionModel()
{
    const DescriptorType descriptorType =
        (mCurrentFrame.mDescriptors.type() == CV_32FC1) ? DescriptorType::FLOAT32 : DescriptorType::BINARY;
    FeatureMatcher matcher(mMotionModelNNRatio, true, descriptorType);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity * mLastFrame.GetPose());

    MotionModelTrackingResult result;
    result.initial_pose = mCurrentFrame.GetPose().inverse();

    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, mMotionModelProjectionSearchTh,
                                              mSensor == System::MONOCULAR);

    result.num_matches = nmatches;

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: nmatches=" << nmatches << std::endl;

    if (nmatches < mMotionModelMinInitialMatches)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: Not enough matches [" << nmatches
            << "] < MinInitialMatches=" << mMotionModelMinInitialMatches << "." << std::endl;
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, mMotionModelRetryProjectionSearchTh,
                                              mSensor == System::MONOCULAR);

        result.retry = true;
        result.num_matches_retry = nmatches;

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: nmatches=" << nmatches << std::endl;
    }

    // Build a reverse lookup: MapPoint* -> index in mLastFrame.mvpMapPoints.
    // Used to find the last-frame keypoint that corresponds to each current-frame match.
    std::unordered_map<MapPoint*, int> lastFramePointIdx;
    lastFramePointIdx.reserve(mLastFrame.N);
    for (int j = 0; j < mLastFrame.N; j++)
    {
        if (mLastFrame.mvpMapPoints[j])
        {
            lastFramePointIdx[mLastFrame.mvpMapPoints[j]] = j;
        }
    }

    // Populate frame_matches (rich) and keypoints_matches (legacy pair vector).
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (!pMP)
        {
            continue;
        }

        MatchedKeypoint m;
        m.current_kp_idx = i;
        m.current_kp = mCurrentFrame.mvKeysUn[i];

        auto it = lastFramePointIdx.find(pMP);
        if (it != lastFramePointIdx.end())
        {
            m.source_kp_idx = it->second;
            m.source_kp = mLastFrame.mvKeysUn[it->second];
        }

        result.frame_matches.push_back(m);
        result.keypoints_matches.push_back({m.current_kp, m.source_kp});  // legacy field
    }

    if (nmatches < mMotionModelMinRetryMatches)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: Not enough matches [" << nmatches
            << "] with wider search < MinRetryMatches=" << mMotionModelMinRetryMatches << "." << std::endl;
        return result;
    }

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = DiscardOutliersAndCountInliers(mCurrentFrame, nmatches, false);
    result.num_matches_optimized = nmatchesMap;
    result.pose = mCurrentFrame.GetPose().inverse();

    // After DiscardOutliersAndCountInliers(), outlier entries in mvpMapPoints are
    // set to nullptr. Iterate frame_matches (captured before optimization) to
    // determine inlier/outlier status and populate optimized fields.
    for (auto& m : result.frame_matches)
    {
        m.is_inlier = (m.current_kp_idx >= 0 && m.current_kp_idx < mCurrentFrame.N &&
                       mCurrentFrame.mvpMapPoints[m.current_kp_idx] != nullptr);
        if (m.is_inlier)
        {
            result.frame_matches_optimized.push_back(m);
            result.keypoints_inliers_optimized.push_back(m.current_kp);
            result.keypoints_matches_optimized.push_back({m.current_kp, m.source_kp});
        }
        else
        {
            result.keypoints_outliers_optimized.push_back(m.current_kp);
        }
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: nmatchesMap=" << nmatchesMap << std::endl;

    if (nmatchesMap < mMotionModelMinOptimizedMapMatches)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL: Not enough matches after pose optimization ["
            << nmatchesMap << "] < MinOptimizedMapMatches=" << mMotionModelMinOptimizedMapMatches << "." << std::endl;
        return result;
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << mCurrentFrame.mnId << "] TRACK_WITH_MOTION_MODEL ok: nmatchesMap=" << nmatchesMap << std::endl;
    result.success = true;
    return result;
}

LocalMapTrackingResult Tracking::TrackLocalMap()
{
    LocalMapTrackingResult result;

    UpdateLocalMap();
    SearchLocalPoints();

    Optimizer::PoseOptimization(&mCurrentFrame);

    mnMatchesInliers = 0;

    // Update MapPoints Statistics and populate observation vectors.
    const Sophus::SE3f Tcw = mCurrentFrame.GetPose();
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (!mCurrentFrame.mvpMapPoints[i])
        {
            continue;
        }

        MapPointObservation obs;
        obs.keypoint_idx = i;
        obs.keypoint = mCurrentFrame.mvKeysUn[i];
        obs.pos_world = mCurrentFrame.mvpMapPoints[i]->GetWorldPos();
        obs.pos_camera = Tcw * obs.pos_world;
        obs.map_point_id = mCurrentFrame.mvpMapPoints[i]->mnId;
        obs.is_inlier = !mCurrentFrame.mvbOutlier[i];

        if (!mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
            if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
            {
                mnMatchesInliers++;
            }
            result.inlier_observations.push_back(obs);
            result.keypoints_inliers.push_back(obs.keypoint);  // legacy field
        }
        else
        {
            if (mSensor == System::STEREO)
            {
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            }
            result.outlier_observations.push_back(obs);
            result.keypoints_outliers.push_back(obs.keypoint);  // legacy field
        }
    }

    // Populate the pose field (was previously unpopulated).
    result.pose = mCurrentFrame.GetPose().inverse();

    result.num_matches = mnMatchesInliers;

    // Decide if the tracking was succesful
    // Inlier count is passed to LocalMapping for keyframe/point culling decisions.
    mpLocalMapper->mnMatchesInliers = mnMatchesInliers;

    if (mnMatchesInliers > mLocalMapGenericMinInliers)
    {
        result.success = true;
        return result;
    }

    if (mnMatchesInliers < mLocalMapVisualMinInliers)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] TRACK_LOCAL_MAP failed: inliers=" << mnMatchesInliers
            << " < VisualMinInliers=" << mLocalMapVisualMinInliers << "." << std::endl;
        return result;
    }

    result.success = true;
    return result;
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

    if (mSensor != System::MONOCULAR)
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
    if (mSensor == System::MONOCULAR)
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
    const bool c1c = mSensor != System::MONOCULAR &&
                     (mnMatchesInliers < nRefMatches * mNewKFWeakTrackingRatio || bNeedToInsertClose);

    // Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 =
        (((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) && mnMatchesInliers > mNewKFMinInliers);

    const bool needKeyFrame = ((c1a || c1b || c1c) && c2);
    if (!needKeyFrame)
    {
        return false;
    }

    std::string kfReason;
    if (c1a)
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
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] NEED_NEW_KEYFRAME: reason=" << kfReason << std::endl;
        return true;
    }

    mpLocalMapper->InterruptBA();
    if (mSensor == System::MONOCULAR)
    {
        return false;
    }
    const int queueSize = mpLocalMapper->KeyframesInQueue();
    const bool canInsert = (queueSize < mNewKFMaxKFsInQueue);
    if (canInsert)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mCurrentFrame.mnId << "] NEED_NEW_KEYFRAME: reason=" << kfReason << std::endl;
    }
    return canInsert;
}

void Tracking::CreateNewKeyFrame()
{
    if (!mpLocalMapper->SetNotStop(true))
    {
        return;
    }
    KeyFrame* pKF = new KeyFrame(mCurrentFrame, mpAtlas->GetCurrentMap());

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if (mSensor != System::MONOCULAR)
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

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
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
        const DescriptorType descriptorType =
            (mCurrentFrame.mDescriptors.type() == CV_32FC1) ? DescriptorType::FLOAT32 : DescriptorType::BINARY;
        FeatureMatcher matcher(0.8, true, descriptorType);
        int th = 1;

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
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
        {
            if (!pMP->isBad())
            {
                const auto observations = pMP->GetObservations();
                for (auto it = observations.begin(), itend = observations.end(); it != itend; it++)
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

    // Sort by (votes DESC, mnId ASC) so that pKFmax selection and mvpLocalKeyFrames
    // ordering are deterministic regardless of pointer address (ASLR).
    std::vector<std::pair<int, KeyFrame*>> vSortedKFs;
    vSortedKFs.reserve(keyframeCounter.size());
    for (const auto& kv : keyframeCounter)
    {
        vSortedKFs.emplace_back(kv.second, kv.first);
    }
    std::sort(vSortedKFs.begin(), vSortedKFs.end(),
              [](const std::pair<int, KeyFrame*>& a, const std::pair<int, KeyFrame*>& b)
              {
                  if (a.first != b.first)
                      return a.first > b.first;            // votes DESC
                  return a.second->mnId < b.second->mnId;  // mnId ASC tie-break
              });

    int max = 0;
    KeyFrame* pKFmax = static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * vSortedKFs.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for (const auto& [votes, pKF] : vSortedKFs)
    {
        if (pKF->isBad())
        {
            continue;
        }
        if (votes > max)
        {
            max = votes;
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

        // Sort children by mnId so the first non-bad child selected is deterministic
        // regardless of pointer address (ASLR), since std::set<KeyFrame*> is pointer-ordered.
        std::vector<KeyFrame*> vChilds;
        const std::set<KeyFrame*> spChilds = pKF->GetChilds();
        vChilds.assign(spChilds.begin(), spChilds.end());
        std::sort(vChilds.begin(), vChilds.end(), [](KeyFrame* a, KeyFrame* b) { return a->mnId < b->mnId; });
        for (KeyFrame* pChildKF : vChilds)
        {
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

    if (pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

void Tracking::Reset(bool bLocMap)
{
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "System Reseting" << std::endl;

    // Reset Local Mapping
    if (!bLocMap)
    {
        mpLocalMapper->RequestReset();
    }

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    mnInitialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;
    mbSetInit = false;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();
}

void Tracking::ResetActiveMap(bool bLocMap)
{
    if (!bLocMap)
    {
        mpLocalMapper->RequestReset();
    }

    // Replace the map (single-map Atlas semantics)
    mpAtlas->CreateNewMap();

    mnLastInitFrameId = Frame::nNextId;
    mState = NO_IMAGES_YET;

    mbReadyToInitializate = false;

    mnInitialFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    mbVelocity = false;
}

std::vector<MapPoint*> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

bool Tracking::isLastFrameKeyframe()
{
    return mnLastKeyFrameId == mLastFrame.mnId;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}
}  // namespace ORB_SLAM3
