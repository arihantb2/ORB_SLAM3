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

#include "Settings.h"

#include "CameraModels/GeometricCamera.h"
#include "CameraModels/Metashape.h"
#include "CameraModels/Pinhole.h"
#include "Optimizer.h"
#include "Verbose.h"

#include "System.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>

#include <string>
#include <vector>

namespace ORB_SLAM3
{

template <>
float Settings::readParameter<float>(cv::FileStorage& fSettings, const std::string& name, bool& found,
                                     const bool required)
{
    cv::FileNode node = fSettings[name];
    if (node.empty())
    {
        if (required)
        {
            throw std::runtime_error(name + " required parameter does not exist, aborting...");
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << name << " optional parameter does not exist..." << std::endl;
            found = false;
            return 0.0f;
        }
    }
    else if (!node.isReal())
    {
        throw std::runtime_error(name + " parameter must be a real number, aborting...");
    }
    else
    {
        found = true;
        return node.real();
    }
}

template <>
int Settings::readParameter<int>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required)
{
    cv::FileNode node = fSettings[name];
    if (node.empty())
    {
        if (required)
        {
            throw std::runtime_error(name + " required parameter does not exist, aborting...");
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << name << " optional parameter does not exist..." << std::endl;
            found = false;
            return 0;
        }
    }
    else if (!node.isInt())
    {
        throw std::runtime_error(name + " parameter must be an integer number, aborting...");
    }
    else
    {
        found = true;
        return node.operator int();
    }
}

template <>
std::string Settings::readParameter<std::string>(cv::FileStorage& fSettings, const std::string& name, bool& found,
                                                 const bool required)
{
    cv::FileNode node = fSettings[name];
    if (node.empty())
    {
        if (required)
        {
            throw std::runtime_error(name + " required parameter does not exist, aborting...");
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << name << " optional parameter does not exist..." << std::endl;
            found = false;
            return std::string();
        }
    }
    else if (!node.isString())
    {
        throw std::runtime_error(name + " parameter must be a string, aborting...");
    }
    else
    {
        found = true;
        return node.string();
    }
}

template <>
cv::Mat Settings::readParameter<cv::Mat>(cv::FileStorage& fSettings, const std::string& name, bool& found,
                                         const bool required)
{
    cv::FileNode node = fSettings[name];
    if (node.empty())
    {
        if (required)
        {
            throw std::runtime_error(name + " required parameter does not exist, aborting...");
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << name << " optional parameter does not exist..." << std::endl;
            found = false;
            return cv::Mat();
        }
    }
    else
    {
        found = true;
        return node.mat();
    }
}

Settings::Settings(const std::string& algorithmConfigPath, const int& sensor, const CameraCalibrationInput& calib)
    : bNeedToUndistort_(false), bNeedToRectify_(false), bNeedToResize1_(false), bNeedToResize2_(false)
{
    sensor_ = sensor;

    if (!calib.camera1)
    {
        throw std::runtime_error("CameraCalibrationInput: camera1 is required");
    }
    if (sensor_ == System::STEREO && !calib.camera2)
    {
        throw std::runtime_error("CameraCalibrationInput: camera2 is required for stereo");
    }

    calibration1_ = calib.camera1.get();
    calibration2_ = calib.camera2.get();
    originalCalib1_ = calib.camera1.get();
    originalCalib2_ = calib.camera2.get();
    originalImSize_ = calib.originalImSize;
    newImSize_ = calib.newImSize;
    cameraType_ = static_cast<CameraType>(calib.cameraType);
    Tlr_ = calib.T_c1_c2;
    thDepth_ = calib.thDepth;
    bNeedToRectify_ = calib.bNeedToRectify;
    bNeedToUndistort_ = calib.bNeedToUndistort;
    bNeedToResize1_ = calib.bNeedToResize1;
    vPinHoleDistorsion1_ = calib.vPinHoleDistorsion1;
    vPinHoleDistorsion2_ = calib.vPinHoleDistorsion2;

    b_ = Tlr_.translation().norm();
    bf_ = b_ * calibration1_->getParameter(0);

    cv::FileStorage fSettings(algorithmConfigPath, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        throw std::runtime_error("Failed to open algorithm config at: " + algorithmConfigPath);
    }
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Loading algorithm settings from " << algorithmConfigPath << std::endl;

    bool found;
    fps_ = readParameter<int>(fSettings, "Camera.fps", found);
    bRGB_ = (bool)readParameter<int>(fSettings, "Camera.RGB", found);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded image info (fps, RGB)" << std::endl;

    bool typeFound;
    featureExtractorType_ = readParameter<std::string>(fSettings, "FeatureExtractor.type", typeFound, false);
    if (!typeFound)
    {
        featureExtractorType_ = "GridORB";
    }
    if (featureExtractorType_ == "ORB")
    {
        readORB(fSettings);
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded ORB settings" << std::endl;
    }
    else if (featureExtractorType_ == "BRISK")
    {
        readBRISK(fSettings);
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded BRISK settings" << std::endl;
    }
    else  // "GridORB" (default)
    {
        readGridORB(fSettings);
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded GridORB settings" << std::endl;
    }
    readOtherParameters(fSettings);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded misc parameters" << std::endl;

    if (bNeedToRectify_)
    {
        precomputeRectificationMaps();
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Computed rectification maps" << std::endl;
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "----------------------------------" << std::endl;
}

void Settings::readGridORB(cv::FileStorage& fSettings)
{
    bool found;

    nFeatures_ = readParameter<int>(fSettings, "FeatureExtractor.GridORB.nFeatures", found);
    nInitFeatures_ = readParameter<int>(fSettings, "FeatureExtractor.GridORB.nInitFeatures", found, false);
    if (!found)
    {
        nInitFeatures_ = static_cast<int>(2.5f * nFeatures_);
    }
    scaleFactor_ = readParameter<float>(fSettings, "FeatureExtractor.GridORB.scaleFactor", found);
    nLevels_ = readParameter<int>(fSettings, "FeatureExtractor.GridORB.nLevels", found);
    initThFAST_ = readParameter<int>(fSettings, "FeatureExtractor.GridORB.iniThFAST", found);
    minThFAST_ = readParameter<int>(fSettings, "FeatureExtractor.GridORB.minThFAST", found);
    matchThLow_  = readParameter<int>(fSettings, "FeatureMatcher.thLow",  found, 50,  false);
    matchThHigh_ = readParameter<int>(fSettings, "FeatureMatcher.thHigh", found, 100, false);
}

void Settings::readORB(cv::FileStorage& fSettings)
{
    bool found;

    nFeatures_ = readParameter<int>(fSettings, "FeatureExtractor.ORB.nFeatures", found);
    nInitFeatures_ = readParameter<int>(fSettings, "FeatureExtractor.ORB.nInitFeatures", found, false);
    if (!found)
    {
        nInitFeatures_ = static_cast<int>(2.5f * nFeatures_);
    }
    scaleFactor_ = readParameter<float>(fSettings, "FeatureExtractor.ORB.scaleFactor", found);
    nLevels_ = readParameter<int>(fSettings, "FeatureExtractor.ORB.nLevels", found);
    initThFAST_ = readParameter<int>(fSettings, "FeatureExtractor.ORB.iniThFAST", found);
    minThFAST_ = 0;  // unused by VanillaORB — GridORB uses FeatureExtractor.GridORB.minThFAST

    const std::string scoreTypeStr =
        readParameter<std::string>(fSettings, "FeatureExtractor.ORB.scoreType", found, false);
    if (!found || scoreTypeStr == "HARRIS")
    {
        orbScoreType_ = 0;  // cv::ORB::HARRIS_SCORE
    }
    else if (scoreTypeStr == "FAST")
    {
        orbScoreType_ = 1;  // cv::ORB::FAST_SCORE
    }
    else
    {
        throw std::runtime_error("FeatureExtractor.ORB.scoreType must be \"HARRIS\" or \"FAST\"");
    }
    matchThLow_  = readParameter<int>(fSettings, "FeatureMatcher.thLow",  found, 50,  false);
    matchThHigh_ = readParameter<int>(fSettings, "FeatureMatcher.thHigh", found, 100, false);
}

void Settings::readBRISK(cv::FileStorage& fSettings)
{
    bool found;

    nFeatures_ = readParameter<int>(fSettings, "FeatureExtractor.BRISK.nFeatures", found);
    nInitFeatures_ = readParameter<int>(fSettings, "FeatureExtractor.BRISK.nInitFeatures", found, false);
    if (!found)
    {
        nInitFeatures_ = static_cast<int>(2.5f * nFeatures_);
    }
    nLevels_ = readParameter<int>(fSettings, "FeatureExtractor.BRISK.nLevels", found, 4, false);
    briskThreshold_ = readParameter<int>(fSettings, "FeatureExtractor.BRISK.threshold", found, 30, false);

    // BRISK's octave scale is always 2x — scaleFactor_ is set here only for
    // consistency with code that reads it (e.g. pyramid allocation).
    scaleFactor_ = 2.0f;
    bool scaleFound;
    readParameter<float>(fSettings, "FeatureExtractor.BRISK.scaleFactor", scaleFound, 2.0f, false);
    if (scaleFound)
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "[WARN] FeatureExtractor.BRISK.scaleFactor is ignored; BRISK uses 2.0" << std::endl;
    }

    matchThLow_  = readParameter<int>(fSettings, "FeatureMatcher.thLow",  found, 100, false);
    matchThHigh_ = readParameter<int>(fSettings, "FeatureMatcher.thHigh", found, 200, false);
}

void Settings::readOtherParameters(cv::FileStorage& fSettings)
{
    bool found;

    thFarPoints_ = readParameter<float>(fSettings, "System.thFarPoints", found, 0.0f, false);

    localMappingOptimizeEveryTSeconds_ =
        readParameter<float>(fSettings, "LocalMapping.OptimizeEveryTSeconds", found, 5.0f, false);
    localMappingMinKeyframesForLBA_ = readParameter<int>(fSettings, "LocalMapping.MinKeyframesForLBA", found, 2, false);

    localMappingMPCullingMinObs_ =
        readParameter<int>(fSettings, "LocalMapping.MapPointCulling.MinObservations", found, 2, false);
    localMappingMPCullingMinKFAgeForObsCheck_ =
        readParameter<int>(fSettings, "LocalMapping.MapPointCulling.MinKFAgeForObsCheck", found, 2, false);
    localMappingMPCullingMaxKFAgeInRecent_ =
        readParameter<int>(fSettings, "LocalMapping.MapPointCulling.MaxKFAgeInRecent", found, 3, false);
    localMappingMPCullingMinFoundRatio_ =
        readParameter<float>(fSettings, "LocalMapping.MapPointCulling.MinFoundRatio", found, 0.25f, false);

    localMappingCreateNewMapPointsCovisibility_ =
        readParameter<int>(fSettings, "LocalMapping.CreateNewMapPoints.CovisibilityNeighbors", found, 30, false);
    localMappingCreateNewMapPointsMatchRatio_ =
        readParameter<float>(fSettings, "LocalMapping.CreateNewMapPoints.MatchRatio", found, 0.6f, false);
    localMappingCreateNewMapPointsMinBaselineDepthRatio_ =
        readParameter<float>(fSettings, "LocalMapping.CreateNewMapPoints.MinBaselineDepthRatio", found, 0.01f, false);
    localMappingCreateNewMapPointsMaxCosParallax_ =
        readParameter<float>(fSettings, "LocalMapping.CreateNewMapPoints.MaxCosParallax", found, 0.9998f, false);
    localMappingCreateNewMapPointsScaleConsistencyFactor_ =
        readParameter<float>(fSettings, "LocalMapping.CreateNewMapPoints.ScaleConsistencyFactor", found, 1.5f, false);

    localMappingSearchInNeighborsNumNeighborKFs_ =
        readParameter<int>(fSettings, "LocalMapping.SearchInNeighbors.NumNeighborKFs", found, 30, false);
    localMappingSearchInNeighborsNumSecondNeighbors_ =
        readParameter<int>(fSettings, "LocalMapping.SearchInNeighbors.NumSecondNeighbors", found, 20, false);
    localMappingSearchInNeighborsMaxTemporalNeighbors_ =
        readParameter<int>(fSettings, "LocalMapping.SearchInNeighbors.MaxTemporalNeighbors", found, 20, false);

    localMappingKeyFrameCullingRedundantRatio_ =
        readParameter<float>(fSettings, "LocalMapping.KeyFrameCulling.RedundantObservationRatio", found, 0.9f, false);
    localMappingKeyFrameCullingMinObsInOthers_ =
        readParameter<int>(fSettings, "LocalMapping.KeyFrameCulling.MinObservationsInOthers", found, 3, false);
    localMappingKeyFrameCullingMaxKeyframesToCheck_ =
        readParameter<int>(fSettings, "LocalMapping.KeyFrameCulling.MaxKeyframesToCheck", found, 100, false);
    localMappingKeyFrameCullingEarlyExitAfterAbort_ =
        readParameter<int>(fSettings, "LocalMapping.KeyFrameCulling.EarlyExitAfterAbort", found, 20, false);

    monocularInitSearchWindowSize_ =
        readParameter<int>(fSettings, "Tracking.MonocularInit.SearchWindowSize", found, 100, false);
    monocularInitMinKeypoints_ =
        readParameter<int>(fSettings, "Tracking.MonocularInit.MinKeypoints", found, 100, false);
    monocularInitNNRatio_ = readParameter<float>(fSettings, "Tracking.MonocularInit.NNRatio", found, 0.9f, false);
    monocularInitMinMatches_ = readParameter<int>(fSettings, "Tracking.MonocularInit.MinMatches", found, 100, false);

    stereoInitMinKeypoints_ = readParameter<int>(fSettings, "Tracking.StereoInit.MinKeypoints", found, 500, false);
    stereoInitMinMapPoints_ = readParameter<int>(fSettings, "Tracking.StereoInit.MinMapPoints", found, 100, false);
    // Local bundle adjustment prior toggles for Optimizer.
    // Exposed as integer flags (0/1) in the YAML config.
    bool use_pose_priors = false;
    bool use_scale_priors = true;
    bool use_odometry_priors = false;

    const int pose_priors_flag =
        readParameter<int>(fSettings, "Optimizer.LocalBundleAdjustment.PosePriors", found, 0, false);
    use_pose_priors = (pose_priors_flag != 0);

    const int scale_priors_flag =
        readParameter<int>(fSettings, "Optimizer.LocalBundleAdjustment.ScalePriors", found, 1, false);
    use_scale_priors = (scale_priors_flag != 0);

    const int odom_priors_flag =
        readParameter<int>(fSettings, "Optimizer.LocalBundleAdjustment.OdometryPriors", found, 0, false);
    use_odometry_priors = (odom_priors_flag != 0);

    Optimizer::ConfigureLocalBundleAdjustmentPriors(use_pose_priors, use_scale_priors, use_odometry_priors);

    referenceKeyframeNNRatio_ =
        readParameter<float>(fSettings, "Tracking.ReferenceKeyframe.NNRatio", found, 0.7f, false);
    referenceKeyframeMinBoWMatches_ =
        readParameter<int>(fSettings, "Tracking.ReferenceKeyframe.MinBoWMatches", found, 15, false);
    referenceKeyframeMinOptimizedMapMatches_ =
        readParameter<int>(fSettings, "Tracking.ReferenceKeyframe.MinOptimizedMapMatches", found, 10, false);

    motionModelNNRatio_ = readParameter<float>(fSettings, "Tracking.MotionModel.NNRatio", found, 0.9f, false);
    motionModelProjectionSearchTh_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.ProjectionSearchTh", found, 7, false);
    motionModelMinInitialMatches_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.MinInitialMatches", found, 20, false);
    motionModelRetryProjectionSearchTh_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.RetryProjectionSearchTh", found, 15, false);
    motionModelMinRetryMatches_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.MinRetryMatches", found, 20, false);
    motionModelMinOptimizedMapMatches_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.MinOptimizedMapMatches", found, 10, false);

    localMapGenericMinInliers_ = readParameter<int>(fSettings, "Tracking.LocalMap.GenericMinInliers", found, 10, false);
    localMapVisualMinInliers_ = readParameter<int>(fSettings, "Tracking.LocalMap.VisualMinInliers", found, 30, false);

    newKFMinTrackedClosePoints_ =
        readParameter<int>(fSettings, "Tracking.NewKF.MinTrackedClosePoints", found, 100, false);
    newKFMinNonTrackedClosePoints_ =
        readParameter<int>(fSettings, "Tracking.NewKF.MinNonTrackedClosePoints", found, 70, false);
    newKFRefRatioMono_ = readParameter<float>(fSettings, "Tracking.NewKF.RefRatioMono", found, 0.9f, false);
    newKFRefRatioStereoFewKFs_ =
        readParameter<float>(fSettings, "Tracking.NewKF.RefRatioStereoFewKFs", found, 0.4f, false);
    newKFRefRatioStereo_ = readParameter<float>(fSettings, "Tracking.NewKF.RefRatioStereo", found, 0.75f, false);
    newKFWeakTrackingRatio_ = readParameter<float>(fSettings, "Tracking.NewKF.WeakTrackingRatio", found, 0.25f, false);
    newKFMinInliers_ = readParameter<int>(fSettings, "Tracking.NewKF.MinInliers", found, 15, false);
    newKFMaxKFsInQueue_ = readParameter<int>(fSettings, "Tracking.NewKF.MaxKFsInQueue", found, 3, false);
    forceEveryFrameKeyframe_ = (readParameter<int>(fSettings, "Tracking.NewKF.ForceEveryFrame", found, 0, false) != 0);

    lostResetMinKFs_ = readParameter<int>(fSettings, "Tracking.LostResetMinKFs", found, 999999, false);
}

void Settings::precomputeRectificationMaps()
{
    //Precompute rectification maps, new calibrations, ...
    cv::Mat K1;
    cv::Mat K2;
    cv::Mat D1;
    cv::Mat D2;

    if (cameraType_ == PinHole)
    {
        K1 = static_cast<Pinhole*>(calibration1_)->toK();
        K2 = static_cast<Pinhole*>(calibration2_)->toK();
        D1 = camera1DistortionCoef();
        D2 = camera2DistortionCoef();
    }
    else if (cameraType_ == Metashape)
    {
        K1 = static_cast<class Metashape*>(calibration1_)->toK();
        K2 = static_cast<class Metashape*>(calibration2_)->toK();
        // OpenCV radtan: (k1, k2, p1, p2, k3). Metashape params [fx,fy,cx,cy,k1,k2,k3,k4,p1,p2,skew]; k4 omitted for API.
        D1 = (cv::Mat_<float>(5, 1) << calibration1_->getParameter(4), calibration1_->getParameter(5),
              calibration1_->getParameter(8), calibration1_->getParameter(9), calibration1_->getParameter(6));
        D2 = (cv::Mat_<float>(5, 1) << calibration2_->getParameter(4), calibration2_->getParameter(5),
              calibration2_->getParameter(8), calibration2_->getParameter(9), calibration2_->getParameter(6));
    }
    else
    {
        throw std::runtime_error("precomputeRectificationMaps only supports PinHole and Metashape camera types");
    }

    K1.convertTo(K1, CV_64F);
    K2.convertTo(K2, CV_64F);
    D1.convertTo(D1, CV_64F);
    D2.convertTo(D2, CV_64F);

    cv::Mat cvTlr;
    cv::eigen2cv(Tlr_.inverse().matrix3x4(), cvTlr);
    cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
    R12.convertTo(R12, CV_64F);
    cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
    t12.convertTo(t12, CV_64F);

    cv::Mat R_r1_u1, R_r2_u2;
    cv::Mat P1, P2, Q;

    cv::stereoRectify(K1, D1, K2, D2, newImSize_, R12, t12, R_r1_u1, R_r2_u2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1,
                      newImSize_);
    cv::initUndistortRectifyMap(K1, D1, R_r1_u1, P1.rowRange(0, 3).colRange(0, 3), newImSize_, CV_32F, M1l_, M2l_);
    cv::initUndistortRectifyMap(K2, D2, R_r2_u2, P2.rowRange(0, 3).colRange(0, 3), newImSize_, CV_32F, M1r_, M2r_);

    //Update calibration to rectified intrinsics
    calibration1_->setParameter(P1.at<double>(0, 0), 0);
    calibration1_->setParameter(P1.at<double>(1, 1), 1);
    calibration1_->setParameter(P1.at<double>(0, 2), 2);
    calibration1_->setParameter(P1.at<double>(1, 2), 3);

    if (cameraType_ == Metashape)
    {
        // Zero distortion and skew so runtime projection matches rectified pinhole assumptions
        for (int i = 4; i <= 10; i++)
        {
            calibration1_->setParameter(0.f, i);
        }
        calibration2_->setParameter(P2.at<double>(0, 0), 0);
        calibration2_->setParameter(P2.at<double>(1, 1), 1);
        calibration2_->setParameter(P2.at<double>(0, 2), 2);
        calibration2_->setParameter(P2.at<double>(1, 2), 3);
        for (int i = 4; i <= 10; i++)
        {
            calibration2_->setParameter(0.f, i);
        }
    }

    //Update bf
    bf_ = b_ * P1.at<double>(0, 0);
}

std::ostream& operator<<(std::ostream& output, const Settings& settings)
{
    output << "SLAM settings: " << std::endl;

    output << "\t-Camera 1 parameters (";
    if (settings.cameraType_ == Settings::PinHole || settings.cameraType_ == Settings::Rectified)
    {
        output << "Pinhole";
    }
    else if (settings.cameraType_ == Settings::Metashape)
    {
        output << "Metashape";
    }
    else
    {
        output << "Unknown";
    }
    output << ")" << ": [";
    for (size_t i = 0; i < settings.originalCalib1_->size(); i++)
    {
        output << " " << settings.originalCalib1_->getParameter(i);
    }
    output << " ]" << std::endl;

    if (!settings.vPinHoleDistorsion1_.empty())
    {
        output << "\t-Camera 1 distortion parameters: [ ";
        for (float d : settings.vPinHoleDistorsion1_)
        {
            output << " " << d;
        }
        output << " ]" << std::endl;
    }

    if (settings.sensor_ == System::STEREO)
    {
        output << "\t-Camera 2 parameters (";
        if (settings.cameraType_ == Settings::PinHole || settings.cameraType_ == Settings::Rectified)
        {
            output << "Pinhole";
        }
        else if (settings.cameraType_ == Settings::Metashape)
        {
            output << "Metashape";
        }
        else
        {
            output << "Unknown";
        }
        output << "" << ": [";
        for (size_t i = 0; i < settings.originalCalib2_->size(); i++)
        {
            output << " " << settings.originalCalib2_->getParameter(i);
        }
        output << " ]" << std::endl;

        if (!settings.vPinHoleDistorsion2_.empty())
        {
            output << "\t-Camera 1 distortion parameters: [ ";
            for (float d : settings.vPinHoleDistorsion2_)
            {
                output << " " << d;
            }
            output << " ]" << std::endl;
        }
    }

    output << "\t-Original image size: [ " << settings.originalImSize_.width << " , " << settings.originalImSize_.height
           << " ]" << std::endl;
    output << "\t-Current image size: [ " << settings.newImSize_.width << " , " << settings.newImSize_.height << " ]"
           << std::endl;

    if (settings.bNeedToRectify_)
    {
        output << "\t-Camera 1 parameters after rectification: [ ";
        for (size_t i = 0; i < settings.calibration1_->size(); i++)
        {
            output << " " << settings.calibration1_->getParameter(i);
        }
        output << " ]" << std::endl;
    }
    else if (settings.bNeedToResize1_)
    {
        output << "\t-Camera 1 parameters after resize: [ ";
        for (size_t i = 0; i < settings.calibration1_->size(); i++)
        {
            output << " " << settings.calibration1_->getParameter(i);
        }
        output << " ]" << std::endl;
    }

    output << "\t-Sequence FPS: " << settings.fps_ << std::endl;

    //Stereo stuff
    if (settings.sensor_ == System::STEREO)
    {
        output << "\t-Stereo baseline: " << settings.b_ << std::endl;
        output << "\t-Stereo depth threshold : " << settings.thDepth_ << std::endl;
    }

    // Extractor type
    output << "\t-Extractor type: " << settings.featureExtractorType_ << std::endl;

    return output;
}
};  // namespace ORB_SLAM3
