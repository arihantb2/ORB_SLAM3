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
#include "Converter.h"
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
            Verbose::Print(Verbose::VERBOSITY_QUIET) << name << " optional parameter does not exist..." << std::endl;
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
            Verbose::Print(Verbose::VERBOSITY_QUIET) << name << " optional parameter does not exist..." << std::endl;
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
            Verbose::Print(Verbose::VERBOSITY_QUIET) << name << " optional parameter does not exist..." << std::endl;
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
            Verbose::Print(Verbose::VERBOSITY_QUIET) << name << " optional parameter does not exist..." << std::endl;
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

Settings::Settings(const std::string& configFile, const int& sensor)
    : bNeedToUndistort_(false), bNeedToRectify_(false), bNeedToResize1_(false), bNeedToResize2_(false)
{
    sensor_ = sensor;

    //Open settings file
    cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        throw std::runtime_error("Failed to open settings file at: " + configFile);
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Loading settings from " << configFile << std::endl;
    }

    //Read first camera
    readCamera1(fSettings);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded camera 1" << std::endl;

    //Read second camera if stereo (not rectified)
    if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO)
    {
        readCamera2(fSettings);
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded camera 2" << std::endl;
    }

    //Read image info
    readImageInfo(fSettings);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded image info" << std::endl;

    if (sensor_ == System::IMU_MONOCULAR || sensor_ == System::IMU_STEREO)
    {
        readIMU(fSettings);
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded IMU calibration" << std::endl;
    }

    readORB(fSettings);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded ORB settings" << std::endl;
    readViewer(fSettings);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded viewer settings" << std::endl;
    readOtherParameters(fSettings);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Loaded misc parameters" << std::endl;

    if (bNeedToRectify_)
    {
        precomputeRectificationMaps();
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "\t-Computed rectification maps" << std::endl;
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "----------------------------------" << std::endl;
}

void Settings::readCamera1(cv::FileStorage& fSettings)
{
    bool found;

    //Read camera model
    std::string cameraModel = readParameter<std::string>(fSettings, "Camera.type", found);

    std::vector<float> vCalibration;
    if (cameraModel == "PinHole")
    {
        cameraType_ = PinHole;

        //Read intrinsic parameters
        float fx = readParameter<float>(fSettings, "Camera1.fx", found);
        float fy = readParameter<float>(fSettings, "Camera1.fy", found);
        float cx = readParameter<float>(fSettings, "Camera1.cx", found);
        float cy = readParameter<float>(fSettings, "Camera1.cy", found);

        vCalibration = {fx, fy, cx, cy};

        calibration1_ = new Pinhole(vCalibration);
        originalCalib1_ = new Pinhole(vCalibration);

        //Check if it is a distorted PinHole
        readParameter<float>(fSettings, "Camera1.k1", found, false);
        if (found)
        {
            readParameter<float>(fSettings, "Camera1.k3", found, false);
            if (found)
            {
                vPinHoleDistorsion1_.resize(5);
                vPinHoleDistorsion1_[4] = readParameter<float>(fSettings, "Camera1.k3", found);
            }
            else
            {
                vPinHoleDistorsion1_.resize(4);
            }
            vPinHoleDistorsion1_[0] = readParameter<float>(fSettings, "Camera1.k1", found);
            vPinHoleDistorsion1_[1] = readParameter<float>(fSettings, "Camera1.k2", found);
            vPinHoleDistorsion1_[2] = readParameter<float>(fSettings, "Camera1.p1", found);
            vPinHoleDistorsion1_[3] = readParameter<float>(fSettings, "Camera1.p2", found);
        }

        //Check if we need to correct distortion from the images
        if ((sensor_ == System::MONOCULAR || sensor_ == System::IMU_MONOCULAR) && vPinHoleDistorsion1_.size() != 0)
        {
            bNeedToUndistort_ = true;
        }
    }
    else if (cameraModel == "Rectified")
    {
        cameraType_ = Rectified;

        //Read intrinsic parameters
        float fx = readParameter<float>(fSettings, "Camera1.fx", found);
        float fy = readParameter<float>(fSettings, "Camera1.fy", found);
        float cx = readParameter<float>(fSettings, "Camera1.cx", found);
        float cy = readParameter<float>(fSettings, "Camera1.cy", found);

        vCalibration = {fx, fy, cx, cy};

        calibration1_ = new Pinhole(vCalibration);
        originalCalib1_ = new Pinhole(vCalibration);

        //Rectified images are assumed to be ideal PinHole images (no distortion)
    }
    else if (cameraModel == "Metashape")
    {
        cameraType_ = Metashape;

        const float f = readParameter<float>(fSettings, "Camera1.f", found);
        const float cx = readParameter<float>(fSettings, "Camera1.cx", found);
        const float cy = readParameter<float>(fSettings, "Camera1.cy", found);
        const int width = readParameter<int>(fSettings, "Camera.width", found);
        const int height = readParameter<int>(fSettings, "Camera.height", found);
        const float b1 = readParameter<float>(fSettings, "Camera1.b1", found);
        const float b2 = readParameter<float>(fSettings, "Camera1.b2", found);

        const float k1 = readParameter<float>(fSettings, "Camera1.k1", found);
        const float k2 = readParameter<float>(fSettings, "Camera1.k2", found);
        const float k3 = readParameter<float>(fSettings, "Camera1.k3", found);
        const float k4 = readParameter<float>(fSettings, "Camera1.k4", found);
        const float p1 = readParameter<float>(fSettings, "Camera1.p1", found);
        const float p2 = readParameter<float>(fSettings, "Camera1.p2", found);

        const float fx = f + b1;
        const float fy = f;
        const float skew = b2;

        const float cx_abs = cx + 0.5f * static_cast<float>(width);
        const float cy_abs = cy + 0.5f * static_cast<float>(height);

        vCalibration = {fx, fy, cx_abs, cy_abs, k1, k2, k3, k4, p1, p2, skew};

        calibration1_ = new ORB_SLAM3::Metashape(vCalibration);
        originalCalib1_ = new ORB_SLAM3::Metashape(vCalibration);
    }
    else
    {
        throw std::runtime_error("Invalid camera model: " + cameraModel);
    }
}

void Settings::readCamera2(cv::FileStorage& fSettings)
{
    bool found;
    std::vector<float> vCalibration;
    if (cameraType_ == PinHole)
    {
        bNeedToRectify_ = true;

        //Read intrinsic parameters
        float fx = readParameter<float>(fSettings, "Camera2.fx", found);
        float fy = readParameter<float>(fSettings, "Camera2.fy", found);
        float cx = readParameter<float>(fSettings, "Camera2.cx", found);
        float cy = readParameter<float>(fSettings, "Camera2.cy", found);

        vCalibration = {fx, fy, cx, cy};

        calibration2_ = new Pinhole(vCalibration);
        originalCalib2_ = new Pinhole(vCalibration);

        //Check if it is a distorted PinHole
        readParameter<float>(fSettings, "Camera2.k1", found, false);
        if (found)
        {
            readParameter<float>(fSettings, "Camera2.k3", found, false);
            if (found)
            {
                vPinHoleDistorsion2_.resize(5);
                vPinHoleDistorsion2_[4] = readParameter<float>(fSettings, "Camera2.k3", found);
            }
            else
            {
                vPinHoleDistorsion2_.resize(4);
            }
            vPinHoleDistorsion2_[0] = readParameter<float>(fSettings, "Camera2.k1", found);
            vPinHoleDistorsion2_[1] = readParameter<float>(fSettings, "Camera2.k2", found);
            vPinHoleDistorsion2_[2] = readParameter<float>(fSettings, "Camera2.p1", found);
            vPinHoleDistorsion2_[3] = readParameter<float>(fSettings, "Camera2.p2", found);
        }
    }
    else if (cameraType_ == Metashape)
    {
        bNeedToRectify_ = true;

        const float f = readParameter<float>(fSettings, "Camera2.f", found);
        const float cx = readParameter<float>(fSettings, "Camera2.cx", found);
        const float cy = readParameter<float>(fSettings, "Camera2.cy", found);
        const int width = readParameter<int>(fSettings, "Camera.width", found);
        const int height = readParameter<int>(fSettings, "Camera.height", found);
        const float b1 = readParameter<float>(fSettings, "Camera2.b1", found);
        const float b2 = readParameter<float>(fSettings, "Camera2.b2", found);

        const float k1 = readParameter<float>(fSettings, "Camera2.k1", found);
        const float k2 = readParameter<float>(fSettings, "Camera2.k2", found);
        const float k3 = readParameter<float>(fSettings, "Camera2.k3", found);
        const float k4 = readParameter<float>(fSettings, "Camera2.k4", found);
        const float p1 = readParameter<float>(fSettings, "Camera2.p1", found);
        const float p2 = readParameter<float>(fSettings, "Camera2.p2", found);

        const float fx = f + b1;
        const float fy = f;
        const float skew = b2;

        const float cx_abs = cx + 0.5f * static_cast<float>(width);
        const float cy_abs = cy + 0.5f * static_cast<float>(height);

        vCalibration = {fx, fy, cx_abs, cy_abs, k1, k2, k3, k4, p1, p2, skew};

        calibration2_ = new ORB_SLAM3::Metashape(vCalibration);
        originalCalib2_ = new ORB_SLAM3::Metashape(vCalibration);
    }

    //Load stereo extrinsic calibration
    if (cameraType_ == Rectified)
    {
        b_ = readParameter<float>(fSettings, "Stereo.b", found);
        bf_ = b_ * calibration1_->getParameter(0);
    }
    else
    {
        cv::Mat cvTlr = readParameter<cv::Mat>(fSettings, "Stereo.T_c1_c2", found);
        Tlr_ = Converter::toSophus(cvTlr);

        //TODO: also search for Trl and invert if necessary

        b_ = Tlr_.translation().norm();
        bf_ = b_ * calibration1_->getParameter(0);
    }

    thDepth_ = readParameter<float>(fSettings, "Stereo.ThDepth", found);
}

void Settings::readImageInfo(cv::FileStorage& fSettings)
{
    bool found;
    //Read original and desired image dimensions
    int originalRows = readParameter<int>(fSettings, "Camera.height", found);
    int originalCols = readParameter<int>(fSettings, "Camera.width", found);
    originalImSize_.width = originalCols;
    originalImSize_.height = originalRows;

    newImSize_ = originalImSize_;
    int newHeigh = readParameter<int>(fSettings, "Camera.newHeight", found, false);
    if (found)
    {
        bNeedToResize1_ = true;
        newImSize_.height = newHeigh;

        if (!bNeedToRectify_)
        {
            //Update calibration
            float scaleRowFactor = (float)newImSize_.height / (float)originalImSize_.height;
            calibration1_->setParameter(calibration1_->getParameter(1) * scaleRowFactor, 1);
            calibration1_->setParameter(calibration1_->getParameter(3) * scaleRowFactor, 3);

            if ((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) && cameraType_ != Rectified)
            {
                calibration2_->setParameter(calibration2_->getParameter(1) * scaleRowFactor, 1);
                calibration2_->setParameter(calibration2_->getParameter(3) * scaleRowFactor, 3);
            }
        }
    }

    int newWidth = readParameter<int>(fSettings, "Camera.newWidth", found, false);
    if (found)
    {
        bNeedToResize1_ = true;
        newImSize_.width = newWidth;

        if (!bNeedToRectify_)
        {
            //Update calibration
            float scaleColFactor = (float)newImSize_.width / (float)originalImSize_.width;
            calibration1_->setParameter(calibration1_->getParameter(0) * scaleColFactor, 0);
            calibration1_->setParameter(calibration1_->getParameter(2) * scaleColFactor, 2);
            if (cameraType_ == Metashape)
            {
                calibration1_->setParameter(calibration1_->getParameter(10) * scaleColFactor, 10);
            }

            if ((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) && cameraType_ != Rectified)
            {
                calibration2_->setParameter(calibration2_->getParameter(0) * scaleColFactor, 0);
                calibration2_->setParameter(calibration2_->getParameter(2) * scaleColFactor, 2);
                if (cameraType_ == Metashape)
                {
                    calibration2_->setParameter(calibration2_->getParameter(10) * scaleColFactor, 10);
                }

            }
        }
    }

    fps_ = readParameter<int>(fSettings, "Camera.fps", found);
    bRGB_ = (bool)readParameter<int>(fSettings, "Camera.RGB", found);
}

void Settings::readIMU(cv::FileStorage& fSettings)
{
    bool found;
    noiseGyro_ = readParameter<float>(fSettings, "IMU.NoiseGyro", found);
    noiseAcc_ = readParameter<float>(fSettings, "IMU.NoiseAcc", found);
    gyroWalk_ = readParameter<float>(fSettings, "IMU.GyroWalk", found);
    accWalk_ = readParameter<float>(fSettings, "IMU.AccWalk", found);
    imuFrequency_ = readParameter<float>(fSettings, "IMU.Frequency", found);

    cv::Mat cvTbc = readParameter<cv::Mat>(fSettings, "IMU.T_b_c1", found);
    Tbc_ = Converter::toSophus(cvTbc);

    readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false);
    if (found)
    {
        insertKFsWhenLost_ = (bool)readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false);
    }
    else
    {
        insertKFsWhenLost_ = true;
    }
}

void Settings::readORB(cv::FileStorage& fSettings)
{
    bool found;

    nFeatures_ = readParameter<int>(fSettings, "ORBextractor.nFeatures", found);
    nInitFeatures_ = readParameter<int>(fSettings, "ORBExtractor.nInitFeatures", found, false);
    if (!found)
    {
        nInitFeatures_ = static_cast<int>(2.5f * nFeatures_);
    }
    scaleFactor_ = readParameter<float>(fSettings, "ORBextractor.scaleFactor", found);
    nLevels_ = readParameter<int>(fSettings, "ORBextractor.nLevels", found);
    initThFAST_ = readParameter<int>(fSettings, "ORBextractor.iniThFAST", found);
    minThFAST_ = readParameter<int>(fSettings, "ORBextractor.minThFAST", found);
}

void Settings::readViewer(cv::FileStorage& fSettings)
{
    bool found;

    keyFrameSize_ = readParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
    keyFrameLineWidth_ = readParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
    graphLineWidth_ = readParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
    pointSize_ = readParameter<float>(fSettings, "Viewer.PointSize", found);
    cameraSize_ = readParameter<float>(fSettings, "Viewer.CameraSize", found);
    cameraLineWidth_ = readParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
    viewPointX_ = readParameter<float>(fSettings, "Viewer.ViewpointX", found);
    viewPointY_ = readParameter<float>(fSettings, "Viewer.ViewpointY", found);
    viewPointZ_ = readParameter<float>(fSettings, "Viewer.ViewpointZ", found);
    viewPointF_ = readParameter<float>(fSettings, "Viewer.ViewpointF", found);
    imageViewerScale_ = readParameter<float>(fSettings, "Viewer.imageViewScale", found, false);

    if (!found)
    {
        imageViewerScale_ = 1.0f;
    }
}

void Settings::readOtherParameters(cv::FileStorage& fSettings)
{
    bool found;

    thFarPoints_ = readParameter<float>(fSettings, "System.thFarPoints", found, false);

    monocularInitSearchWindowSize_ = readParameter<int>(fSettings, "MonocularInit.SearchWindowSize", found, false);
    if (!found)
    {
        monocularInitSearchWindowSize_ = 100;
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "[WARNING] MonocularInit.SearchWindowSize not found. Defaulting to 100." << std::endl;
    }

    monocularInitMinKeypoints_ = readParameter<int>(fSettings, "MonocularInit.MinKeypoints", found, false);
    if (!found)
    {
        monocularInitMinKeypoints_ = 100;
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "[WARNING] MonocularInit.MinKeypoints not found. Defaulting to 100." << std::endl;
    }

    monocularInitNNRatio_ = readParameter<float>(fSettings, "MonocularInit.NNRatio", found, false);
    if (!found)
    {
        monocularInitNNRatio_ = 0.9f;
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "[WARNING] MonocularInit.NNRatio not found. Defaulting to 0.9." << std::endl;
    }

    monocularInitMinMatches_ = readParameter<int>(fSettings, "MonocularInit.MinMatches", found, false);
    if (!found)
    {
        monocularInitMinMatches_ = 100;
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "[WARNING] MonocularInit.MinMatches not found. Defaulting to 100." << std::endl;
    }

    stereoInitMinKeypoints_ = readParameter<int>(fSettings, "Tracking.StereoInit.MinKeypoints", found, false);
    if (!found)
    {
        stereoInitMinKeypoints_ = 500;
    }

    referenceKeyframeNNRatio_ = readParameter<float>(fSettings, "Tracking.ReferenceKeyframe.NNRatio", found, false);
    if (!found)
    {
        referenceKeyframeNNRatio_ = 0.7f;
    }
    referenceKeyframeMinBoWMatches_ =
        readParameter<int>(fSettings, "Tracking.ReferenceKeyframe.MinBoWMatches", found, false);
    if (!found)
    {
        referenceKeyframeMinBoWMatches_ = 15;
    }
    referenceKeyframeMinOptimizedMapMatches_ =
        readParameter<int>(fSettings, "Tracking.ReferenceKeyframe.MinOptimizedMapMatches", found, false);
    if (!found)
    {
        referenceKeyframeMinOptimizedMapMatches_ = 10;
    }

    motionModelNNRatio_ = readParameter<float>(fSettings, "Tracking.MotionModel.NNRatio", found, false);
    if (!found)
    {
        motionModelNNRatio_ = 0.9f;
    }
    motionModelProjectionSearchThStereo_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.ProjectionSearchThStereo", found, false);
    if (!found)
    {
        motionModelProjectionSearchThStereo_ = 7;
    }
    motionModelProjectionSearchThMono_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.ProjectionSearchThMono", found, false);
    if (!found)
    {
        motionModelProjectionSearchThMono_ = 30;
    }
    motionModelMinInitialMatches_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.MinInitialMatches", found, false);
    if (!found)
    {
        motionModelMinInitialMatches_ = 20;
    }
    motionModelRetryProjectionSearchThStereo_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.RetryProjectionSearchThStereo", found, false);
    if (!found)
    {
        motionModelRetryProjectionSearchThStereo_ = 14;
    }
    motionModelRetryProjectionSearchThMono_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.RetryProjectionSearchThMono", found, false);
    if (!found)
    {
        motionModelRetryProjectionSearchThMono_ = 60;
    }
    motionModelMinRetryMatches_ = readParameter<int>(fSettings, "Tracking.MotionModel.MinRetryMatches", found, false);
    if (!found)
    {
        motionModelMinRetryMatches_ = 20;
    }
    motionModelMinOptimizedMapMatches_ =
        readParameter<int>(fSettings, "Tracking.MotionModel.MinOptimizedMapMatches", found, false);
    if (!found)
    {
        motionModelMinOptimizedMapMatches_ = 10;
    }

    localMapGenericMinInliers_ = readParameter<int>(fSettings, "Tracking.LocalMap.GenericMinInliers", found, false);
    if (!found)
    {
        localMapGenericMinInliers_ = 10;
    }
    localMapVisualMinInliers_ = readParameter<int>(fSettings, "Tracking.LocalMap.VisualMinInliers", found, false);
    if (!found)
    {
        localMapVisualMinInliers_ = 30;
    }

    newKFMinTrackedClosePoints_ = readParameter<int>(fSettings, "Tracking.NewKF.MinTrackedClosePoints", found, false);
    if (!found)
    {
        newKFMinTrackedClosePoints_ = 100;
    }
    newKFMinNonTrackedClosePoints_ =
        readParameter<int>(fSettings, "Tracking.NewKF.MinNonTrackedClosePoints", found, false);
    if (!found)
    {
        newKFMinNonTrackedClosePoints_ = 70;
    }
    newKFRefRatioMono_ = readParameter<float>(fSettings, "Tracking.NewKF.RefRatioMono", found, false);
    if (!found)
    {
        newKFRefRatioMono_ = 0.9f;
    }
    newKFRefRatioStereoFewKFs_ = readParameter<float>(fSettings, "Tracking.NewKF.RefRatioStereoFewKFs", found, false);
    if (!found)
    {
        newKFRefRatioStereoFewKFs_ = 0.4f;
    }
    newKFRefRatioStereo_ = readParameter<float>(fSettings, "Tracking.NewKF.RefRatioStereo", found, false);
    if (!found)
    {
        newKFRefRatioStereo_ = 0.75f;
    }
    newKFWeakTrackingRatio_ = readParameter<float>(fSettings, "Tracking.NewKF.WeakTrackingRatio", found, false);
    if (!found)
    {
        newKFWeakTrackingRatio_ = 0.25f;
    }
    newKFMinInliers_ = readParameter<int>(fSettings, "Tracking.NewKF.MinInliers", found, false);
    if (!found)
    {
        newKFMinInliers_ = 15;
    }
    newKFMaxKFsInQueue_ = readParameter<int>(fSettings, "Tracking.NewKF.MaxKFsInQueue", found, false);
    if (!found)
    {
        newKFMaxKFsInQueue_ = 3;
    }

    lostResetMinKFs_ = readParameter<int>(fSettings, "Tracking.LostResetMinKFs", found, false);
    if (!found)
    {
        lostResetMinKFs_ = 999999;
    }
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

    //Update relative pose between camera 1 and IMU if necessary
    if (sensor_ == System::IMU_STEREO)
    {
        Eigen::Matrix3f eigenR_r1_u1;
        cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
        Sophus::SE3f T_r1_u1(eigenR_r1_u1, Eigen::Vector3f::Zero());
        Tbc_ = Tbc_ * T_r1_u1.inverse();
    }
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

    if (settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO)
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
    if (settings.sensor_ == System::STEREO || settings.sensor_ == System::IMU_STEREO)
    {
        output << "\t-Stereo baseline: " << settings.b_ << std::endl;
        output << "\t-Stereo depth threshold : " << settings.thDepth_ << std::endl;

    }

    if (settings.sensor_ == System::IMU_MONOCULAR || settings.sensor_ == System::IMU_STEREO)
    {
        output << "\t-Gyro noise: " << settings.noiseGyro_ << std::endl;
        output << "\t-Accelerometer noise: " << settings.noiseAcc_ << std::endl;
        output << "\t-Gyro walk: " << settings.gyroWalk_ << std::endl;
        output << "\t-Accelerometer walk: " << settings.accWalk_ << std::endl;
        output << "\t-IMU frequency: " << settings.imuFrequency_ << std::endl;
    }

    output << "\t-Features per image: " << settings.nFeatures_ << std::endl;
    output << "\t-ORB scale factor: " << settings.scaleFactor_ << std::endl;
    output << "\t-ORB number of scales: " << settings.nLevels_ << std::endl;
    output << "\t-Initial FAST threshold: " << settings.initThFAST_ << std::endl;
    output << "\t-Min FAST threshold: " << settings.minThFAST_ << std::endl;

    return output;
}
};  // namespace ORB_SLAM3
