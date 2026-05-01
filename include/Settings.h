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

#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H

#include <stdlib.h>
#include <unistd.h>
#include <ostream>
#include <string>
#include <vector>

#include "Verbose.h"

#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

#include "CameraModels/CameraCalibrationInput.h"

namespace ORB_SLAM3
{

class System;
class GeometricCamera;

//TODO: change to double instead of float

class Settings
{
public:
    /*
         * Enum for the different camera types implemented
         */
    enum CameraType
    {
        PinHole = 0,
        Rectified = 1,
        Metashape = 3
    };

    /*
         * Delete default constructor
         */
    Settings() = delete;

    /*
         * Constructor from algorithm config file and injected camera calibration.
         * Opens only the algorithm config; all camera data comes from calib.
         */
    Settings(const std::string& algorithmConfigPath, const int& sensor, const CameraCalibrationInput& calib);

    /*
         * Ostream operator overloading to dump settings to the terminal
         */
    friend std::ostream& operator<<(std::ostream& output, const Settings& s);

    /*
         * Getter methods
         */
    CameraType cameraType() { return cameraType_; }
    GeometricCamera* camera1() { return calibration1_; }
    GeometricCamera* camera2() { return calibration2_; }
    cv::Mat camera1DistortionCoef()
    {
        return cv::Mat(vPinHoleDistorsion1_.size(), 1, CV_32F, vPinHoleDistorsion1_.data());
    }
    cv::Mat camera2DistortionCoef()
    {
        return cv::Mat(vPinHoleDistorsion2_.size(), 1, CV_32F, vPinHoleDistorsion2_.data());
    }

    Sophus::SE3f Tlr() { return Tlr_; }
    float bf() { return bf_; }
    float b() { return b_; }
    float thDepth() { return thDepth_; }

    bool needToUndistort() { return bNeedToUndistort_; }

    cv::Size newImSize() { return newImSize_; }
    float fps() { return fps_; }
    bool rgb() { return bRGB_; }
    bool needToResize() { return bNeedToResize1_; }
    bool needToRectify() { return bNeedToRectify_; }

    float depthMapFactor() { return depthMapFactor_; }

    std::string featureExtractorType() { return featureExtractorType_; }
    int nFeatures() { return nFeatures_; }
    int nInitFeatures() { return nInitFeatures_; }
    int nLevels() { return nLevels_; }
    float scaleFactor() { return scaleFactor_; }
    // ORB-specific
    float initThFAST() { return initThFAST_; }
    float minThFAST() { return minThFAST_; }
    int orbScoreType() { return orbScoreType_; }  // cv::ORB::HARRIS_SCORE or cv::ORB::FAST_SCORE
    // BRISK-specific
    int briskThreshold() { return briskThreshold_; }
    // Matching thresholds (descriptor-type-specific defaults set by each readXXX())
    int matchThLow()  { return matchThLow_; }
    int matchThHigh() { return matchThHigh_; }
    float thFarPoints() { return thFarPoints_; }
    float localMappingOptimizeEveryTSeconds() { return localMappingOptimizeEveryTSeconds_; }
    int localMappingMinKeyframesForLBA() { return localMappingMinKeyframesForLBA_; }
    int localMappingMPCullingMinObs() { return localMappingMPCullingMinObs_; }
    int localMappingMPCullingMinKFAgeForObsCheck() { return localMappingMPCullingMinKFAgeForObsCheck_; }
    int localMappingMPCullingMaxKFAgeInRecent() { return localMappingMPCullingMaxKFAgeInRecent_; }
    float localMappingMPCullingMinFoundRatio() { return localMappingMPCullingMinFoundRatio_; }
    int localMappingCreateNewMapPointsCovisibility() { return localMappingCreateNewMapPointsCovisibility_; }
    float localMappingCreateNewMapPointsMatchRatio() { return localMappingCreateNewMapPointsMatchRatio_; }
    float localMappingCreateNewMapPointsMinBaselineDepthRatio()
    {
        return localMappingCreateNewMapPointsMinBaselineDepthRatio_;
    }
    float localMappingCreateNewMapPointsMaxCosParallax() { return localMappingCreateNewMapPointsMaxCosParallax_; }
    float localMappingCreateNewMapPointsScaleConsistencyFactor()
    {
        return localMappingCreateNewMapPointsScaleConsistencyFactor_;
    }
    int localMappingSearchInNeighborsNumNeighborKFs() { return localMappingSearchInNeighborsNumNeighborKFs_; }
    int localMappingSearchInNeighborsNumSecondNeighbors() { return localMappingSearchInNeighborsNumSecondNeighbors_; }
    int localMappingSearchInNeighborsMaxTemporalNeighbors()
    {
        return localMappingSearchInNeighborsMaxTemporalNeighbors_;
    }
    float localMappingKeyFrameCullingRedundantRatio() { return localMappingKeyFrameCullingRedundantRatio_; }
    int localMappingKeyFrameCullingMinObsInOthers() { return localMappingKeyFrameCullingMinObsInOthers_; }
    int localMappingKeyFrameCullingMaxKeyframesToCheck() { return localMappingKeyFrameCullingMaxKeyframesToCheck_; }
    int localMappingKeyFrameCullingEarlyExitAfterAbort() { return localMappingKeyFrameCullingEarlyExitAfterAbort_; }
    int monocularInitSearchWindowSize() { return monocularInitSearchWindowSize_; }
    int monocularInitMinKeypoints() { return monocularInitMinKeypoints_; }
    float monocularInitNNRatio() { return monocularInitNNRatio_; }
    int monocularInitMinMatches() { return monocularInitMinMatches_; }

    int stereoInitMinKeypoints() { return stereoInitMinKeypoints_; }
    int stereoInitMinMapPoints() { return stereoInitMinMapPoints_; }
    float referenceKeyframeNNRatio() { return referenceKeyframeNNRatio_; }
    int referenceKeyframeMinBoWMatches() { return referenceKeyframeMinBoWMatches_; }
    int referenceKeyframeMinOptimizedMapMatches() { return referenceKeyframeMinOptimizedMapMatches_; }
    float motionModelNNRatio() { return motionModelNNRatio_; }
    int motionModelProjectionSearchTh() { return motionModelProjectionSearchTh_; }
    int motionModelMinInitialMatches() { return motionModelMinInitialMatches_; }
    int motionModelRetryProjectionSearchTh() { return motionModelRetryProjectionSearchTh_; }
    int motionModelMinRetryMatches() { return motionModelMinRetryMatches_; }
    int motionModelMinOptimizedMapMatches() { return motionModelMinOptimizedMapMatches_; }
    int localMapGenericMinInliers() { return localMapGenericMinInliers_; }
    int localMapVisualMinInliers() { return localMapVisualMinInliers_; }

    int newKFMinTrackedClosePoints() { return newKFMinTrackedClosePoints_; }
    int newKFMinNonTrackedClosePoints() { return newKFMinNonTrackedClosePoints_; }
    float newKFRefRatioMono() { return newKFRefRatioMono_; }
    float newKFRefRatioStereoFewKFs() { return newKFRefRatioStereoFewKFs_; }
    float newKFRefRatioStereo() { return newKFRefRatioStereo_; }
    float newKFWeakTrackingRatio() { return newKFWeakTrackingRatio_; }
    int newKFMinInliers() { return newKFMinInliers_; }
    int newKFMaxKFsInQueue() { return newKFMaxKFsInQueue_; }
    bool forceEveryFrameKeyframe() { return forceEveryFrameKeyframe_; }

    int lostResetMinKFs() { return lostResetMinKFs_; }

    cv::Mat M1l() { return M1l_; }
    cv::Mat M2l() { return M2l_; }
    cv::Mat M1r() { return M1r_; }
    cv::Mat M2r() { return M2r_; }

private:
    template <typename T>
    T readParameter(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required = true)
    {
        return readParameter<T>(fSettings, name, found, T(), required);
    }

    template <typename T>
    T readParameter(cv::FileStorage& fSettings, const std::string& name, bool& found, const T& default_value,
                    const bool required = true)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "[WARNING] " << name << " required parameter does not exist. Using default value." << std::endl;
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << name << " optional parameter does not exist..." << std::endl;
            }

            found = false;
            return default_value;
        }
        else
        {
            found = true;
            return (T)node;
        }
    }

    void readGridORB(cv::FileStorage& fSettings);
    void readORB(cv::FileStorage& fSettings);
    void readBRISK(cv::FileStorage& fSettings);
    void readOtherParameters(cv::FileStorage& fSettings);

    void precomputeRectificationMaps();

    int sensor_;
    CameraType cameraType_;  //Camera type

    /*
         * Visual stuff
         */
    GeometricCamera *calibration1_, *calibration2_;  //Camera calibration
    GeometricCamera *originalCalib1_, *originalCalib2_;
    std::vector<float> vPinHoleDistorsion1_, vPinHoleDistorsion2_;

    cv::Size originalImSize_, newImSize_;
    float fps_;
    bool bRGB_;

    bool bNeedToUndistort_;
    bool bNeedToRectify_;
    bool bNeedToResize1_, bNeedToResize2_;

    Sophus::SE3f Tlr_;
    float thDepth_;
    float bf_, b_;

    /*
         * Rectification stuff
         */
    cv::Mat M1l_, M2l_;
    cv::Mat M1r_, M2r_;

    float depthMapFactor_;

    /*
         * Feature extractor stuff (shared)
         */
    std::string featureExtractorType_;
    int nFeatures_;
    int nInitFeatures_;
    float scaleFactor_;
    int nLevels_;
    // ORB-specific
    int initThFAST_, minThFAST_;
    int orbScoreType_;  // cv::ORB::HARRIS_SCORE or cv::ORB::FAST_SCORE (VanillaORB only)
    // BRISK-specific
    int briskThreshold_ = 30;
    // Matching thresholds (set per extractor type in readXXX())
    int matchThLow_  = 50;
    int matchThHigh_ = 100;
    /*
         * Other stuff
         */
    float thFarPoints_;
    float localMappingOptimizeEveryTSeconds_;
    int localMappingMinKeyframesForLBA_;
    int localMappingMPCullingMinObs_;
    int localMappingMPCullingMinKFAgeForObsCheck_;
    int localMappingMPCullingMaxKFAgeInRecent_;
    float localMappingMPCullingMinFoundRatio_;
    int localMappingCreateNewMapPointsCovisibility_;
    float localMappingCreateNewMapPointsMatchRatio_;
    float localMappingCreateNewMapPointsMinBaselineDepthRatio_;
    float localMappingCreateNewMapPointsMaxCosParallax_;
    float localMappingCreateNewMapPointsScaleConsistencyFactor_;
    int localMappingSearchInNeighborsNumNeighborKFs_;
    int localMappingSearchInNeighborsNumSecondNeighbors_;
    int localMappingSearchInNeighborsMaxTemporalNeighbors_;
    float localMappingKeyFrameCullingRedundantRatio_;
    int localMappingKeyFrameCullingMinObsInOthers_;
    int localMappingKeyFrameCullingMaxKeyframesToCheck_;
    int localMappingKeyFrameCullingEarlyExitAfterAbort_;
    int monocularInitSearchWindowSize_;
    int monocularInitMinKeypoints_;
    float monocularInitNNRatio_;
    int monocularInitMinMatches_;

    int stereoInitMinKeypoints_;
    int stereoInitMinMapPoints_;
    float referenceKeyframeNNRatio_;
    int referenceKeyframeMinBoWMatches_;
    int referenceKeyframeMinOptimizedMapMatches_;
    float motionModelNNRatio_;
    int motionModelProjectionSearchTh_;
    int motionModelMinInitialMatches_;
    int motionModelRetryProjectionSearchTh_;
    int motionModelMinRetryMatches_;
    int motionModelMinOptimizedMapMatches_;
    int localMapGenericMinInliers_;
    int localMapVisualMinInliers_;

    int newKFMinTrackedClosePoints_;
    int newKFMinNonTrackedClosePoints_;
    float newKFRefRatioMono_;
    float newKFRefRatioStereoFewKFs_;
    float newKFRefRatioStereo_;
    float newKFWeakTrackingRatio_;
    int newKFMinInliers_;
    int newKFMaxKFsInQueue_;
    bool forceEveryFrameKeyframe_ = false;

    int lostResetMinKFs_;
};
};  // namespace ORB_SLAM3

#endif  //ORB_SLAM3_SETTINGS_H
