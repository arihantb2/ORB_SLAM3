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

#ifndef ORB_SLAM3_CAMERACALIBRATIONINPUT_H
#define ORB_SLAM3_CAMERACALIBRATIONINPUT_H

#include <memory>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <vector>

#include "CameraModels/GeometricCamera.h"

namespace ORB_SLAM3
{

/**
 * Calibration input for injection into System/Settings.
 * Owns the camera objects; Settings stores raw pointers from .get() and must not outlive this struct.
 * cameraType: 0 = PinHole, 1 = Rectified, 3 = Metashape (matches Settings::CameraType).
 */
struct CameraCalibrationInput
{
    std::unique_ptr<GeometricCamera> camera1;
    std::unique_ptr<GeometricCamera> camera2;  // null for monocular
    cv::Size originalImSize;
    cv::Size newImSize;
    int cameraType = 0;    // Settings::CameraType: PinHole=0, Rectified=1, Metashape=3
    Sophus::SE3f T_c1_c2;  // stereo only
    float thDepth = 0.f;   // stereo only
    bool bNeedToRectify = false;
    bool bNeedToUndistort = false;
    bool bNeedToResize1 = false;
    std::vector<float> vPinHoleDistorsion1;
    std::vector<float> vPinHoleDistorsion2;
};

/**
 * Build a Metashape camera from parameters. Caller owns the returned pointer.
 * cx, cy are offsets from image center; width, height are image dimensions.
 */
GeometricCamera* CreateMetashapeCamera(float f, float cx, float cy, int width, int height, float b1, float b2, float k1,
                                       float k2, float k3, float k4, float p1, float p2);

/**
 * Build a Pinhole camera from parameters. Caller owns the returned pointer.
 */
GeometricCamera* CreatePinholeCamera(float fx, float fy, float cx, float cy);

}  // namespace ORB_SLAM3

#endif  // ORB_SLAM3_CAMERACALIBRATIONINPUT_H
