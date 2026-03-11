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

#include "CameraModels/CameraCalibrationInput.h"
#include "CameraModels/GeometricCamera.h"
#include "CameraModels/Metashape.h"
#include "CameraModels/Pinhole.h"

namespace ORB_SLAM3
{

GeometricCamera* CreateMetashapeCamera(float f, float cx, float cy, int width, int height, float b1, float b2,
                                       float k1, float k2, float k3, float k4, float p1, float p2)
{
    const float fx = f + b1;
    const float fy = f;
    const float skew = b2;
    const float cx_abs = cx + 0.5f * static_cast<float>(width);
    const float cy_abs = cy + 0.5f * static_cast<float>(height);
    const std::vector<float> vCalibration = {fx, fy, cx_abs, cy_abs, k1, k2, k3, k4, p1, p2, skew};
    return new Metashape(vCalibration);
}

GeometricCamera* CreatePinholeCamera(float fx, float fy, float cx, float cy)
{
    const std::vector<float> vCalibration = {fx, fy, cx, cy};
    return new Pinhole(vCalibration);
}

}  // namespace ORB_SLAM3
