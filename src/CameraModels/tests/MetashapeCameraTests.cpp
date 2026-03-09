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
* You should have received a copy of the GNU General Public License along with ORB_SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "CameraModels/Metashape.h"

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

using namespace ORB_SLAM3;

namespace
{

TEST(MetashapeCamera, ProjectPinhole)
{
    const float fx = 500.f;
    const float fy = 480.f;
    const float cx = 320.f;
    const float cy = 240.f;
    const std::vector<float> params = {fx, fy, cx, cy, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    Metashape cam(params);

    const Eigen::Vector3d P(0.2, -0.1, 1.5);
    const Eigen::Vector2d uv = cam.project(P);
    const double ex = fx * P[0] / P[2] + cx;
    const double ey = fy * P[1] / P[2] + cy;
    EXPECT_NEAR(uv[0], ex, 1e-4) << "pinhole_u";
    EXPECT_NEAR(uv[1], ey, 1e-4) << "pinhole_v";
}

TEST(MetashapeCamera, ProjectWithSkew)
{
    const float fx = 600.f;
    const float fy = 580.f;
    const float cx = 300.f;
    const float cy = 260.f;
    const float skew = 25.f;
    const std::vector<float> params = {fx, fy, cx, cy, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, skew};
    Metashape cam(params);

    const Eigen::Vector3d P(0.15, 0.07, 1.2);
    const double x = P[0] / P[2];
    const double y = P[1] / P[2];
    const Eigen::Vector2d uv = cam.project(P);
    const double ex = fx * x + skew * y + cx;
    const double ey = fy * y + cy;
    EXPECT_NEAR(uv[0], ex, 1e-6) << "skew_u";
    EXPECT_NEAR(uv[1], ey, 1e-6) << "skew_v";
}

TEST(MetashapeCamera, UnprojectWithSkew)
{
    const float fx = 700.f;
    const float fy = 690.f;
    const float cx = 310.f;
    const float cy = 250.f;
    const float skew = 12.f;
    const std::vector<float> params = {fx, fy, cx, cy, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, skew};
    Metashape cam(params);

    const cv::Point2f pixel(640.f, 512.f);
    const cv::Point3f ray = cam.unproject(pixel);
    const double y = (pixel.y - cy) / fy;
    const double x = (pixel.x - cx - skew * y) / fx;
    EXPECT_NEAR(ray.x, x, 1e-6) << "unproject_x";
    EXPECT_NEAR(ray.y, y, 1e-6) << "unproject_y";
    EXPECT_NEAR(ray.z, 1.0, 1e-6) << "unproject_z";
}

TEST(MetashapeCamera, ProjectJacobian)
{
    const std::vector<float> params = {500.f, 510.f, 320.f, 240.f, -0.1f, 0.02f, -0.005f, 0.001f, 0.001f, -0.002f, 5.f};
    Metashape cam(params);

    const Eigen::Vector3d P(0.12, -0.08, 1.7);
    const Eigen::Matrix<double, 2, 3> J = cam.projectJac(P);

    const double eps = 1e-4;
    Eigen::Matrix<double, 2, 3> Jnum;
    for (int i = 0; i < 3; ++i)
    {
        Eigen::Vector3d Pp = P;
        Eigen::Vector3d Pm = P;
        Pp[i] += eps;
        Pm[i] -= eps;
        const Eigen::Vector2d up = cam.project(Pp);
        const Eigen::Vector2d um = cam.project(Pm);
        Jnum.col(i) = (up - um) / (2.0 * eps);
    }

    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 3; ++c)
            EXPECT_NEAR(J(r, c), Jnum(r, c), 1e-2) << "Jacobian (" << r << "," << c << ")";
}

}  // namespace
