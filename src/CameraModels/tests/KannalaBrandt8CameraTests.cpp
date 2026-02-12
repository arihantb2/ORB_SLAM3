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

#include "CameraModels/KannalaBrandt8.h"

#include "Verbose.h"

#include <cassert>
#include <cmath>
#include <vector>

using namespace ORB_SLAM3;

namespace
{
bool NearlyEqual(const double a, const double b, const double tol)
{
    return std::abs(a - b) <= tol;
}

void ExpectNear(const double a, const double b, const double tol, const char* label)
{
    if (!NearlyEqual(a, b, tol))
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL) << label << " mismatch: " << a << " vs " << b << std::endl;
        assert(false);
    }
}
}  // namespace

int main()
{
    {
        const float fx = 500.f;
        const float fy = 510.f;
        const float cx = 320.f;
        const float cy = 240.f;
        const std::vector<float> params = {fx, fy, cx, cy, 0.f, 0.f, 0.f, 0.f};
        ORB_SLAM3::KannalaBrandt8 cam(params);

        const Eigen::Vector3d P(0.2, -0.1, 1.5);
        const Eigen::Vector2d uv = cam.project(P);
        const double x2y2 = P[0] * P[0] + P[1] * P[1];
        const double theta = std::atan2(std::sqrt(x2y2), P[2]);
        const double psi = std::atan2(P[1], P[0]);
        const double ex = fx * theta * std::cos(psi) + cx;
        const double ey = fy * theta * std::sin(psi) + cy;
        ExpectNear(uv[0], ex, 1e-6, "kb_project_u");
        ExpectNear(uv[1], ey, 1e-6, "kb_project_v");
    }

    {
        const std::vector<float> params = {520.f, 530.f, 310.f, 245.f, 0.01f, -0.001f, 0.0001f, -0.00001f};
        ORB_SLAM3::KannalaBrandt8 cam(params);

        const Eigen::Vector3f P(0.15f, -0.08f, 1.2f);
        const Eigen::Vector2f uv = cam.project(P);
        const cv::Point2f p2d(uv[0], uv[1]);
        const cv::Point3f ray = cam.unproject(p2d);
        const Eigen::Vector2f uv_back = cam.project(Eigen::Vector3f(ray.x, ray.y, ray.z));
        ExpectNear(uv_back[0], uv[0], 1e-3, "kb_roundtrip_u");
        ExpectNear(uv_back[1], uv[1], 1e-3, "kb_roundtrip_v");
    }

    {
        const std::vector<float> params = {480.f, 490.f, 300.f, 230.f, 0.02f, -0.003f, 0.0002f, -0.00002f};
        ORB_SLAM3::KannalaBrandt8 cam(params);

        const Eigen::Vector3d P(0.12, -0.07, 1.6);
        const Eigen::Matrix<double, 2, 3> J = cam.projectJac(P);

        const double eps = 1e-6;
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
        {
            for (int c = 0; c < 3; ++c)
            {
                const double diff = std::abs(J(r, c) - Jnum(r, c));
                if (diff > 1e-4)
                {
                    Verbose::Print(Verbose::VERBOSITY_NORMAL) << "Jacobian mismatch (" << r << "," << c
                                                              << "): " << J(r, c) << " vs " << Jnum(r, c) << std::endl;
                    assert(false);
                }
            }
        }
    }

    return 0;
}
