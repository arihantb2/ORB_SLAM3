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

#include "CameraModels/Metashape.h"

#include <boost/serialization/export.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::Metashape)

namespace ORB_SLAM3
{
namespace
{
inline bool CanUseOpenCvRadtan(const std::vector<float>& p)
{
    return p[7] == 0.f && p[10] == 0.f;
}

inline cv::Mat OpenCvDistCoeffs(const std::vector<float>& p)
{
    return (cv::Mat_<float>(5, 1) << p[4], p[5], p[8], p[9], p[6]);
}

inline void DistortPoint(const std::vector<float>& p, const float x, const float y, float& xd, float& yd)
{
    const float k1 = p[4];
    const float k2 = p[5];
    const float k3 = p[6];
    const float k4 = p[7];
    const float p1 = p[8];
    const float p2 = p[9];

    const float r2 = x * x + y * y;
    const float r4 = r2 * r2;
    const float r6 = r4 * r2;
    const float r8 = r4 * r4;
    const float radial = 1.f + k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8;

    const float x_rad = x * radial;
    const float y_rad = y * radial;

    const float x_tan = 2.f * p1 * x * y + p2 * (r2 + 2.f * x * x);
    const float y_tan = p1 * (r2 + 2.f * y * y) + 2.f * p2 * x * y;

    xd = x_rad + x_tan;
    yd = y_rad + y_tan;
}

inline cv::Point2f UndistortPixel(const std::vector<float>& p, const cv::Point2f& pt)
{
    const float fx = p[0];
    const float fy = p[1];
    const float cx = p[2];
    const float cy = p[3];
    const float s = p[10];

    if (CanUseOpenCvRadtan(p))
    {
        cv::Mat K = (cv::Mat_<float>(3, 3) << fx, s, cx, 0.f, fy, cy, 0.f, 0.f, 1.f);
        cv::Mat D = OpenCvDistCoeffs(p);
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = pt.x;
        mat.at<float>(0, 1) = pt.y;
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, K, D, cv::Mat(), K);
        mat = mat.reshape(1);
        return cv::Point2f(mat.at<float>(0, 0), mat.at<float>(0, 1));
    }

    const float yd = (pt.y - cy) / fy;
    const float xd = (pt.x - cx - s * yd) / fx;

    float x = xd;
    float y = yd;
    for (int i = 0; i < 10; ++i)
    {
        float x_dist = 0.f;
        float y_dist = 0.f;
        DistortPoint(p, x, y, x_dist, y_dist);
        const float dx = xd - x_dist;
        const float dy = yd - y_dist;
        x += dx;
        y += dy;
        if (dx * dx + dy * dy < 1e-12f)
            break;
    }

    return cv::Point2f(fx * x + s * y + cx, fy * y + cy);
}

inline Eigen::Vector2d UndistortPixel(const std::vector<float>& p, const Eigen::Vector2d& pt)
{
    const float fx = p[0];
    const float fy = p[1];
    const float cx = p[2];
    const float cy = p[3];
    const float s = p[10];

    const float yd = (pt[1] - cy) / fy;
    const float xd = (pt[0] - cx - s * yd) / fx;

    float x = xd;
    float y = yd;
    for (int i = 0; i < 10; ++i)
    {
        float x_dist = 0.f;
        float y_dist = 0.f;
        DistortPoint(p, x, y, x_dist, y_dist);
        const float dx = xd - x_dist;
        const float dy = yd - y_dist;
        x += dx;
        y += dy;
        if (dx * dx + dy * dy < 1e-12f)
            break;
    }

    return Eigen::Vector2d(fx * x + s * y + cx, fy * y + cy);
}
}  // namespace

cv::Point2f Metashape::project(const cv::Point3f& p3D)
{
    const float fx = mvParameters[0];
    const float fy = mvParameters[1];
    const float cx = mvParameters[2];
    const float cy = mvParameters[3];
    const float s = mvParameters[10];

    const float x = p3D.x / p3D.z;
    const float y = p3D.y / p3D.z;

    float xd = 0.f;
    float yd = 0.f;
    DistortPoint(mvParameters, x, y, xd, yd);

    return cv::Point2f(fx * xd + s * yd + cx, fy * yd + cy);
}

Eigen::Vector2d Metashape::project(const Eigen::Vector3d& v3D)
{
    const double fx = mvParameters[0];
    const double fy = mvParameters[1];
    const double cx = mvParameters[2];
    const double cy = mvParameters[3];
    const double s = mvParameters[10];

    const double x = v3D[0] / v3D[2];
    const double y = v3D[1] / v3D[2];

    float xd = 0.f;
    float yd = 0.f;
    DistortPoint(mvParameters, static_cast<float>(x), static_cast<float>(y), xd, yd);

    Eigen::Vector2d res;
    res[0] = fx * xd + s * yd + cx;
    res[1] = fy * yd + cy;

    return res;
}

Eigen::Vector2f Metashape::project(const Eigen::Vector3f& v3D)
{
    const float fx = mvParameters[0];
    const float fy = mvParameters[1];
    const float cx = mvParameters[2];
    const float cy = mvParameters[3];
    const float s = mvParameters[10];

    const float x = v3D[0] / v3D[2];
    const float y = v3D[1] / v3D[2];

    float xd = 0.f;
    float yd = 0.f;
    DistortPoint(mvParameters, x, y, xd, yd);

    Eigen::Vector2f res;
    res[0] = fx * xd + s * yd + cx;
    res[1] = fy * yd + cy;

    return res;
}

Eigen::Vector2f Metashape::projectMat(const cv::Point3f& p3D)
{
    cv::Point2f point = this->project(p3D);
    return Eigen::Vector2f(point.x, point.y);
}

float Metashape::uncertainty2(const Eigen::Matrix<double, 2, 1>& p2D)
{
    return 1.f;
}

Eigen::Vector3f Metashape::unprojectEig(const cv::Point2f& p2D)
{
    cv::Point3f ray = this->unproject(p2D);
    return Eigen::Vector3f(ray.x, ray.y, ray.z);
}

cv::Point3f Metashape::unproject(const cv::Point2f& p2D)
{
    const float fx = mvParameters[0];
    const float fy = mvParameters[1];
    const float cx = mvParameters[2];
    const float cy = mvParameters[3];
    const float s = mvParameters[10];

    if (CanUseOpenCvRadtan(mvParameters))
    {
        cv::Mat K = (cv::Mat_<float>(3, 3) << fx, s, cx, 0.f, fy, cy, 0.f, 0.f, 1.f);
        cv::Mat D = OpenCvDistCoeffs(mvParameters);
        cv::Mat mat(1, 2, CV_32F);
        mat.at<float>(0, 0) = p2D.x;
        mat.at<float>(0, 1) = p2D.y;
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, K, D);
        mat = mat.reshape(1);
        return cv::Point3f(mat.at<float>(0, 0), mat.at<float>(0, 1), 1.f);
    }

    const float yd = (p2D.y - cy) / fy;
    const float xd = (p2D.x - cx - s * yd) / fx;

    float x = xd;
    float y = yd;
    for (int i = 0; i < 10; ++i)
    {
        float x_dist = 0.f;
        float y_dist = 0.f;
        DistortPoint(mvParameters, x, y, x_dist, y_dist);
        const float dx = xd - x_dist;
        const float dy = yd - y_dist;
        x += dx;
        y += dy;
        if (dx * dx + dy * dy < 1e-12f)
            break;
    }

    return cv::Point3f(x, y, 1.f);
}

// Jacobian derived symbolically with SymPy (python3 + sympy).
// Model:
//   x = X/Z, y = Y/Z, r2 = x^2 + y^2
//   radial = 1 + k1*r2 + k2*r2^2 + k3*r2^3 + k4*r2^4
//   x_d = x*radial + 2*p1*x*y + p2*(r2 + 2*x^2)
//   y_d = y*radial + p1*(r2 + 2*y^2) + 2*p2*x*y
//   u = fx*x_d + s*y_d + cx, v = fy*y_d + cy
//
// Jacobian entries in terms of x,y,z (z=Z):
//   dr = k1 + 2*k2*r2 + 3*k3*r2^2 + 4*k4*r2^3
//   term_xy = p1*x + p2*y + x*y*dr
//   term_x = radial + 2*x^2*dr + 2*p1*y + 6*p2*x
//   term_y = radial + 2*y^2*dr + 6*p1*y + 2*p2*x
//   du/dX = (fx*term_x + 2*s*term_xy)/z
//   du/dY = (2*fx*term_xy + s*term_y)/z
//   du/dZ = -(fx*(4*p1*x*y + 2*p2*(3*x^2 + y^2) + 2*x*r2*dr + x*radial)
//            + s*(2*p1*(x^2 + 3*y^2) + 4*p2*x*y + 2*y*r2*dr + y*radial))/z
//   dv/dX = (2*fy*term_xy)/z
//   dv/dY = (fy*term_y)/z
//   dv/dZ = -(fy*(2*p1*(x^2 + 3*y^2) + 4*p2*x*y + 2*y*r2*dr + y*radial))/z
Eigen::Matrix<double, 2, 3> Metashape::projectJac(const Eigen::Vector3d& v3D)
{
    const double fx = mvParameters[0];
    const double fy = mvParameters[1];
    const double s = mvParameters[10];

    const double z = v3D[2];
    const double x = v3D[0] / z;
    const double y = v3D[1] / z;

    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double r8 = r4 * r4;

    const double k1 = mvParameters[4];
    const double k2 = mvParameters[5];
    const double k3 = mvParameters[6];
    const double k4 = mvParameters[7];
    const double p1 = mvParameters[8];
    const double p2 = mvParameters[9];

    const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8;
    const double dr = k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4 + 4.0 * k4 * r6;

    const double term_xy = p1 * x + p2 * y + x * y * dr;
    const double term_x = radial + 2.0 * x * x * dr + 2.0 * p1 * y + 6.0 * p2 * x;
    const double term_y = radial + 2.0 * y * y * dr + 6.0 * p1 * y + 2.0 * p2 * x;

    const double du_dX = (fx * term_x + 2.0 * s * term_xy) / z;
    const double du_dY = (2.0 * fx * term_xy + s * term_y) / z;
    const double du_dZ = -(fx * (4.0 * p1 * x * y + 2.0 * p2 * (3.0 * x * x + y * y) + 2.0 * x * r2 * dr + x * radial) +
                           s * (2.0 * p1 * (x * x + 3.0 * y * y) + 4.0 * p2 * x * y + 2.0 * y * r2 * dr + y * radial)) /
                         z;
    const double dv_dX = (2.0 * fy * term_xy) / z;
    const double dv_dY = (fy * term_y) / z;
    const double dv_dZ =
        -fy * (2.0 * p1 * (x * x + 3.0 * y * y) + 4.0 * p2 * x * y + 2.0 * y * r2 * dr + y * radial) / z;

    Eigen::Matrix<double, 2, 3> Jac;
    Jac(0, 0) = du_dX;
    Jac(0, 1) = du_dY;
    Jac(0, 2) = du_dZ;
    Jac(1, 0) = dv_dX;
    Jac(1, 1) = dv_dY;
    Jac(1, 2) = dv_dZ;

    return Jac;
}

bool Metashape::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1,
                                        const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int>& vMatches12,
                                        Sophus::SE3f& T21, std::vector<cv::Point3f>& vP3D,
                                        std::vector<bool>& vbTriangulated)
{
    if (!tvr)
    {
        Eigen::Matrix3f K = this->toK_();
        tvr = new TwoViewReconstruction(K);
    }

    std::vector<cv::KeyPoint> vKeysUn1 = vKeys1;
    std::vector<cv::KeyPoint> vKeysUn2 = vKeys2;

    if (CanUseOpenCvRadtan(mvParameters))
    {
        cv::Mat K = this->toK();
        cv::Mat D = OpenCvDistCoeffs(mvParameters);
        cv::Mat pts1(static_cast<int>(vKeys1.size()), 2, CV_32F);
        cv::Mat pts2(static_cast<int>(vKeys2.size()), 2, CV_32F);
        for (size_t i = 0; i < vKeys1.size(); ++i)
        {
            pts1.at<float>(static_cast<int>(i), 0) = vKeys1[i].pt.x;
            pts1.at<float>(static_cast<int>(i), 1) = vKeys1[i].pt.y;
        }
        for (size_t i = 0; i < vKeys2.size(); ++i)
        {
            pts2.at<float>(static_cast<int>(i), 0) = vKeys2[i].pt.x;
            pts2.at<float>(static_cast<int>(i), 1) = vKeys2[i].pt.y;
        }
        pts1 = pts1.reshape(2);
        pts2 = pts2.reshape(2);
        cv::undistortPoints(pts1, pts1, K, D, cv::Mat(), K);
        cv::undistortPoints(pts2, pts2, K, D, cv::Mat(), K);
        pts1 = pts1.reshape(1);
        pts2 = pts2.reshape(1);
        for (size_t i = 0; i < vKeys1.size(); ++i)
        {
            vKeysUn1[i].pt.x = pts1.at<float>(static_cast<int>(i), 0);
            vKeysUn1[i].pt.y = pts1.at<float>(static_cast<int>(i), 1);
        }
        for (size_t i = 0; i < vKeys2.size(); ++i)
        {
            vKeysUn2[i].pt.x = pts2.at<float>(static_cast<int>(i), 0);
            vKeysUn2[i].pt.y = pts2.at<float>(static_cast<int>(i), 1);
        }
    }
    else
    {
        for (size_t i = 0; i < vKeys1.size(); ++i)
        {
            vKeysUn1[i].pt = UndistortPixel(mvParameters, vKeys1[i].pt);
        }
        for (size_t i = 0; i < vKeys2.size(); ++i)
        {
            vKeysUn2[i].pt = UndistortPixel(mvParameters, vKeys2[i].pt);
        }
    }

    return tvr->Reconstruct(vKeysUn1, vKeysUn2, vMatches12, T21, vP3D, vbTriangulated);
}

cv::Mat Metashape::toK()
{
    const float fx = mvParameters[0];
    const float fy = mvParameters[1];
    const float cx = mvParameters[2];
    const float cy = mvParameters[3];
    const float s = mvParameters[10];
    cv::Mat K = (cv::Mat_<float>(3, 3) << fx, s, cx, 0.f, fy, cy, 0.f, 0.f, 1.f);
    return K;
}

Eigen::Matrix3f Metashape::toK_()
{
    const float fx = mvParameters[0];
    const float fy = mvParameters[1];
    const float cx = mvParameters[2];
    const float cy = mvParameters[3];
    const float s = mvParameters[10];
    Eigen::Matrix3f K;
    K << fx, s, cx, 0.f, fy, cy, 0.f, 0.f, 1.f;
    return K;
}

bool Metashape::epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                                  const Eigen::Matrix3f& R12, const Eigen::Vector3f& t12, const float sigmaLevel,
                                  const float unc)
{
    cv::Point2f kp1u;
    cv::Point2f kp2u;
    if (CanUseOpenCvRadtan(mvParameters))
    {
        cv::Mat K = this->toK();
        cv::Mat D = OpenCvDistCoeffs(mvParameters);
        cv::Mat mat(2, 2, CV_32F);
        mat.at<float>(0, 0) = kp1.pt.x;
        mat.at<float>(0, 1) = kp1.pt.y;
        mat.at<float>(1, 0) = kp2.pt.x;
        mat.at<float>(1, 1) = kp2.pt.y;
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, K, D, cv::Mat(), K);
        mat = mat.reshape(1);
        kp1u.x = mat.at<float>(0, 0);
        kp1u.y = mat.at<float>(0, 1);
        kp2u.x = mat.at<float>(1, 0);
        kp2u.y = mat.at<float>(1, 1);
    }
    else
    {
        kp1u = UndistortPixel(mvParameters, kp1.pt);
        kp2u = UndistortPixel(mvParameters, kp2.pt);
    }

    //Compute Fundamental Matrix
    Eigen::Matrix3f t12x = Sophus::SO3f::hat(t12);
    Eigen::Matrix3f K1 = this->toK_();
    Eigen::Matrix3f K2 = pCamera2->toK_();
    Eigen::Matrix3f F12 = K1.transpose().inverse() * t12x * R12 * K2.inverse();

    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1u.x * F12(0, 0) + kp1u.y * F12(1, 0) + F12(2, 0);
    const float b = kp1u.x * F12(0, 1) + kp1u.y * F12(1, 1) + F12(2, 1);
    const float c = kp1u.x * F12(0, 2) + kp1u.y * F12(1, 2) + F12(2, 2);

    const float num = a * kp2u.x + b * kp2u.y + c;

    const float den = a * a + b * b;

    if (den == 0)
        return false;

    const float dsqr = num * num / den;

    return dsqr < 3.84 * unc;
}

std::ostream& operator<<(std::ostream& os, const Metashape& ms)
{
    for (size_t i = 0; i < ms.mvParameters.size(); ++i)
    {
        if (i != 0)
            os << " ";
        os << ms.mvParameters[i];
    }
    return os;
}

std::istream& operator>>(std::istream& is, Metashape& ms)
{
    float nextParam;
    for (size_t i = 0; i < 11; i++)
    {
        assert(is.good());  //Make sure the input stream is good
        is >> nextParam;
        ms.mvParameters[i] = nextParam;
    }
    return is;
}

bool Metashape::IsEqual(GeometricCamera* pCam)
{
    if (pCam->GetType() != GetType())
        return false;

    Metashape* pMSCam = (Metashape*)pCam;

    if (size() != pMSCam->size())
        return false;

    bool is_same_camera = true;
    for (size_t i = 0; i < size(); ++i)
    {
        if (abs(mvParameters[i] - pMSCam->getParameter(i)) > 1e-6)
        {
            is_same_camera = false;
            break;
        }
    }
    return is_same_camera;
}

}  // namespace ORB_SLAM3
