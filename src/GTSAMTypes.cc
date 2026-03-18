/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "GTSAMTypes.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <sophus/so3.hpp>

#include <cassert>
#include <cmath>

namespace ORB_SLAM3
{

// ─────────────────────────────────────────────────────────────────────────────
// SO(3) utilities  (formerly in G2oTypes.cc)
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w)
{
    return ExpSO3(w[0], w[1], w[2]);
}

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z)
{
    const double d2 = x * x + y * y + z * z;
    const double d = std::sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5)
    {
        const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
        return NormalizeRotation(R);
    }
    else
    {
        const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + W * std::sin(d) / d + W * W * (1.0 - std::cos(d)) / d2;
        return NormalizeRotation(R);
    }
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R)
{
    const double tr = R(0, 0) + R(1, 1) + R(2, 2);
    Eigen::Vector3d w;
    w << (R(2, 1) - R(1, 2)) / 2.0, (R(0, 2) - R(2, 0)) / 2.0, (R(1, 0) - R(0, 1)) / 2.0;
    const double costheta = (tr - 1.0) * 0.5;
    if (costheta > 1.0 || costheta < -1.0)
    {
        return w;
    }
    const double theta = std::acos(costheta);
    const double s = std::sin(theta);
    if (std::fabs(s) < 1e-5)
    {
        return w;
    }
    return theta * w / s;
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d S;
    S << 0.0, -w(2), w(1), w(2), 0.0, -w(0), -w(1), w(0), 0.0;
    return S;
}

Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x * x + y * y + z * z;
    const double d = std::sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5)
    {
        return Eigen::Matrix3d::Identity();
    }
    return Eigen::Matrix3d::Identity() - W * (1.0 - std::cos(d)) / d2 + W * W * (d - std::sin(d)) / (d2 * d);
}

Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v)
{
    return RightJacobianSO3(v[0], v[1], v[2]);
}

Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x * x + y * y + z * z;
    const double d = std::sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5)
    {
        return Eigen::Matrix3d::Identity();
    }
    return Eigen::Matrix3d::Identity() + W / 2.0 + W * W * (1.0 / d2 - (1.0 + std::cos(d)) / (2.0 * d * std::sin(d)));
}

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d& v)
{
    return InverseRightJacobianSO3(v[0], v[1], v[2]);
}

// ─────────────────────────────────────────────────────────────────────────────
// Conversion helpers
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Pose3 sophusToGTSAMPose(const Sophus::SE3f& T)
{
    const Eigen::Matrix3d R = T.rotationMatrix().cast<double>();
    const Eigen::Vector3d t = T.translation().cast<double>();
    return gtsam::Pose3(gtsam::Rot3(R), t);
}

Sophus::SE3f gtsamToSophusPose(const gtsam::Pose3& P)
{
    Eigen::Matrix3f R = P.rotation().matrix().cast<float>();
    R = NormalizeRotation(R);
    const Eigen::Vector3f t = P.translation().cast<float>();
    return Sophus::SE3f(R, t);
}

gtsam::Similarity3 toGTSAMSim3(const Sophus::Sim3f& S)
{
    const double scale = static_cast<double>(S.scale());
    const Eigen::Matrix3d R = S.rotationMatrix().cast<double>();
    const Eigen::Vector3d t = S.translation().cast<double>();
    return gtsam::Similarity3(gtsam::Rot3(R), gtsam::Point3(t), scale);
}

Sophus::Sim3f fromGTSAMSim3(const gtsam::Similarity3& S)
{
    const Eigen::Matrix3f R = S.rotation().matrix().cast<float>();
    const Eigen::Vector3f t = S.translation().cast<float>();
    const float s = static_cast<float>(S.scale());
    return Sophus::Sim3f(Sophus::RxSO3f(s, R), t);
}

gtsam::Cal3_S2 toGTSAMCal(const GeometricCamera* pCam)
{
    assert(pCam->GetType() == GeometricCamera::CAM_PINHOLE && "toGTSAMCal: camera must be pinhole");
    // Pinhole params: [fx, fy, cx, cy] (ORB-SLAM3 Pinhole::mvParameters)
    const double fx = pCam->getParameter(0);
    const double fy = pCam->getParameter(1);
    const double cx = pCam->getParameter(2);
    const double cy = pCam->getParameter(3);
    return gtsam::Cal3_S2(fx, fy, 0.0, cx, cy);
}

gtsam::Cal3_S2Stereo toGTSAMStereoCal(const GeometricCamera* pCam, double bf)
{
    assert(pCam->GetType() == GeometricCamera::CAM_PINHOLE && "toGTSAMStereoCal: camera must be pinhole");
    const double fx = pCam->getParameter(0);
    const double fy = pCam->getParameter(1);
    const double cx = pCam->getParameter(2);
    const double cy = pCam->getParameter(3);
    const double baseline = bf / fx;
    return gtsam::Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline);
}

// ─────────────────────────────────────────────────────────────────────────────
// Noise-model helpers
// ─────────────────────────────────────────────────────────────────────────────
gtsam::SharedNoiseModel makeHuberNoise(int dim, double chi2Threshold, double invSigma2)
{
    auto baseModel = gtsam::noiseModel::Isotropic::Precision(dim, invSigma2);
    auto huber = gtsam::noiseModel::mEstimator::Huber::Create(std::sqrt(chi2Threshold));
    return gtsam::noiseModel::Robust::Create(huber, baseModel);
}

gtsam::SharedNoiseModel makeIsotropicNoise(int dim, double invSigma2)
{
    return gtsam::noiseModel::Isotropic::Precision(dim, invSigma2);
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal helper: transform world point to camera frame given Twb and Tbc.
// Tcw = (Twb * Tbc)^{-1}  →  Xc = Tcw * Xw
// Optionally returns dXc/dTwb (3×6) and dXc/dXw (3×3).
// ─────────────────────────────────────────────────────────────────────────────
static Eigen::Vector3d transformToCamera(const gtsam::Pose3& Twb, const gtsam::Pose3& Tbc, const Eigen::Vector3d& Xw,
                                         boost::optional<Eigen::Matrix<double, 3, 6>&> dXc_dTwb = boost::none,
                                         boost::optional<Eigen::Matrix<double, 3, 3>&> dXc_dXw = boost::none)
{
    // Twc = Twb * Tbc  (camera pose in world)
    // Tcw = Twc^{-1}
    // Xc  = Tcw * Xw

    // Chain rule for Jacobians:
    // dXc/dTwb via dXc/dTwc and dTwc/dTwb
    // GTSAM Pose3::transformTo(p, Hpose, Hpoint) gives derivatives wrt the
    // pose on the LEFT side of the transformation.

    if (dXc_dTwb || dXc_dXw)
    {
        // Step 1: Twc = Twb * Tbc
        gtsam::Matrix6 dTwc_dTwb;
        gtsam::Pose3 Twc = Twb.compose(Tbc, dTwc_dTwb);

        // Step 2: Xc = Twc^{-1} * Xw  (transformTo)
        gtsam::Matrix36 dXc_dTwc;
        gtsam::Matrix33 dXc_dXw_local;
        Eigen::Vector3d Xc = Twc.transformTo(Xw, dXc_dTwb ? &dXc_dTwc : nullptr, dXc_dXw ? &dXc_dXw_local : nullptr);

        if (dXc_dTwb)
        {
            *dXc_dTwb = dXc_dTwc * dTwc_dTwb;
        }
        if (dXc_dXw)
        {
            *dXc_dXw = dXc_dXw_local;
        }
        return Xc;
    }
    else
    {
        return Twb.compose(Tbc).transformTo(Xw);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MonoOnlyPoseFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector MonoOnlyPoseFactor::evaluateError(const gtsam::Pose3& Twb, boost::optional<gtsam::Matrix&> H) const
{
    Eigen::Matrix<double, 3, 6> dXc_dTwb;
    boost::optional<Eigen::Matrix<double, 3, 6>&> optTwb =
        H ? boost::optional<Eigen::Matrix<double, 3, 6>&>(dXc_dTwb) : boost::none;
    Eigen::Vector3d Xc = transformToCamera(Twb, Tbc_, Xw_, optTwb);

    if (Xc(2) <= 0.0)
    {
        // Behind camera — return large error, zero Jacobian
        if (H)
        {
            *H = Eigen::Matrix<double, 2, 6>::Zero();
        }
        return Eigen::Vector2d(1e6, 1e6);
    }

    const Eigen::Matrix<double, 2, 3> dProj_dXc = pCamera_->projectJac(Xc);
    const Eigen::Vector2d proj = pCamera_->project(Xc);

    if (H)
    {
        *H = -dProj_dXc * dXc_dTwb;
    }
    return obs_ - proj;
}

bool MonoOnlyPoseFactor::isDepthPositive(const gtsam::Pose3& Twb) const
{
    return transformToCamera(Twb, Tbc_, Xw_)(2) > 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// StereoOnlyPoseFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector StereoOnlyPoseFactor::evaluateError(const gtsam::Pose3& Twb, boost::optional<gtsam::Matrix&> H) const
{
    Eigen::Matrix<double, 3, 6> dXc_dTwb;
    boost::optional<Eigen::Matrix<double, 3, 6>&> optTwb =
        H ? boost::optional<Eigen::Matrix<double, 3, 6>&>(dXc_dTwb) : boost::none;
    Eigen::Vector3d Xc = transformToCamera(Twb, Tbc_, Xw_, optTwb);

    if (Xc(2) <= 0.0)
    {
        if (H)
        {
            *H = Eigen::Matrix<double, 3, 6>::Zero();
        }
        return Eigen::Vector3d(1e6, 1e6, 1e6);
    }

    // Build 3×3 Jacobian of stereo projection wrt Xc
    Eigen::Matrix<double, 2, 3> proj_jac = pCamera_->projectJac(Xc);
    Eigen::Matrix<double, 3, 3> dStereo_dXc;
    dStereo_dXc.block<2, 3>(0, 0) = proj_jac;                    // rows 0-1: monocular
    dStereo_dXc.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);  // row  2:   disparity row
    const double invZ2 = 1.0 / (Xc(2) * Xc(2));
    dStereo_dXc(2, 2) += bf_ * invZ2;  // disparity's Z derivative

    // Stereo projection [ul, v, ur]
    Eigen::Vector2d proj2 = pCamera_->project(Xc);
    Eigen::Vector3d proj3;
    proj3(0) = proj2(0);
    proj3(1) = proj2(1);
    proj3(2) = proj2(0) - bf_ / Xc(2);

    if (H)
    {
        *H = -dStereo_dXc * dXc_dTwb;
    }
    return obs_ - proj3;
}

bool StereoOnlyPoseFactor::isDepthPositive(const gtsam::Pose3& Twb) const
{
    return transformToCamera(Twb, Tbc_, Xw_)(2) > 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// PinholeMonoPoseTcwFactor / PinholeStereoPoseTcwFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector PinholeMonoPoseTcwFactor::evaluateError(const gtsam::Pose3& Tcw, boost::optional<gtsam::Matrix&> H) const
{
    assert(pCamera_ && pCamera_->GetType() == GeometricCamera::CAM_PINHOLE);

    const gtsam::Point3 Xw_p(Xw_.x(), Xw_.y(), Xw_.z());
    gtsam::Matrix36 dXc_dTcw;
    gtsam::Point3 Xc_p;

    if (H)
    {
        Xc_p = Tcw.transformFrom(Xw_p, dXc_dTcw, boost::none);
    }
    else
    {
        Xc_p = Tcw.transformFrom(Xw_p);
    }

    const Eigen::Vector3d Xc(Xc_p.x(), Xc_p.y(), Xc_p.z());
    if (Xc.z() <= 0.0)
    {
        if (H)
        {
            *H = Eigen::Matrix<double, 2, 6>::Zero();
        }
        return Eigen::Vector2d(1e6, 1e6);
    }

    const Eigen::Matrix<double, 2, 3> dProj_dXc = pCamera_->projectJac(Xc);
    const Eigen::Vector2d proj = pCamera_->project(Xc);

    if (H)
    {
        *H = -dProj_dXc * dXc_dTcw;
    }

    return obs_ - proj;
}

gtsam::Vector PinholeStereoPoseTcwFactor::evaluateError(const gtsam::Pose3& Tcw,
                                                        boost::optional<gtsam::Matrix&> H) const
{
    assert(pCamera_ && pCamera_->GetType() == GeometricCamera::CAM_PINHOLE);

    const gtsam::Point3 Xw_p(Xw_.x(), Xw_.y(), Xw_.z());
    gtsam::Matrix36 dXc_dTcw;
    gtsam::Point3 Xc_p;

    if (H)
    {
        Xc_p = Tcw.transformFrom(Xw_p, dXc_dTcw, boost::none);
    }
    else
    {
        Xc_p = Tcw.transformFrom(Xw_p);
    }

    const Eigen::Vector3d Xc(Xc_p.x(), Xc_p.y(), Xc_p.z());
    if (Xc.z() <= 0.0)
    {
        if (H)
        {
            *H = Eigen::Matrix<double, 3, 6>::Zero();
        }
        return Eigen::Vector3d(1e6, 1e6, 1e6);
    }

    Eigen::Matrix<double, 2, 3> proj_jac = pCamera_->projectJac(Xc);
    Eigen::Matrix<double, 3, 3> dStereo_dXc;
    dStereo_dXc.block<2, 3>(0, 0) = proj_jac;
    dStereo_dXc.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
    const double invZ2 = 1.0 / (Xc.z() * Xc.z());
    dStereo_dXc(2, 2) += bf_ * invZ2;

    const Eigen::Vector2d proj2 = pCamera_->project(Xc);
    Eigen::Vector3d proj3;
    proj3(0) = proj2(0);
    proj3(1) = proj2(1);
    proj3(2) = proj2(0) - bf_ / Xc.z();

    if (H)
    {
        *H = -dStereo_dXc * dXc_dTcw;
    }

    return obs_ - proj3;
}

// ─────────────────────────────────────────────────────────────────────────────
// PinholeMonoTcwFactor / PinholeStereoTcwFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector PinholeMonoTcwFactor::evaluateError(const gtsam::Pose3& Tcw, const gtsam::Point3& Xw,
                                                  boost::optional<gtsam::Matrix&> H1,
                                                  boost::optional<gtsam::Matrix&> H2) const
{
    assert(pCamera_ && pCamera_->GetType() == GeometricCamera::CAM_PINHOLE);

    gtsam::Matrix36 dXc_dTcw;
    gtsam::Matrix33 dXc_dXw;
    gtsam::Point3 Xc_p;

    if (H1 || H2)
    {
        Xc_p = Tcw.transformFrom(Xw, H1 ? &dXc_dTcw : nullptr, H2 ? &dXc_dXw : nullptr);
    }
    else
    {
        Xc_p = Tcw.transformFrom(Xw);
    }

    const Eigen::Vector3d Xc(Xc_p.x(), Xc_p.y(), Xc_p.z());
    if (Xc.z() <= 0.0)
    {
        if (H1)
        {
            *H1 = Eigen::Matrix<double, 2, 6>::Zero();
        }
        if (H2)
        {
            *H2 = Eigen::Matrix<double, 2, 3>::Zero();
        }
        return Eigen::Vector2d(1e6, 1e6);
    }

    const Eigen::Matrix<double, 2, 3> dProj_dXc = pCamera_->projectJac(Xc);
    const Eigen::Vector2d proj = pCamera_->project(Xc);

    if (H1)
    {
        *H1 = -dProj_dXc * dXc_dTcw;
    }
    if (H2)
    {
        *H2 = -dProj_dXc * dXc_dXw;
    }

    return obs_ - proj;
}

gtsam::Vector PinholeStereoTcwFactor::evaluateError(const gtsam::Pose3& Tcw, const gtsam::Point3& Xw,
                                                    boost::optional<gtsam::Matrix&> H1,
                                                    boost::optional<gtsam::Matrix&> H2) const
{
    assert(pCamera_ && pCamera_->GetType() == GeometricCamera::CAM_PINHOLE);

    gtsam::Matrix36 dXc_dTcw;
    gtsam::Matrix33 dXc_dXw;
    gtsam::Point3 Xc_p;

    if (H1 || H2)
    {
        Xc_p = Tcw.transformFrom(Xw, H1 ? &dXc_dTcw : nullptr, H2 ? &dXc_dXw : nullptr);
    }
    else
    {
        Xc_p = Tcw.transformFrom(Xw);
    }

    const Eigen::Vector3d Xc(Xc_p.x(), Xc_p.y(), Xc_p.z());
    if (Xc.z() <= 0.0)
    {
        if (H1)
        {
            *H1 = Eigen::Matrix<double, 3, 6>::Zero();
        }
        if (H2)
        {
            *H2 = Eigen::Matrix<double, 3, 3>::Zero();
        }
        return Eigen::Vector3d(1e6, 1e6, 1e6);
    }

    Eigen::Matrix<double, 2, 3> proj_jac = pCamera_->projectJac(Xc);
    Eigen::Matrix<double, 3, 3> dStereo_dXc;
    dStereo_dXc.block<2, 3>(0, 0) = proj_jac;
    dStereo_dXc.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
    const double invZ2 = 1.0 / (Xc.z() * Xc.z());
    dStereo_dXc(2, 2) += bf_ * invZ2;

    const Eigen::Vector2d proj2 = pCamera_->project(Xc);
    Eigen::Vector3d proj3;
    proj3(0) = proj2(0);
    proj3(1) = proj2(1);
    proj3(2) = proj2(0) - bf_ / Xc.z();

    if (H1)
    {
        *H1 = -dStereo_dXc * dXc_dTcw;
    }
    if (H2)
    {
        *H2 = -dStereo_dXc * dXc_dXw;
    }

    return obs_ - proj3;
}

// ─────────────────────────────────────────────────────────────────────────────
// Sim3ProjectionFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector Sim3ProjectionFactor::evaluateError(const gtsam::Similarity3& S12,
                                                  boost::optional<gtsam::Matrix&> H) const
{
    // Transform fixed point P3Dc_ by S12
    gtsam::Matrix37 dP_dS12;  // 3 output × 7 Sim3 tangent
    gtsam::Point3 p;
    if (H)
        p = S12.transformFrom(P3Dc_, dP_dS12);
    else
        p = S12.transformFrom(P3Dc_);

    if (p(2) <= 0.0)
    {
        if (H)
            *H = Eigen::Matrix<double, 2, 7>::Zero();
        return Eigen::Vector2d(1e6, 1e6);
    }

    const Eigen::Matrix<double, 2, 3> dProj_dP = pCamera_->projectJac(p);
    const Eigen::Vector2d proj = pCamera_->project(p);

    if (H)
        *H = -dProj_dP * dP_dS12;

    return obs_ - proj;
}

// ─────────────────────────────────────────────────────────────────────────────
// InverseSim3ProjectionFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector InverseSim3ProjectionFactor::evaluateError(const gtsam::Similarity3& S12,
                                                         boost::optional<gtsam::Matrix&> H) const
{
    // Apply S12^{-1} then project
    gtsam::Matrix77 dSinv_dS;
    gtsam::Similarity3 Sinv;
    if (H)
        Sinv = S12.inverse(dSinv_dS);
    else
        Sinv = S12.inverse();

    gtsam::Matrix37 dP_dSinv;
    gtsam::Point3 p;
    if (H)
        p = Sinv.transformFrom(P3Dc2_, dP_dSinv);
    else
        p = Sinv.transformFrom(P3Dc2_);

    if (p(2) <= 0.0)
    {
        if (H)
            *H = Eigen::Matrix<double, 2, 7>::Zero();
        return Eigen::Vector2d(1e6, 1e6);
    }

    const Eigen::Matrix<double, 2, 3> dProj_dP = pCamera_->projectJac(p);
    const Eigen::Vector2d proj = pCamera_->project(p);

    if (H)
        *H = -dProj_dP * dP_dSinv * dSinv_dS;

    return obs_ - proj;
}

// ─────────────────────────────────────────────────────────────────────────────
// FourDOFBetweenFactor
//   Pose3 is stored in Tcw convention (world-to-camera) in this graph.
//   Error replicates Edge4DoF::computeError() exactly.
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector FourDOFBetweenFactor::evaluateError(const gtsam::Pose3& Ti, const gtsam::Pose3& Tj,
                                                  boost::optional<gtsam::Matrix&> H1,
                                                  boost::optional<gtsam::Matrix&> H2) const
{
    // In the 4DoF graph, pose.rotation() = Rcw, pose.translation() = tcw
    const Eigen::Matrix3d Rcwi = Ti.rotation().matrix();
    const Eigen::Vector3d tcwi = Ti.translation();
    const Eigen::Matrix3d Rcwj = Tj.rotation().matrix();
    const Eigen::Vector3d tcwj = Tj.translation();

    // twcj = -Rcwj^T * tcwj  (world position of camera j)
    const Eigen::Vector3d twcj = -Rcwj.transpose() * tcwj;

    // Error (replicates Edge4DoF::computeError)
    const Eigen::Vector3d er = LogSO3(Rcwi * Rcwj.transpose() * dRij_.transpose());
    const Eigen::Vector3d et = Rcwi * twcj + tcwi - dtij_;

    Eigen::Matrix<double, 6, 1> err;
    err << er, et;

    // Numerical Jacobians (consistent with g2o's numerical linearization for Edge4DoF)
    if (H1 || H2)
    {
        const double eps = 1e-7;
        Eigen::Matrix<double, 6, 6> J1 = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 6> J2 = Eigen::Matrix<double, 6, 6>::Zero();

        for (int i = 0; i < 6; ++i)
        {
            // Perturb Ti
            gtsam::Vector6 xi1 = gtsam::Vector6::Zero();
            xi1[i] = eps;
            const gtsam::Pose3 Ti_p = Ti.retract(xi1);
            const Eigen::Matrix3d Rcwi_p = Ti_p.rotation().matrix();
            const Eigen::Vector3d tcwi_p = Ti_p.translation();
            const Eigen::Vector3d er_p1 = LogSO3(Rcwi_p * Rcwj.transpose() * dRij_.transpose());
            const Eigen::Vector3d et_p1 = Rcwi_p * twcj + tcwi_p - dtij_;
            J1.col(i).head<3>() = (er_p1 - er) / eps;
            J1.col(i).tail<3>() = (et_p1 - et) / eps;

            // Perturb Tj
            gtsam::Vector6 xi2 = gtsam::Vector6::Zero();
            xi2[i] = eps;
            const gtsam::Pose3 Tj_p = Tj.retract(xi2);
            const Eigen::Matrix3d Rcwj_p = Tj_p.rotation().matrix();
            const Eigen::Vector3d tcwj_p = Tj_p.translation();
            const Eigen::Vector3d twcj_p = -Rcwj_p.transpose() * tcwj_p;
            const Eigen::Vector3d er_p2 = LogSO3(Rcwi * Rcwj_p.transpose() * dRij_.transpose());
            const Eigen::Vector3d et_p2 = Rcwi * twcj_p + tcwi - dtij_;
            J2.col(i).head<3>() = (er_p2 - er) / eps;
            J2.col(i).tail<3>() = (et_p2 - et) / eps;
        }

        if (H1)
            *H1 = J1;
        if (H2)
            *H2 = J2;
    }

    return err;
}

}  // namespace ORB_SLAM3
