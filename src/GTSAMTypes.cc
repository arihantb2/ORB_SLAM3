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
    const double d  = std::sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5)
    {
        const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
        return NormalizeRotation(R);
    }
    else
    {
        const Eigen::Matrix3d R =
            Eigen::Matrix3d::Identity() + W * std::sin(d) / d + W * W * (1.0 - std::cos(d)) / d2;
        return NormalizeRotation(R);
    }
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R)
{
    const double tr = R(0, 0) + R(1, 1) + R(2, 2);
    Eigen::Vector3d w;
    w << (R(2, 1) - R(1, 2)) / 2.0,
         (R(0, 2) - R(2, 0)) / 2.0,
         (R(1, 0) - R(0, 1)) / 2.0;
    const double costheta = (tr - 1.0) * 0.5;
    if (costheta > 1.0 || costheta < -1.0)
        return w;
    const double theta = std::acos(costheta);
    const double s     = std::sin(theta);
    if (std::fabs(s) < 1e-5)
        return w;
    return theta * w / s;
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d S;
    S <<  0.0,  -w(2),  w(1),
          w(2),   0.0, -w(0),
         -w(1),  w(0),   0.0;
    return S;
}

Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x * x + y * y + z * z;
    const double d  = std::sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5)
        return Eigen::Matrix3d::Identity();
    return Eigen::Matrix3d::Identity() -
           W * (1.0 - std::cos(d)) / d2 +
           W * W * (d - std::sin(d)) / (d2 * d);
}

Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v)
{
    return RightJacobianSO3(v[0], v[1], v[2]);
}

Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x * x + y * y + z * z;
    const double d  = std::sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5)
        return Eigen::Matrix3d::Identity();
    return Eigen::Matrix3d::Identity() +
           W / 2.0 +
           W * W * (1.0 / d2 - (1.0 + std::cos(d)) / (2.0 * d * std::sin(d)));
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
    const Eigen::Matrix3f R = P.rotation().matrix().cast<float>();
    const Eigen::Vector3f t = P.translation().cast<float>();
    return Sophus::SE3f(R, t);
}

gtsam::imuBias::ConstantBias toGTSAMBias(const IMU::Bias& b)
{
    // ConstantBias stores (accelerometer, gyroscope)
    // IMU::Bias stores   (bax,bay,baz, bwx,bwy,bwz) — same order
    return gtsam::imuBias::ConstantBias(
        gtsam::Vector3(b.bax, b.bay, b.baz),
        gtsam::Vector3(b.bwx, b.bwy, b.bwz));
}

IMU::Bias fromGTSAMBias(const gtsam::imuBias::ConstantBias& cb)
{
    const auto& ba = cb.accelerometer();
    const auto& bg = cb.gyroscope();
    return IMU::Bias(static_cast<float>(ba.x()), static_cast<float>(ba.y()), static_cast<float>(ba.z()),
                     static_cast<float>(bg.x()), static_cast<float>(bg.y()), static_cast<float>(bg.z()));
}

gtsam::Similarity3 toGTSAMSim3(const Sophus::Sim3f& S)
{
    const double scale  = static_cast<double>(S.scale());
    const Eigen::Matrix3d R = S.rotationMatrix().cast<double>();
    const Eigen::Vector3d t = S.translation().cast<double>();
    return gtsam::Similarity3(gtsam::Rot3(R), gtsam::Point3(t), scale);
}

Sophus::Sim3f fromGTSAMSim3(const gtsam::Similarity3& S)
{
    const Eigen::Matrix3f R = S.rotation().matrix().cast<float>();
    const Eigen::Vector3f t = S.translation().cast<float>();
    const float s           = static_cast<float>(S.scale());
    return Sophus::Sim3f(Sophus::RxSO3f(s, R), t);
}

gtsam::Cal3_S2 toGTSAMCal(const GeometricCamera* pCam)
{
    assert(pCam->GetType() == GeometricCamera::CAM_PINHOLE &&
           "toGTSAMCal: camera must be pinhole");
    // Pinhole params: [fx, fy, cx, cy] (ORB-SLAM3 Pinhole::mvParameters)
    const double fx = pCam->getParameter(0);
    const double fy = pCam->getParameter(1);
    const double cx = pCam->getParameter(2);
    const double cy = pCam->getParameter(3);
    return gtsam::Cal3_S2(fx, fy, 0.0, cx, cy);
}

gtsam::Cal3_S2Stereo toGTSAMStereoCal(const GeometricCamera* pCam, double bf)
{
    assert(pCam->GetType() == GeometricCamera::CAM_PINHOLE &&
           "toGTSAMStereoCal: camera must be pinhole");
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
    auto huber     = gtsam::noiseModel::mEstimator::Huber::Create(std::sqrt(chi2Threshold));
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
static Eigen::Vector3d transformToCamera(
    const gtsam::Pose3& Twb, const gtsam::Pose3& Tbc, const Eigen::Vector3d& Xw,
    boost::optional<Eigen::Matrix<double, 3, 6>&> dXc_dTwb = boost::none,
    boost::optional<Eigen::Matrix<double, 3, 3>&> dXc_dXw  = boost::none)
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
        Eigen::Vector3d Xc = Twc.transformTo(Xw,
            dXc_dTwb ? &dXc_dTwc : nullptr,
            dXc_dXw  ? &dXc_dXw_local : nullptr);

        if (dXc_dTwb)
            *dXc_dTwb = dXc_dTwc * dTwc_dTwb;
        if (dXc_dXw)
            *dXc_dXw = dXc_dXw_local;

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
gtsam::Vector MonoOnlyPoseFactor::evaluateError(
    const gtsam::Pose3& Twb,
    boost::optional<gtsam::Matrix&> H) const
{
    Eigen::Matrix<double, 3, 6> dXc_dTwb;
    boost::optional<Eigen::Matrix<double, 3, 6>&> optTwb = H ? boost::optional<Eigen::Matrix<double, 3, 6>&>(dXc_dTwb) : boost::none;
    Eigen::Vector3d Xc = transformToCamera(Twb, Tbc_, Xw_, optTwb);

    if (Xc(2) <= 0.0)
    {
        // Behind camera — return large error, zero Jacobian
        if (H)
            *H = Eigen::Matrix<double, 2, 6>::Zero();
        return Eigen::Vector2d(1e6, 1e6);
    }

    const Eigen::Matrix<double, 2, 3> dProj_dXc = pCamera_->projectJac(Xc);
    const Eigen::Vector2d proj = pCamera_->project(Xc);

    if (H)
        *H = -dProj_dXc * dXc_dTwb;

    return obs_ - proj;
}

bool MonoOnlyPoseFactor::isDepthPositive(const gtsam::Pose3& Twb) const
{
    return transformToCamera(Twb, Tbc_, Xw_)(2) > 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// StereoOnlyPoseFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector StereoOnlyPoseFactor::evaluateError(
    const gtsam::Pose3& Twb,
    boost::optional<gtsam::Matrix&> H) const
{
    Eigen::Matrix<double, 3, 6> dXc_dTwb;
    boost::optional<Eigen::Matrix<double, 3, 6>&> optTwb = H ? boost::optional<Eigen::Matrix<double, 3, 6>&>(dXc_dTwb) : boost::none;
    Eigen::Vector3d Xc = transformToCamera(Twb, Tbc_, Xw_, optTwb);

    if (Xc(2) <= 0.0)
    {
        if (H)
            *H = Eigen::Matrix<double, 3, 6>::Zero();
        return Eigen::Vector3d(1e6, 1e6, 1e6);
    }

    // Build 3×3 Jacobian of stereo projection wrt Xc
    Eigen::Matrix<double, 2, 3> proj_jac = pCamera_->projectJac(Xc);
    Eigen::Matrix<double, 3, 3> dStereo_dXc;
    dStereo_dXc.block<2, 3>(0, 0) = proj_jac;                     // rows 0-1: monocular
    dStereo_dXc.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);   // row  2:   disparity row
    const double invZ2 = 1.0 / (Xc(2) * Xc(2));
    dStereo_dXc(2, 2) += bf_ * invZ2;                              // disparity's Z derivative

    // Stereo projection [ul, v, ur]
    Eigen::Vector2d proj2 = pCamera_->project(Xc);
    Eigen::Vector3d proj3;
    proj3(0) = proj2(0);
    proj3(1) = proj2(1);
    proj3(2) = proj2(0) - bf_ / Xc(2);

    if (H)
        *H = -dStereo_dXc * dXc_dTwb;

    return obs_ - proj3;
}

bool StereoOnlyPoseFactor::isDepthPositive(const gtsam::Pose3& Twb) const
{
    return transformToCamera(Twb, Tbc_, Xw_)(2) > 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// FisheyeProjectionFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector FisheyeProjectionFactor::evaluateError(
    const gtsam::Pose3& Twb, const gtsam::Point3& Xw,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const
{
    Eigen::Matrix<double, 3, 6> dXc_dTwb;
    Eigen::Matrix<double, 3, 3> dXc_dXw;
    boost::optional<Eigen::Matrix<double, 3, 6>&> optTwb = (H1 || H2) ? boost::optional<Eigen::Matrix<double, 3, 6>&>(dXc_dTwb) : boost::none;
    boost::optional<Eigen::Matrix<double, 3, 3>&> optXw  = (H1 || H2) ? boost::optional<Eigen::Matrix<double, 3, 3>&>(dXc_dXw) : boost::none;
    Eigen::Vector3d Xc = transformToCamera(Twb, Tbc_, Xw, optTwb, optXw);

    if (Xc(2) <= 0.0)
    {
        if (H1) *H1 = Eigen::Matrix<double, 2, 6>::Zero();
        if (H2) *H2 = Eigen::Matrix<double, 2, 3>::Zero();
        return Eigen::Vector2d(1e6, 1e6);
    }

    const Eigen::Matrix<double, 2, 3> dProj_dXc = pCamera_->projectJac(Xc);
    const Eigen::Vector2d proj = pCamera_->project(Xc);

    if (H1) *H1 = -dProj_dXc * dXc_dTwb;
    if (H2) *H2 = -dProj_dXc * dXc_dXw;

    return obs_ - proj;
}

bool FisheyeProjectionFactor::isDepthPositive(const gtsam::Pose3& Twb, const gtsam::Point3& Xw) const
{
    return transformToCamera(Twb, Tbc_, Xw)(2) > 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// FisheyeStereoFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector FisheyeStereoFactor::evaluateError(
    const gtsam::Pose3& Twb, const gtsam::Point3& Xw,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const
{
    Eigen::Matrix<double, 3, 6> dXc_dTwb;
    Eigen::Matrix<double, 3, 3> dXc_dXw;
    boost::optional<Eigen::Matrix<double, 3, 6>&> optTwb = (H1 || H2) ? boost::optional<Eigen::Matrix<double, 3, 6>&>(dXc_dTwb) : boost::none;
    boost::optional<Eigen::Matrix<double, 3, 3>&> optXw  = (H1 || H2) ? boost::optional<Eigen::Matrix<double, 3, 3>&>(dXc_dXw) : boost::none;
    Eigen::Vector3d Xc = transformToCamera(Twb, Tbc_, Xw, optTwb, optXw);

    if (Xc(2) <= 0.0)
    {
        if (H1) *H1 = Eigen::Matrix<double, 3, 6>::Zero();
        if (H2) *H2 = Eigen::Matrix<double, 3, 3>::Zero();
        return Eigen::Vector3d(1e6, 1e6, 1e6);
    }

    Eigen::Matrix<double, 2, 3> proj_jac = pCamera_->projectJac(Xc);
    Eigen::Matrix<double, 3, 3> dStereo_dXc;
    dStereo_dXc.block<2, 3>(0, 0) = proj_jac;
    dStereo_dXc.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
    const double invZ2 = 1.0 / (Xc(2) * Xc(2));
    dStereo_dXc(2, 2) += bf_ * invZ2;

    Eigen::Vector2d proj2 = pCamera_->project(Xc);
    Eigen::Vector3d proj3;
    proj3(0) = proj2(0);
    proj3(1) = proj2(1);
    proj3(2) = proj2(0) - bf_ / Xc(2);

    if (H1) *H1 = -dStereo_dXc * dXc_dTwb;
    if (H2) *H2 = -dStereo_dXc * dXc_dXw;

    return obs_ - proj3;
}

bool FisheyeStereoFactor::isDepthPositive(const gtsam::Pose3& Twb, const gtsam::Point3& Xw) const
{
    return transformToCamera(Twb, Tbc_, Xw)(2) > 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// InertialFactor
// ─────────────────────────────────────────────────────────────────────────────
static Matrix9d buildInertialInfo(IMU::Preintegrated* pInt)
{
    Matrix9d Info = pInt->C.block<9, 9>(0, 0).cast<double>().inverse();
    Info = (Info + Info.transpose()) / 2.0;
    Eigen::SelfAdjointEigenSolver<Matrix9d> es(Info);
    Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
    for (int i = 0; i < 9; i++)
        if (eigs[i] < 1e-12)
            eigs[i] = 0.0;
    return es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
}

InertialFactor::InertialFactor(
    const gtsam::Key& pose1Key, const gtsam::Key& vel1Key,
    const gtsam::Key& bias1Key, const gtsam::Key& pose2Key,
    const gtsam::Key& vel2Key,
    IMU::Preintegrated* pInt)
    : gtsam::NonlinearFactor(gtsam::KeyVector{pose1Key, vel1Key, bias1Key, pose2Key, vel2Key}),
      JRg_(pInt->JRg.cast<double>()),
      JVg_(pInt->JVg.cast<double>()),
      JPg_(pInt->JPg.cast<double>()),
      JVa_(pInt->JVa.cast<double>()),
      JPa_(pInt->JPa.cast<double>()),
      mpInt_(pInt),
      dt_(static_cast<double>(pInt->dT)),
      g_(0.0, 0.0, -static_cast<double>(IMU::GRAVITY_VALUE)),
      information_(buildInertialInfo(pInt))
{}

Eigen::Matrix<double, 9, 1> InertialFactor::computeResidual(
    const gtsam::Pose3& Twb1, const Eigen::Vector3d& v1,
    const gtsam::imuBias::ConstantBias& bias1,
    const gtsam::Pose3& Twb2, const Eigen::Vector3d& v2) const
{
    // Build IMU::Bias from ConstantBias (acc, gyro order)
    const auto& ba = bias1.accelerometer();
    const auto& bg = bias1.gyroscope();
    const IMU::Bias b1(static_cast<float>(ba.x()), static_cast<float>(ba.y()), static_cast<float>(ba.z()),
                       static_cast<float>(bg.x()), static_cast<float>(bg.y()), static_cast<float>(bg.z()));

    const Eigen::Matrix3d dR = mpInt_->GetDeltaRotation(b1).cast<double>();
    const Eigen::Vector3d dV = mpInt_->GetDeltaVelocity(b1).cast<double>();
    const Eigen::Vector3d dP = mpInt_->GetDeltaPosition(b1).cast<double>();

    const Eigen::Matrix3d Rwb1 = Twb1.rotation().matrix();
    const Eigen::Matrix3d Rwb2 = Twb2.rotation().matrix();
    const Eigen::Vector3d twb1 = Twb1.translation();
    const Eigen::Vector3d twb2 = Twb2.translation();

    const Eigen::Vector3d er = LogSO3(dR.transpose() * Rwb1.transpose() * Rwb2);
    const Eigen::Vector3d ev = Rwb1.transpose() * (v2 - v1 - g_ * dt_) - dV;
    const Eigen::Vector3d ep = Rwb1.transpose() * (twb2 - twb1 - v1 * dt_ - 0.5 * g_ * dt_ * dt_) - dP;

    Eigen::Matrix<double, 9, 1> err;
    err << er, ev, ep;
    return err;
}

double InertialFactor::error(const gtsam::Values& c) const
{
    const auto Twb1  = c.at<gtsam::Pose3>(keys()[0]);
    const auto v1    = c.at<gtsam::Vector3>(keys()[1]);
    const auto bias1 = c.at<gtsam::imuBias::ConstantBias>(keys()[2]);
    const auto Twb2  = c.at<gtsam::Pose3>(keys()[3]);
    const auto v2    = c.at<gtsam::Vector3>(keys()[4]);

    const Eigen::Matrix<double, 9, 1> res = computeResidual(Twb1, v1, bias1, Twb2, v2);
    return 0.5 * res.transpose() * information_ * res;
}

void InertialFactor::computeJacobians(
    const gtsam::Pose3& Twb1, const Eigen::Vector3d& v1,
    const gtsam::imuBias::ConstantBias& bias1,
    const gtsam::Pose3& Twb2, const Eigen::Vector3d& v2,
    Eigen::Matrix<double, 9, 6>&  J_pose1,
    Eigen::Matrix<double, 9, 3>&  J_vel1,
    Eigen::Matrix<double, 9, 6>&  J_bias1,
    Eigen::Matrix<double, 9, 6>&  J_pose2,
    Eigen::Matrix<double, 9, 3>&  J_vel2) const
{
    const auto& ba = bias1.accelerometer();
    const auto& bg = bias1.gyroscope();
    const IMU::Bias b1(static_cast<float>(ba.x()), static_cast<float>(ba.y()), static_cast<float>(ba.z()),
                       static_cast<float>(bg.x()), static_cast<float>(bg.y()), static_cast<float>(bg.z()));

    const IMU::Bias db = mpInt_->GetDeltaBias(b1);
    Eigen::Vector3d dbg_vec;
    dbg_vec << db.bwx, db.bwy, db.bwz;

    const Eigen::Matrix3d Rwb1 = Twb1.rotation().matrix();
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = Twb2.rotation().matrix();
    const Eigen::Vector3d twb1 = Twb1.translation();
    const Eigen::Vector3d twb2 = Twb2.translation();

    const Eigen::Matrix3d dR   = mpInt_->GetDeltaRotation(b1).cast<double>();
    const Eigen::Matrix3d eR   = dR.transpose() * Rbw1 * Rwb2;
    const Eigen::Vector3d er   = LogSO3(eR);
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

    // ── Jacobians wrt Pose1  (9×6: [rot_part | trans_part]) ──────────────────
    J_pose1.setZero();
    // rotation block (rows 0-2)
    J_pose1.block<3, 3>(0, 0) = -invJr * Rwb2.transpose() * Rwb1;
    J_pose1.block<3, 3>(3, 0) = Skew(Rbw1 * (v2 - v1 - g_ * dt_));
    J_pose1.block<3, 3>(6, 0) = Skew(Rbw1 * (twb2 - twb1 - v1 * dt_ - 0.5 * g_ * dt_ * dt_));
    // translation block (rows 6-8)
    J_pose1.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();

    // ── Jacobians wrt Velocity1  (9×3) ───────────────────────────────────────
    J_vel1.setZero();
    J_vel1.block<3, 3>(3, 0) = -Rbw1;
    J_vel1.block<3, 3>(6, 0) = -Rbw1 * dt_;

    // ── Jacobians wrt combined bias  (9×6: [gyro_cols 0-2 | acc_cols 3-5]) ──
    // Note: ConstantBias stores (acc, gyro) but here we order (gyro, acc) to
    //       match EdgeInertial::linearizeOplus ordering (_jacobianOplus[2] = gyro,
    //       _jacobianOplus[3] = acc).  Caller must know this layout.
    J_bias1.setZero();
    // Gyro columns (0-2)
    J_bias1.block<3, 3>(0, 0) = -invJr * eR.transpose() * RightJacobianSO3(JRg_ * dbg_vec) * JRg_;
    J_bias1.block<3, 3>(3, 0) = -JVg_;
    J_bias1.block<3, 3>(6, 0) = -JPg_;
    // Acc columns (3-5)
    J_bias1.block<3, 3>(3, 3) = -JVa_;
    J_bias1.block<3, 3>(6, 3) = -JPa_;

    // ── Jacobians wrt Pose2  (9×6) ───────────────────────────────────────────
    J_pose2.setZero();
    J_pose2.block<3, 3>(0, 0) = invJr;
    J_pose2.block<3, 3>(6, 3) = Rbw1 * Rwb2;

    // ── Jacobians wrt Velocity2  (9×3) ───────────────────────────────────────
    J_vel2.setZero();
    J_vel2.block<3, 3>(3, 0) = Rbw1;
}

boost::shared_ptr<gtsam::GaussianFactor> InertialFactor::linearize(const gtsam::Values& c) const
{
    const auto Twb1  = c.at<gtsam::Pose3>(keys()[0]);
    const auto v1    = c.at<gtsam::Vector3>(keys()[1]);
    const auto bias1 = c.at<gtsam::imuBias::ConstantBias>(keys()[2]);
    const auto Twb2  = c.at<gtsam::Pose3>(keys()[3]);
    const auto v2    = c.at<gtsam::Vector3>(keys()[4]);

    Eigen::Matrix<double, 9, 6> J_pose1, J_pose2;
    Eigen::Matrix<double, 9, 3> J_vel1, J_vel2;
    Eigen::Matrix<double, 9, 6> J_bias1;
    computeJacobians(Twb1, v1, bias1, Twb2, v2,
                     J_pose1, J_vel1, J_bias1, J_pose2, J_vel2);

    // Build full 9×(6+3+6+6+3) = 9×24 Jacobian
    // key order: pose1(6), vel1(3), bias1(6), pose2(6), vel2(3)
    // bias1 in ConstantBias tangent space is 6D (acc 3 + gyro 3)
    // Our J_bias1 is laid out as [gyro(3) | acc(3)] for historical reasons;
    // ConstantBias tangent space is [acc(3) | gyro(3)], so we swap columns.
    Eigen::Matrix<double, 9, 6> J_bias1_gtsam;
    J_bias1_gtsam.block<9, 3>(0, 0) = J_bias1.block<9, 3>(0, 3);  // acc first
    J_bias1_gtsam.block<9, 3>(0, 3) = J_bias1.block<9, 3>(0, 0);  // then gyro

    const Eigen::Matrix<double, 9, 1> res = computeResidual(Twb1, v1, bias1, Twb2, v2);
    // Whiten: whitened_J = sqrt_info * J,  whitened_b = sqrt_info * (-res)
    // JacobianFactor stores A*x = b form; GTSAM handles whitening internally
    // We pass the information matrix via a separate RHS construction.
    // Use JacobianFactor with explicit sqrt-information weighting:
    const Eigen::LLT<Matrix9d> llt(information_);
    const Eigen::Matrix<double, 9, 9> sqrtInfo = llt.matrixU();  // upper triangular

    // Whitened Jacobians
    const Eigen::Matrix<double, 9, 6>  wJ_p1   = sqrtInfo * J_pose1;
    const Eigen::Matrix<double, 9, 3>  wJ_v1   = sqrtInfo * J_vel1;
    const Eigen::Matrix<double, 9, 6>  wJ_b1   = sqrtInfo * J_bias1_gtsam;
    const Eigen::Matrix<double, 9, 6>  wJ_p2   = sqrtInfo * J_pose2;
    const Eigen::Matrix<double, 9, 3>  wJ_v2   = sqrtInfo * J_vel2;
    const Eigen::Matrix<double, 9, 1>  wRes    = sqrtInfo * (-res);

    std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
    terms.emplace_back(keys()[0], wJ_p1);
    terms.emplace_back(keys()[1], wJ_v1);
    terms.emplace_back(keys()[2], wJ_b1);
    terms.emplace_back(keys()[3], wJ_p2);
    terms.emplace_back(keys()[4], wJ_v2);
    return boost::make_shared<gtsam::JacobianFactor>(terms, wRes);
}

// ─────────────────────────────────────────────────────────────────────────────
// InertialGSFactor
// ─────────────────────────────────────────────────────────────────────────────
InertialGSFactor::InertialGSFactor(
    const gtsam::Key& pose1Key, const gtsam::Key& vel1Key,
    const gtsam::Key& bias1Key, const gtsam::Key& pose2Key,
    const gtsam::Key& vel2Key,  const gtsam::Key& gravKey,
    const gtsam::Key& scaleKey,
    IMU::Preintegrated* pInt)
    : gtsam::NonlinearFactor(gtsam::KeyVector{pose1Key, vel1Key, bias1Key, pose2Key, vel2Key, gravKey, scaleKey}),
      JRg_(pInt->JRg.cast<double>()),
      JVg_(pInt->JVg.cast<double>()),
      JPg_(pInt->JPg.cast<double>()),
      JVa_(pInt->JVa.cast<double>()),
      JPa_(pInt->JPa.cast<double>()),
      mpInt_(pInt),
      dt_(static_cast<double>(pInt->dT)),
      gI_(0.0, 0.0, -static_cast<double>(IMU::GRAVITY_VALUE)),
      information_(buildInertialInfo(pInt))
{}

Eigen::Matrix<double, 9, 1> InertialGSFactor::computeResidual(
    const gtsam::Pose3& Twb1, const Eigen::Vector3d& v1,
    const gtsam::imuBias::ConstantBias& bias1,
    const gtsam::Pose3& Twb2, const Eigen::Vector3d& v2,
    const gtsam::Rot3& gravRot, double logScale) const
{
    const auto& ba = bias1.accelerometer();
    const auto& bg = bias1.gyroscope();
    const IMU::Bias b(static_cast<float>(ba.x()), static_cast<float>(ba.y()), static_cast<float>(ba.z()),
                      static_cast<float>(bg.x()), static_cast<float>(bg.y()), static_cast<float>(bg.z()));

    const double s = std::exp(logScale);
    const Eigen::Vector3d g = gravRot.matrix() * gI_;

    const Eigen::Matrix3d dR = mpInt_->GetDeltaRotation(b).cast<double>();
    const Eigen::Vector3d dV = mpInt_->GetDeltaVelocity(b).cast<double>();
    const Eigen::Vector3d dP = mpInt_->GetDeltaPosition(b).cast<double>();

    const Eigen::Matrix3d Rwb1 = Twb1.rotation().matrix();
    const Eigen::Matrix3d Rwb2 = Twb2.rotation().matrix();
    const Eigen::Vector3d twb1 = Twb1.translation();
    const Eigen::Vector3d twb2 = Twb2.translation();

    const Eigen::Vector3d er = LogSO3(dR.transpose() * Rwb1.transpose() * Rwb2);
    const Eigen::Vector3d ev = Rwb1.transpose() * (s * (v2 - v1) - g * dt_) - dV;
    const Eigen::Vector3d ep =
        Rwb1.transpose() * (s * (twb2 - twb1 - v1 * dt_) - 0.5 * g * dt_ * dt_) - dP;

    Eigen::Matrix<double, 9, 1> err;
    err << er, ev, ep;
    return err;
}

double InertialGSFactor::error(const gtsam::Values& c) const
{
    const auto Twb1     = c.at<gtsam::Pose3>(keys()[0]);
    const auto v1       = c.at<gtsam::Vector3>(keys()[1]);
    const auto bias1    = c.at<gtsam::imuBias::ConstantBias>(keys()[2]);
    const auto Twb2     = c.at<gtsam::Pose3>(keys()[3]);
    const auto v2       = c.at<gtsam::Vector3>(keys()[4]);
    const auto gravRot  = c.at<gtsam::Rot3>(keys()[5]);
    const double logScale = c.atDouble(keys()[6]);

    const auto res = computeResidual(Twb1, v1, bias1, Twb2, v2, gravRot, logScale);
    return 0.5 * res.transpose() * information_ * res;
}

void InertialGSFactor::computeJacobians(
    const gtsam::Pose3& Twb1, const Eigen::Vector3d& v1,
    const gtsam::imuBias::ConstantBias& bias1,
    const gtsam::Pose3& Twb2, const Eigen::Vector3d& v2,
    const gtsam::Rot3& gravRot, double logScale,
    Eigen::Matrix<double, 9, 6>&  J_pose1,
    Eigen::Matrix<double, 9, 3>&  J_vel1,
    Eigen::Matrix<double, 9, 6>&  J_bias1,
    Eigen::Matrix<double, 9, 6>&  J_pose2,
    Eigen::Matrix<double, 9, 3>&  J_vel2,
    Eigen::Matrix<double, 9, 2>&  J_grav,
    Eigen::Matrix<double, 9, 1>&  J_scale) const
{
    const auto& ba = bias1.accelerometer();
    const auto& bg = bias1.gyroscope();
    const IMU::Bias b(static_cast<float>(ba.x()), static_cast<float>(ba.y()), static_cast<float>(ba.z()),
                      static_cast<float>(bg.x()), static_cast<float>(bg.y()), static_cast<float>(bg.z()));
    const IMU::Bias db = mpInt_->GetDeltaBias(b);
    Eigen::Vector3d dbg_vec;
    dbg_vec << db.bwx, db.bwy, db.bwz;

    const double s = std::exp(logScale);
    const Eigen::Matrix3d Rwg = gravRot.matrix();
    const Eigen::Vector3d g = Rwg * gI_;

    const Eigen::Matrix3d Rwb1 = Twb1.rotation().matrix();
    const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2 = Twb2.rotation().matrix();
    const Eigen::Vector3d twb1 = Twb1.translation();
    const Eigen::Vector3d twb2 = Twb2.translation();

    const Eigen::Matrix3d dR   = mpInt_->GetDeltaRotation(b).cast<double>();
    const Eigen::Matrix3d eR   = dR.transpose() * Rbw1 * Rwb2;
    const Eigen::Vector3d er   = LogSO3(eR);
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

    // Gravity direction Jacobian matrix (3×2 tangent in SO3, columns = [roll, pitch])
    // Gm replicates the g2o formulation: dg/dtheta rows
    Eigen::MatrixXd Gm = Eigen::MatrixXd::Zero(3, 2);
    Gm(0, 1) = -IMU::GRAVITY_VALUE;
    Gm(1, 0) =  IMU::GRAVITY_VALUE;
    const Eigen::MatrixXd dGdTheta = Rwg * Gm;  // 3×2

    // ── Jacobians wrt Pose1 ───────────────────────────────────────────────────
    J_pose1.setZero();
    J_pose1.block<3, 3>(0, 0) = -invJr * Rwb2.transpose() * Rwb1;
    J_pose1.block<3, 3>(3, 0) = Skew(Rbw1 * (s * (v2 - v1) - g * dt_));
    J_pose1.block<3, 3>(6, 0) = Skew(Rbw1 * (s * (twb2 - twb1 - v1 * dt_) - 0.5 * g * dt_ * dt_));
    J_pose1.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<double, 3>(-s, -s, -s);

    // ── Jacobians wrt Velocity1 ───────────────────────────────────────────────
    J_vel1.setZero();
    J_vel1.block<3, 3>(3, 0) = -s * Rbw1;
    J_vel1.block<3, 3>(6, 0) = -s * Rbw1 * dt_;

    // ── Jacobians wrt combined bias (gyro|acc layout — same as InertialFactor) ─
    J_bias1.setZero();
    J_bias1.block<3, 3>(0, 0) = -invJr * eR.transpose() * RightJacobianSO3(JRg_ * dbg_vec) * JRg_;
    J_bias1.block<3, 3>(3, 0) = -JVg_;
    J_bias1.block<3, 3>(6, 0) = -JPg_;
    J_bias1.block<3, 3>(3, 3) = -JVa_;
    J_bias1.block<3, 3>(6, 3) = -JPa_;

    // ── Jacobians wrt Pose2 ───────────────────────────────────────────────────
    J_pose2.setZero();
    J_pose2.block<3, 3>(0, 0) = invJr;
    J_pose2.block<3, 3>(6, 3) = s * Rbw1 * Rwb2;

    // ── Jacobians wrt Velocity2 ───────────────────────────────────────────────
    J_vel2.setZero();
    J_vel2.block<3, 3>(3, 0) = s * Rbw1;

    // ── Jacobians wrt gravity direction (9×2; 3rd SO3 tangent col = 0) ────────
    J_grav.setZero();
    J_grav.block<3, 2>(3, 0) = -Rbw1 * dGdTheta * dt_;
    J_grav.block<3, 2>(6, 0) = -0.5 * Rbw1 * dGdTheta * dt_ * dt_;
    // Column 2 (yaw-around-gravity) is already zero — matches VertexGDir::oplusImpl

    // ── Jacobians wrt log-scale ───────────────────────────────────────────────
    // dE/d(log_s) = dE/ds * ds/d(log_s) = dE/ds * s
    J_scale.setZero();
    J_scale.block<3, 1>(3, 0) = Rbw1 * (v2 - v1);
    J_scale.block<3, 1>(6, 0) = Rbw1 * (twb2 - twb1 - v1 * dt_);
    J_scale *= s;  // chain rule: d(log_s) = s * d(scale)
}

boost::shared_ptr<gtsam::GaussianFactor> InertialGSFactor::linearize(const gtsam::Values& c) const
{
    const auto Twb1     = c.at<gtsam::Pose3>(keys()[0]);
    const auto v1       = c.at<gtsam::Vector3>(keys()[1]);
    const auto bias1    = c.at<gtsam::imuBias::ConstantBias>(keys()[2]);
    const auto Twb2     = c.at<gtsam::Pose3>(keys()[3]);
    const auto v2       = c.at<gtsam::Vector3>(keys()[4]);
    const auto gravRot  = c.at<gtsam::Rot3>(keys()[5]);
    const double logScale = c.atDouble(keys()[6]);

    Eigen::Matrix<double, 9, 6> J_pose1, J_pose2;
    Eigen::Matrix<double, 9, 3> J_vel1, J_vel2;
    Eigen::Matrix<double, 9, 6> J_bias1;
    Eigen::Matrix<double, 9, 2> J_grav;
    Eigen::Matrix<double, 9, 1> J_scale;
    computeJacobians(Twb1, v1, bias1, Twb2, v2, gravRot, logScale,
                     J_pose1, J_vel1, J_bias1, J_pose2, J_vel2, J_grav, J_scale);

    // Swap bias columns to ConstantBias tangent order (acc|gyro)
    Eigen::Matrix<double, 9, 6> J_bias1_gtsam;
    J_bias1_gtsam.block<9, 3>(0, 0) = J_bias1.block<9, 3>(0, 3);
    J_bias1_gtsam.block<9, 3>(0, 3) = J_bias1.block<9, 3>(0, 0);

    const auto res = computeResidual(Twb1, v1, bias1, Twb2, v2, gravRot, logScale);
    const Eigen::LLT<Matrix9d> llt(information_);
    const Eigen::Matrix<double, 9, 9> sqrtInfo = llt.matrixU();

    std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
    terms.emplace_back(keys()[0], sqrtInfo * J_pose1);
    terms.emplace_back(keys()[1], sqrtInfo * J_vel1);
    terms.emplace_back(keys()[2], sqrtInfo * J_bias1_gtsam);
    terms.emplace_back(keys()[3], sqrtInfo * J_pose2);
    terms.emplace_back(keys()[4], sqrtInfo * J_vel2);
    terms.emplace_back(keys()[5], sqrtInfo * J_grav);
    terms.emplace_back(keys()[6], sqrtInfo * J_scale);
    return boost::make_shared<gtsam::JacobianFactor>(terms, sqrtInfo * (-res));
}

// ─────────────────────────────────────────────────────────────────────────────
// PriorNavFactor
// ─────────────────────────────────────────────────────────────────────────────
PriorNavFactor::PriorNavFactor(
    const gtsam::Key& poseKey_, const gtsam::Key& velKey_,
    const gtsam::Key& biasKey_, const ConstraintPoseImu& c)
    : gtsam::NonlinearFactor(gtsam::KeyVector{poseKey_, velKey_, biasKey_}),
      Rwb_(c.Rwb), twb_(c.twb), vwb_(c.vwb), bg_(c.bg), ba_(c.ba),
      information_(c.H)
{}

double PriorNavFactor::error(const gtsam::Values& c) const
{
    const auto Twb  = c.at<gtsam::Pose3>(keys()[0]);
    const auto vel  = c.at<gtsam::Vector3>(keys()[1]);
    const auto bias = c.at<gtsam::imuBias::ConstantBias>(keys()[2]);

    const Eigen::Matrix3d Rcur = Twb.rotation().matrix();
    const Eigen::Vector3d tcur = Twb.translation();
    const auto& ba_cur = bias.accelerometer();
    const auto& bg_cur = bias.gyroscope();

    // 15-D residual: [LogSO3(R_prior^T * R_cur); t-t_prior; v-v_prior; bg-bg_prior; ba-ba_prior]
    Vector15d res;
    res.segment<3>(0)  = LogSO3(Rwb_.transpose() * Rcur);
    res.segment<3>(3)  = Rwb_.transpose() * (tcur - twb_);
    res.segment<3>(6)  = vel - vwb_;
    res.segment<3>(9)  = Eigen::Vector3d(bg_cur.x(), bg_cur.y(), bg_cur.z()) - bg_;
    res.segment<3>(12) = Eigen::Vector3d(ba_cur.x(), ba_cur.y(), ba_cur.z()) - ba_;

    return 0.5 * res.transpose() * information_ * res;
}

void PriorNavFactor::computeJacobians(
    const gtsam::Pose3& Twb, const Eigen::Vector3d& vel,
    const gtsam::imuBias::ConstantBias& bias,
    Eigen::Matrix<double, 15, 6>&  J_pose,
    Eigen::Matrix<double, 15, 3>&  J_vel,
    Eigen::Matrix<double, 15, 6>&  J_bias) const
{
    const Eigen::Matrix3d Rcur = Twb.rotation().matrix();
    const Eigen::Vector3d er   = LogSO3(Rwb_.transpose() * Rcur);

    J_pose.setZero();
    J_pose.block<3, 3>(0, 0) = InverseRightJacobianSO3(er);
    J_pose.block<3, 3>(3, 3) = Rwb_.transpose() * Rcur;

    J_vel.setZero();
    J_vel.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity();

    // ConstantBias tangent: [acc_delta(3) | gyro_delta(3)]
    J_bias.setZero();
    J_bias.block<3, 3>(9,  3) = Eigen::Matrix3d::Identity();  // gyro (rows 9-11, cols 3-5)
    J_bias.block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();  // acc  (rows 12-14, cols 0-2)
}

boost::shared_ptr<gtsam::GaussianFactor> PriorNavFactor::linearize(const gtsam::Values& c) const
{
    const auto Twb  = c.at<gtsam::Pose3>(keys()[0]);
    const auto vel  = c.at<gtsam::Vector3>(keys()[1]);
    const auto bias = c.at<gtsam::imuBias::ConstantBias>(keys()[2]);

    Eigen::Matrix<double, 15, 6> J_pose;
    Eigen::Matrix<double, 15, 3> J_vel;
    Eigen::Matrix<double, 15, 6> J_bias;
    computeJacobians(Twb, vel, bias, J_pose, J_vel, J_bias);

    const Eigen::Matrix3d Rcur = Twb.rotation().matrix();
    const Eigen::Vector3d tcur = Twb.translation();
    const auto& ba_cur = bias.accelerometer();
    const auto& bg_cur = bias.gyroscope();

    Vector15d res;
    res.segment<3>(0)  = LogSO3(Rwb_.transpose() * Rcur);
    res.segment<3>(3)  = Rwb_.transpose() * (tcur - twb_);
    res.segment<3>(6)  = vel - vwb_;
    res.segment<3>(9)  = Eigen::Vector3d(bg_cur.x(), bg_cur.y(), bg_cur.z()) - bg_;
    res.segment<3>(12) = Eigen::Vector3d(ba_cur.x(), ba_cur.y(), ba_cur.z()) - ba_;

    const Eigen::LLT<Matrix15d> llt(information_);
    const Eigen::Matrix<double, 15, 15> sqrtInfo = llt.matrixU();

    std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms;
    terms.emplace_back(keys()[0], sqrtInfo * J_pose);
    terms.emplace_back(keys()[1], sqrtInfo * J_vel);
    terms.emplace_back(keys()[2], sqrtInfo * J_bias);
    return boost::make_shared<gtsam::JacobianFactor>(terms, sqrtInfo * (-res));
}

// ─────────────────────────────────────────────────────────────────────────────
// Sim3ProjectionFactor
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Vector Sim3ProjectionFactor::evaluateError(
    const gtsam::Similarity3& S12,
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
        if (H) *H = Eigen::Matrix<double, 2, 7>::Zero();
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
gtsam::Vector InverseSim3ProjectionFactor::evaluateError(
    const gtsam::Similarity3& S12,
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
        if (H) *H = Eigen::Matrix<double, 2, 7>::Zero();
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
gtsam::Vector FourDOFBetweenFactor::evaluateError(
    const gtsam::Pose3& Ti, const gtsam::Pose3& Tj,
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

        if (H1) *H1 = J1;
        if (H2) *H2 = J2;
    }

    return err;
}

}  // namespace ORB_SLAM3
