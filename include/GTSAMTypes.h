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

#ifndef GTSAMTYPES_H
#define GTSAMTYPES_H

// ── GTSAM ────────────────────────────────────────────────────────────────────
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// ── Eigen / Sophus ───────────────────────────────────────────────────────────
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

// ── Project ──────────────────────────────────────────────────────────────────
#include "CameraModels/GeometricCamera.h"
#include "ImuTypes.h"

#include <cmath>

namespace ORB_SLAM3
{

// ─────────────────────────────────────────────────────────────────────────────
// Eigen convenience typedefs (shared with the old G2oTypes / Optimizer code)
// ─────────────────────────────────────────────────────────────────────────────
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 15, 1> Vector15d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 15, 15> Matrix15d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;

// ─────────────────────────────────────────────────────────────────────────────
// SO(3) / Lie-algebra utilities  (formerly in G2oTypes.h / G2oTypes.cc)
// ─────────────────────────────────────────────────────────────────────────────
Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w);
Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R);
Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d& v);
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z);
Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v);
Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z);
Eigen::Matrix3d Skew(const Eigen::Vector3d& w);

template <typename T = double>
Eigen::Matrix<T, 3, 3> NormalizeRotation(const Eigen::Matrix<T, 3, 3>& R)
{
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

// ─────────────────────────────────────────────────────────────────────────────
// ConstraintPoseImu  (formerly in G2oTypes.h — pure Eigen, no g2o dependency)
// Used by PoseInertialOptimization* to carry the marginalized prior.
// ─────────────────────────────────────────────────────────────────────────────
class ConstraintPoseImu
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintPoseImu(const Eigen::Matrix3d& Rwb_, const Eigen::Vector3d& twb_, const Eigen::Vector3d& vwb_,
                      const Eigen::Vector3d& bg_, const Eigen::Vector3d& ba_, const Matrix15d& H_)
        : Rwb(Rwb_), twb(twb_), vwb(vwb_), bg(bg_), ba(ba_), H(H_)
    {
        H = (H + H.transpose()) / 2.0;
        Eigen::SelfAdjointEigenSolver<Matrix15d> es(H);
        Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 15; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0.0;
        H = es.eigenvectors() * eigs.asDiagonal() * es.eigenvectors().transpose();
    }

    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d vwb;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
    Matrix15d H;
};

// ─────────────────────────────────────────────────────────────────────────────
// Key-generation utilities
// ─────────────────────────────────────────────────────────────────────────────
//  'x' — standard camera pose  (Tcw stored inverted as Twc in Values)
//  'p' — IMU body pose         (Twb)
//  'f' — 4-DoF pose            (Tcw convention)
//  'l' — map-point (Point3)
//  'v' — velocity
//  'b' — combined IMU bias     (imuBias::ConstantBias)
//  'g' — gravity rotation      (Rot3)
//  's' — log-scale             (double)
//  'S' — Sim3 pose             (Similarity3)

inline gtsam::Key poseKey(uint32_t id)
{
    return gtsam::Symbol('x', id);
}
inline gtsam::Key imuPoseKey(uint32_t id)
{
    return gtsam::Symbol('p', id);
}
inline gtsam::Key fourDofKey(uint32_t id)
{
    return gtsam::Symbol('f', id);
}
inline gtsam::Key pointKey(uint32_t id)
{
    return gtsam::Symbol('l', id);
}
inline gtsam::Key velKey(uint32_t id)
{
    return gtsam::Symbol('v', id);
}
inline gtsam::Key biasKey(uint32_t id)
{
    return gtsam::Symbol('b', id);
}
inline gtsam::Key gravKey()
{
    return gtsam::Symbol('g', 0);
}
inline gtsam::Key scaleKey()
{
    return gtsam::Symbol('s', 0);
}
inline gtsam::Key sim3Key(uint32_t id)
{
    return gtsam::Symbol('S', id);
}

// ─────────────────────────────────────────────────────────────────────────────
// Conversion helpers (declarations — implemented in GTSAMTypes.cc)
// ─────────────────────────────────────────────────────────────────────────────
gtsam::Pose3 sophusToGTSAMPose(const Sophus::SE3f& T);
Sophus::SE3f gtsamToSophusPose(const gtsam::Pose3& P);
gtsam::imuBias::ConstantBias toGTSAMBias(const IMU::Bias& b);
IMU::Bias fromGTSAMBias(const gtsam::imuBias::ConstantBias& cb);
gtsam::Similarity3 toGTSAMSim3(const Sophus::Sim3f& S);
Sophus::Sim3f fromGTSAMSim3(const gtsam::Similarity3& S);

/// Returns a pinhole calibration for the left camera (asserts pinhole type).
gtsam::Cal3_S2 toGTSAMCal(const GeometricCamera* pCam);

/// Returns a stereo calibration (asserts pinhole type; bf = baseline × fx).
gtsam::Cal3_S2Stereo toGTSAMStereoCal(const GeometricCamera* pCam, double bf);

// ─────────────────────────────────────────────────────────────────────────────
// Noise-model helpers
// ─────────────────────────────────────────────────────────────────────────────
/// Huber-wrapped isotropic noise: Precision(dim, invSigma2) with Huber(sqrt(chi2)).
gtsam::SharedNoiseModel makeHuberNoise(int dim, double chi2Threshold, double invSigma2);

/// Plain isotropic precision noise (no robust kernel).
gtsam::SharedNoiseModel makeIsotropicNoise(int dim, double invSigma2);

// ─────────────────────────────────────────────────────────────────────────────
// MonoOnlyPoseFactor
//   Replaces EdgeSE3ProjectXYZOnlyPose (standard) and EdgeMonoOnlyPose (IMU).
//   Unary factor on Pose3 (interpreted as Twb; body = camera when Tbc = I).
//   Supports both pinhole and fisheye via GeometricCamera::project().
// ─────────────────────────────────────────────────────────────────────────────
class MonoOnlyPoseFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

    MonoOnlyPoseFactor(const gtsam::Key& poseKey, const Eigen::Vector3d& Xw, const Eigen::Vector2d& obs,
                       const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera,
                       const gtsam::Pose3& Tbc = gtsam::Pose3())
        : Base(noise, poseKey), Xw_(Xw), obs_(obs), pCamera_(pCamera), Tbc_(Tbc)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& Twb,
                                boost::optional<gtsam::Matrix&> H = boost::none) const override;

    /// Returns true if the 3-D point projects in front of the camera.
    bool isDepthPositive(const gtsam::Pose3& Twb) const;

private:
    Eigen::Vector3d Xw_;
    Eigen::Vector2d obs_;
    GeometricCamera* pCamera_;
    gtsam::Pose3 Tbc_;
};

// ─────────────────────────────────────────────────────────────────────────────
// StereoOnlyPoseFactor
//   Replaces EdgeStereoSE3ProjectXYZOnlyPose and EdgeStereoOnlyPose.
//   Unary factor on Pose3.  Measurement = [ul, v, ur] (left u, v, right u).
// ─────────────────────────────────────────────────────────────────────────────
class StereoOnlyPoseFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

    StereoOnlyPoseFactor(const gtsam::Key& poseKey, const Eigen::Vector3d& Xw,
                         const Eigen::Vector3d& obs,  // [ul, v, ur]
                         double bf, const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera,
                         const gtsam::Pose3& Tbc = gtsam::Pose3())
        : Base(noise, poseKey), Xw_(Xw), obs_(obs), bf_(bf), pCamera_(pCamera), Tbc_(Tbc)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& Twb,
                                boost::optional<gtsam::Matrix&> H = boost::none) const override;

    bool isDepthPositive(const gtsam::Pose3& Twb) const;

private:
    Eigen::Vector3d Xw_;
    Eigen::Vector3d obs_;  // [ul, v, ur]
    double bf_;
    GeometricCamera* pCamera_;
    gtsam::Pose3 Tbc_;
};

// ─────────────────────────────────────────────────────────────────────────────
// FisheyeProjectionFactor
//   Replaces EdgeMono / EdgeSE3ProjectXYZ for fisheye (Metashape) cameras.
//   Binary factor: Pose3(Twb) × Point3(Xw) → ℝ².
//   For pinhole cameras use gtsam::GenericProjectionFactor instead.
// ─────────────────────────────────────────────────────────────────────────────
class FisheyeProjectionFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;

    FisheyeProjectionFactor(const gtsam::Key& poseKey, const gtsam::Key& pointKey, const Eigen::Vector2d& obs,
                            const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera,
                            const gtsam::Pose3& Tbc = gtsam::Pose3())
        : Base(noise, poseKey, pointKey), obs_(obs), pCamera_(pCamera), Tbc_(Tbc)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& Twb, const gtsam::Point3& Xw,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

    bool isDepthPositive(const gtsam::Pose3& Twb, const gtsam::Point3& Xw) const;

private:
    Eigen::Vector2d obs_;
    GeometricCamera* pCamera_;
    gtsam::Pose3 Tbc_;
};

// ─────────────────────────────────────────────────────────────────────────────
// FisheyeStereoFactor
//   Replaces EdgeStereo for fisheye cameras.
//   Binary factor: Pose3(Twb) × Point3(Xw) → ℝ³  ([ul, v, ur]).
// ─────────────────────────────────────────────────────────────────────────────
class FisheyeStereoFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;

    FisheyeStereoFactor(const gtsam::Key& poseKey, const gtsam::Key& pointKey,
                        const Eigen::Vector3d& obs,  // [ul, v, ur]
                        double bf, const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera,
                        const gtsam::Pose3& Tbc = gtsam::Pose3())
        : Base(noise, poseKey, pointKey), obs_(obs), bf_(bf), pCamera_(pCamera), Tbc_(Tbc)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& Twb, const gtsam::Point3& Xw,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

    bool isDepthPositive(const gtsam::Pose3& Twb, const gtsam::Point3& Xw) const;

private:
    Eigen::Vector3d obs_;
    double bf_;
    GeometricCamera* pCamera_;
    gtsam::Pose3 Tbc_;
};

// ─────────────────────────────────────────────────────────────────────────────
// InertialFactor
//   Replaces EdgeInertial (6-vertex multi-edge in g2o).
//   Connects: pose1(Twb), vel1, bias1(ConstantBias), pose2(Twb), vel2.
//   Replicates EdgeInertial::computeError() and linearizeOplus() exactly.
// ─────────────────────────────────────────────────────────────────────────────
class InertialFactor : public gtsam::NonlinearFactor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // keys order: pose1, vel1, bias1, pose2, vel2
    InertialFactor(const gtsam::Key& pose1Key, const gtsam::Key& vel1Key, const gtsam::Key& bias1Key,
                   const gtsam::Key& pose2Key, const gtsam::Key& vel2Key, IMU::Preintegrated* pInt);

    double error(const gtsam::Values& c) const override;

    boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& c) const override;

    size_t dim() const override { return 9; }

    /// Compute the 9-D inertial residual given explicit state values.
    Eigen::Matrix<double, 9, 1> computeResidual(const gtsam::Pose3& Twb1, const Eigen::Vector3d& v1,
                                                const gtsam::imuBias::ConstantBias& bias1, const gtsam::Pose3& Twb2,
                                                const Eigen::Vector3d& v2) const;

    // Jacobian blocks (same ordering as linearizeOplus in EdgeInertial)
    // [J_pose1 | J_vel1 | J_bias1_gyro | J_bias1_acc | J_pose2 | J_vel2]
    // sizes:   9×6       9×3              9×3              9×3        9×6      9×3
    void computeJacobians(const gtsam::Pose3& Twb1, const Eigen::Vector3d& v1,
                          const gtsam::imuBias::ConstantBias& bias1, const gtsam::Pose3& Twb2,
                          const Eigen::Vector3d& v2, Eigen::Matrix<double, 9, 6>& J_pose1,
                          Eigen::Matrix<double, 9, 3>& J_vel1,
                          Eigen::Matrix<double, 9, 6>& J_bias1,  // [gyro_cols | acc_cols]
                          Eigen::Matrix<double, 9, 6>& J_pose2, Eigen::Matrix<double, 9, 3>& J_vel2) const;

private:
    // Preintegration Jacobians (double-precision copies)
    const Eigen::Matrix3d JRg_, JVg_, JPg_, JVa_, JPa_;
    IMU::Preintegrated* mpInt_;
    const double dt_;
    const Eigen::Vector3d g_;  // [0, 0, -9.81]
    Matrix9d information_;
};

// ─────────────────────────────────────────────────────────────────────────────
// InertialGSFactor
//   Replaces EdgeInertialGS (8-vertex multi-edge in g2o).
//   Adds gravity Rot3 and log-scale double to InertialFactor.
//   keys: pose1, vel1, bias1, pose2, vel2, gravRot, logScale
// ─────────────────────────────────────────────────────────────────────────────
class InertialGSFactor : public gtsam::NonlinearFactor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InertialGSFactor(const gtsam::Key& pose1Key, const gtsam::Key& vel1Key, const gtsam::Key& bias1Key,
                     const gtsam::Key& pose2Key, const gtsam::Key& vel2Key, const gtsam::Key& gravKey,
                     const gtsam::Key& scaleKey, IMU::Preintegrated* pInt);

    double error(const gtsam::Values& c) const override;

    boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& c) const override;

    size_t dim() const override { return 9; }

    Eigen::Matrix<double, 9, 1> computeResidual(const gtsam::Pose3& Twb1, const Eigen::Vector3d& v1,
                                                const gtsam::imuBias::ConstantBias& bias1, const gtsam::Pose3& Twb2,
                                                const Eigen::Vector3d& v2, const gtsam::Rot3& gravRot,
                                                double logScale) const;

    // Jacobian blocks — same ordering as linearizeOplus in EdgeInertialGS
    // [J_pose1|J_vel1|J_bias1_gyro|J_bias1_acc|J_pose2|J_vel2|J_grav|J_scale]
    void computeJacobians(const gtsam::Pose3& Twb1, const Eigen::Vector3d& v1,
                          const gtsam::imuBias::ConstantBias& bias1, const gtsam::Pose3& Twb2,
                          const Eigen::Vector3d& v2, const gtsam::Rot3& gravRot, double logScale,
                          Eigen::Matrix<double, 9, 6>& J_pose1, Eigen::Matrix<double, 9, 3>& J_vel1,
                          Eigen::Matrix<double, 9, 6>& J_bias1, Eigen::Matrix<double, 9, 6>& J_pose2,
                          Eigen::Matrix<double, 9, 3>& J_vel2,
                          Eigen::Matrix<double, 9, 2>& J_grav,  // 3rd column zeroed (yaw-around-g)
                          Eigen::Matrix<double, 9, 1>& J_scale) const;

private:
    const Eigen::Matrix3d JRg_, JVg_, JPg_, JVa_, JPa_;
    IMU::Preintegrated* mpInt_;
    const double dt_;
    const Eigen::Vector3d gI_;  // [0, 0, -GRAVITY_VALUE]
    Matrix9d information_;
};

// ─────────────────────────────────────────────────────────────────────────────
// PriorNavFactor
//   Replaces EdgePriorPoseImu (4-vertex, 15-D joint prior).
//   3 keys: pose(Twb), vel(Vector3), bias(ConstantBias).
//   The 15×15 information matrix preserves cross-covariance terms.
// ─────────────────────────────────────────────────────────────────────────────
class PriorNavFactor : public gtsam::NonlinearFactor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Construct from an existing ConstraintPoseImu (marginalization result).
    PriorNavFactor(const gtsam::Key& poseKey, const gtsam::Key& velKey, const gtsam::Key& biasKey,
                   const ConstraintPoseImu& c);

    double error(const gtsam::Values& c) const override;

    boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& c) const override;

    size_t dim() const override { return 15; }

    // Jacobian blocks for external Hessian assembly.
    void computeJacobians(const gtsam::Pose3& Twb, const Eigen::Vector3d& vel, const gtsam::imuBias::ConstantBias& bias,
                          Eigen::Matrix<double, 15, 6>& J_pose, Eigen::Matrix<double, 15, 3>& J_vel,
                          Eigen::Matrix<double, 15, 6>& J_bias) const;

private:
    Eigen::Matrix3d Rwb_;
    Eigen::Vector3d twb_, vwb_, bg_, ba_;
    Matrix15d information_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Sim3ProjectionFactor
//   Replaces EdgeSim3ProjectXYZ.
//   Unary factor on Similarity3.  3-D point is fixed (stored in factor).
// ─────────────────────────────────────────────────────────────────────────────
class Sim3ProjectionFactor : public gtsam::NoiseModelFactorN<gtsam::Similarity3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Similarity3>;

    Sim3ProjectionFactor(const gtsam::Key& sim3Key, const Eigen::Vector3d& P3Dc, const Eigen::Vector2d& obs,
                         const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera)
        : Base(noise, sim3Key), P3Dc_(P3Dc), obs_(obs), pCamera_(pCamera)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Similarity3& S12,
                                boost::optional<gtsam::Matrix&> H = boost::none) const override;

private:
    Eigen::Vector3d P3Dc_;  // fixed point in camera-1 frame
    Eigen::Vector2d obs_;
    GeometricCamera* pCamera_;
};

// ─────────────────────────────────────────────────────────────────────────────
// InverseSim3ProjectionFactor
//   Replaces EdgeInverseSim3ProjectXYZ.
//   Unary factor on Similarity3.  Applies S12.inverse() before projecting.
// ─────────────────────────────────────────────────────────────────────────────
class InverseSim3ProjectionFactor : public gtsam::NoiseModelFactorN<gtsam::Similarity3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Similarity3>;

    InverseSim3ProjectionFactor(const gtsam::Key& sim3Key, const Eigen::Vector3d& P3Dc2, const Eigen::Vector2d& obs,
                                const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera)
        : Base(noise, sim3Key), P3Dc2_(P3Dc2), obs_(obs), pCamera_(pCamera)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Similarity3& S12,
                                boost::optional<gtsam::Matrix&> H = boost::none) const override;

private:
    Eigen::Vector3d P3Dc2_;  // fixed point in camera-2 frame
    Eigen::Vector2d obs_;
    GeometricCamera* pCamera_;
};

// ─────────────────────────────────────────────────────────────────────────────
// FourDOFBetweenFactor
//   Replaces Edge4DoF.  Binary factor between two Pose3 variables.
//   Convention: Pose3 stores Tcw (world-to-camera) in this graph.
//   Roll and pitch are anchored via PriorFactor<Pose3> per KF.
// ─────────────────────────────────────────────────────────────────────────────
class FourDOFBetweenFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>;

    FourDOFBetweenFactor(const gtsam::Key& keyI, const gtsam::Key& keyJ, const Eigen::Matrix3d& dRij,
                         const Eigen::Vector3d& dtij, const gtsam::SharedNoiseModel& noise)
        : Base(noise, keyI, keyJ), dRij_(dRij), dtij_(dtij)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& Ti, const gtsam::Pose3& Tj,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

private:
    Eigen::Matrix3d dRij_;
    Eigen::Vector3d dtij_;
};

}  // namespace ORB_SLAM3

#endif  // GTSAMTYPES_H
