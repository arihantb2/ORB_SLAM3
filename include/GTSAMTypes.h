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
// Key-generation utilities
// ─────────────────────────────────────────────────────────────────────────────
//  'x' — standard camera pose  (Tcw stored inverted as Twc in Values)
//  'f' — 4-DoF pose            (Tcw convention)
//  'l' — map-point (Point3)
//  's' — log-scale             (double)
//  'S' — Sim3 pose             (Similarity3)

inline gtsam::Key poseKey(uint32_t id)
{
    return gtsam::Symbol('x', id);
}
inline gtsam::Key fourDofKey(uint32_t id)
{
    return gtsam::Symbol('f', id);
}
inline gtsam::Key pointKey(uint32_t id)
{
    return gtsam::Symbol('l', id);
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
// PinholeMonoPoseTcwFactor / PinholeStereoPoseTcwFactor
//   Pinhole-only unary factors on Pose3, interpreted as Tcw (world-to-camera).
//   Used by visual-only PoseOptimization (no IMU body frame).
// ─────────────────────────────────────────────────────────────────────────────
class PinholeMonoPoseTcwFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

    PinholeMonoPoseTcwFactor(const gtsam::Key& poseKey, const Eigen::Vector3d& Xw, const Eigen::Vector2d& obs,
                             const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera)
        : Base(noise, poseKey), Xw_(Xw), obs_(obs), pCamera_(pCamera)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& Tcw,
                                boost::optional<gtsam::Matrix&> H = boost::none) const override;

private:
    Eigen::Vector3d Xw_;
    Eigen::Vector2d obs_;
    GeometricCamera* pCamera_;
};

class PinholeStereoPoseTcwFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

    PinholeStereoPoseTcwFactor(const gtsam::Key& poseKey, const Eigen::Vector3d& Xw,
                               const Eigen::Vector3d& obs,  // [ul, v, ur]
                               double bf, const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera)
        : Base(noise, poseKey), Xw_(Xw), obs_(obs), bf_(bf), pCamera_(pCamera)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& Tcw,
                                boost::optional<gtsam::Matrix&> H = boost::none) const override;

private:
    Eigen::Vector3d Xw_;
    Eigen::Vector3d obs_;  // [ul, v, ur]
    double bf_;
    GeometricCamera* pCamera_;
};

// ─────────────────────────────────────────────────────────────────────────────
// PinholeMonoTcwFactor / PinholeStereoTcwFactor
//   Pinhole-only binary factors: Pose3(Tcw) × Point3(Xw) → ℝ² / ℝ³.
//   Used by LocalBundleAdjustment with Tcw-parametrized camera poses.
// ─────────────────────────────────────────────────────────────────────────────
class PinholeMonoTcwFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;

    PinholeMonoTcwFactor(const gtsam::Key& poseKey, const gtsam::Key& pointKey, const Eigen::Vector2d& obs,
                         const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera)
        : Base(noise, poseKey, pointKey), obs_(obs), pCamera_(pCamera)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& Tcw, const gtsam::Point3& Xw,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

private:
    Eigen::Vector2d obs_;
    GeometricCamera* pCamera_;
};

class PinholeStereoTcwFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Point3>;

    PinholeStereoTcwFactor(const gtsam::Key& poseKey, const gtsam::Key& pointKey,
                           const Eigen::Vector3d& obs,  // [ul, v, ur]
                           double bf, const gtsam::SharedNoiseModel& noise, GeometricCamera* pCamera)
        : Base(noise, poseKey, pointKey), obs_(obs), bf_(bf), pCamera_(pCamera)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& Tcw, const gtsam::Point3& Xw,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

private:
    Eigen::Vector3d obs_;
    double bf_;
    GeometricCamera* pCamera_;
};

// BetweenFactorTcw
class BetweenFactorTcw : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
private:
    gtsam::Pose3 measured_;  // T_c1_c2

public:
    BetweenFactorTcw(gtsam::Key key1, gtsam::Key key2, const gtsam::Pose3& measured,
                     const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key1, key2), measured_(measured)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& X1, const gtsam::Pose3& X2,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        // X1 == T_c1_w
        // X2 == T_c2_w

        // If the solver needs Jacobians, we compute the math and track the derivatives
        if (H1 || H2)
        {
            gtsam::Matrix H_invX2, H_comp_X1, H_comp_invX2, H_local;

            // 1. Invert X2 (T_c2_w -> T_w_c2)
            gtsam::Pose3 invX2 = X2.inverse(H_invX2);

            // 2. Compose X1 and invX2 to predict T_c1_c2
            gtsam::Pose3 hx = X1.compose(invX2, H_comp_X1, H_comp_invX2);

            // 3. Calculate the error via LogMap
            gtsam::Vector error = measured_.localCoordinates(hx, boost::none, H_local);

            // 4. Apply the chain rule
            if (H1)
            {
                *H1 = H_local * H_comp_X1;
            }
            if (H2)
            {
                *H2 = H_local * H_comp_invX2 * H_invX2;
            }
            return error;
        }
        // If Jacobians are not needed, we run the fast path
        else
        {
            gtsam::Pose3 invX2 = X2.inverse();
            gtsam::Pose3 hx = X1.compose(invX2);
            return measured_.localCoordinates(hx);
        }
    }

    // (Include clone(), print(), and equals() methods here as shown previously)
};

// PriorFactorTcw
class PriorFactorTcw : public gtsam::NoiseModelFactor1<gtsam::Pose3>  // Swapped to Factor1
{
private:
    gtsam::Pose3 measured_;  // T_w_cPrior

public:
    PriorFactorTcw(const gtsam::Key& poseKey, const gtsam::Pose3& measured, const gtsam::SharedNoiseModel& noise)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise, poseKey), measured_(measured)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& X, boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        // X = Tcw
        if (H)
        {
            gtsam::Matrix H_invX, H_local;

            // 1. Invert the state to match the measurement frame (T_w_c)
            gtsam::Pose3 hx = X.inverse(H_invX);

            // 2. Compute error. boost::none ignores the derivative w.r.t the constant measurement.
            gtsam::Vector error = measured_.localCoordinates(hx, boost::none, H_local);

            // 3. Apply chain rule
            *H = H_local * H_invX;

            return error;
        }
        else
        {
            // Fast path when Jacobians are not needed by the solver
            return measured_.localCoordinates(X.inverse());
        }
    }
};

// ScaleFactorTcw
class ScaleFactorTcw : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>
{
private:
    double measured_;  // measured translation between two keyframe pose priors

public:
    ScaleFactorTcw(const gtsam::Key& poseKey1, const gtsam::Key& poseKey2, double measured,
                   const gtsam::SharedNoiseModel& noise)
        : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>(noise, poseKey1, poseKey2), measured_(measured)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& X1, const gtsam::Pose3& X2,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        // 1. Invert X2 and Compose, tracking Jacobians
        gtsam::Matrix H_invX2, H_comp_X1, H_comp_invX2;
        gtsam::Pose3 invX2 = X2.inverse(H_invX2);
        gtsam::Pose3 rel_pose = X1.compose(invX2, H_comp_X1, H_comp_invX2);

        // 2. Extract translation and track its Jacobian w.r.t the relative pose
        gtsam::Matrix H_t_rel;
        gtsam::Point3 t = rel_pose.translation(H_t_rel);

        // 3. Compute the norm (hx)
        double hx = t.norm();

        // 4. Compute the error
        gtsam::Vector1 error;
        error(0) = 1.0 - (hx / measured_);

        // 5. Apply the Chain Rule if Jacobians are requested
        if (H1 || H2)
        {
            // Derivative of error w.r.t translation vector (1x3 matrix)
            gtsam::Matrix13 H_e_t;

            // Safeguard against division by zero if translation is perfectly zero
            if (hx > 1e-7)
            {
                H_e_t = -(1.0 / (measured_ * hx)) * t.transpose();
            }
            else
            {
                H_e_t = gtsam::Matrix13::Zero();
            }

            if (H1)
            {
                // H1 (1x6) = H_e_t (1x3) * H_t_rel (3x6) * H_comp_X1 (6x6)
                *H1 = H_e_t * H_t_rel * H_comp_X1;
            }
            if (H2)
            {
                // H2 (1x6) = H_e_t (1x3) * H_t_rel (3x6) * H_comp_invX2 (6x6) * H_invX2 (6x6)
                *H2 = H_e_t * H_t_rel * H_comp_invX2 * H_invX2;
            }
        }

        return error;
    }
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
