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

#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/types/sba/types_sba.h"

#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include <math.h>

namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class GeometricCamera;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w);

Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R);

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d& v);
Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v);
Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z);

Eigen::Matrix3d Skew(const Eigen::Vector3d& w);
Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z);

template <typename T = double>
Eigen::Matrix<T, 3, 3> NormalizeRotation(const Eigen::Matrix<T, 3, 3>& R)
{
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

class ImuCamPose
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuCamPose() {}
    ImuCamPose(KeyFrame* pKF);
    ImuCamPose(Frame* pF);
    ImuCamPose(Eigen::Matrix3d& _Rwc, Eigen::Vector3d& _twc, KeyFrame* pKF);

    void SetParam(const std::vector<Eigen::Matrix3d>& _Rcw, const std::vector<Eigen::Vector3d>& _tcw,
                  const std::vector<Eigen::Matrix3d>& _Rbc, const std::vector<Eigen::Vector3d>& _tbc,
                  const double& _bf);

    void Update(const double* pu);
    Eigen::Vector2d Project(const Eigen::Vector3d& Xw, int cam_idx = 0) const;        // Mono
    Eigen::Vector3d ProjectStereo(const Eigen::Vector3d& Xw, int cam_idx = 0) const;  // Stereo
    bool isDepthPositive(const Eigen::Vector3d& Xw, int cam_idx = 0) const;

public:
    // Body pose (= camera[0] pose when no IMU; kept for Update() math)
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;

    // For set of cameras
    std::vector<Eigen::Matrix3d> Rcw;
    std::vector<Eigen::Vector3d> tcw;
    std::vector<Eigen::Matrix3d> Rcb, Rbc;
    std::vector<Eigen::Vector3d> tcb, tbc;
    double bf;
    std::vector<GeometricCamera*> pCamera;

    int its;
};

class InvDepthPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    InvDepthPoint() {}
    InvDepthPoint(double _rho, double _u, double _v, KeyFrame* pHostKF);

    void Update(const double* pu);

    double rho;
    double u, v;  // they are not variables, observation in the host frame

    double fx, fy, cx, cy, bf;  // from host frame

    int its;
};

// Optimizable parameters are IMU pose
class VertexPose : public g2o::BaseVertex<6, ImuCamPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() {}
    VertexPose(KeyFrame* pKF) { setEstimate(ImuCamPose(pKF)); }
    VertexPose(Frame* pF) { setEstimate(ImuCamPose(pF)); }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double* update_)
    {
        _estimate.Update(update_);
        updateCache();
    }
};


// Inverse depth point (just one parameter, inverse depth at the host frame)
class VertexInvDepth : public g2o::BaseVertex<1, InvDepthPoint>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexInvDepth() {}
    VertexInvDepth(double invDepth, double u, double v, KeyFrame* pHostKF)
    {
        setEstimate(InvDepthPoint(invDepth, u, v, pHostKF));
    }

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double* update_)
    {
        _estimate.Update(update_);
        updateCache();
    }
};

class EdgeMono : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMono(int cam_idx_ = 0) : cam_idx(cam_idx_) {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError()
    {
        const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
        const Eigen::Vector2d obs(_measurement);
        _error = obs - VPose->estimate().Project(VPoint->estimate(), cam_idx);
    }

    virtual void linearizeOplus();

    bool isDepthPositive()
    {
        const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
        return VPose->estimate().isDepthPositive(VPoint->estimate(), cam_idx);
    }

    Eigen::Matrix<double, 2, 9> GetJacobian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 2, 9> J;
        J.block<2, 3>(0, 0) = _jacobianOplusXi;
        J.block<2, 6>(0, 3) = _jacobianOplusXj;
        return J;
    }

    Eigen::Matrix<double, 9, 9> GetHessian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 2, 9> J;
        J.block<2, 3>(0, 0) = _jacobianOplusXi;
        J.block<2, 6>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }

public:
    const int cam_idx;
};

class EdgeMonoOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoOnlyPose(const Eigen::Vector3f& Xw_, int cam_idx_ = 0) : Xw(Xw_.cast<double>()), cam_idx(cam_idx_) {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError()
    {
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        const Eigen::Vector2d obs(_measurement);
        _error = obs - VPose->estimate().Project(Xw, cam_idx);
    }

    virtual void linearizeOplus();

    bool isDepthPositive()
    {
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        return VPose->estimate().isDepthPositive(Xw, cam_idx);
    }

    Eigen::Matrix<double, 6, 6> GetHessian()
    {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

public:
    const Eigen::Vector3d Xw;
    const int cam_idx;
};

class EdgeStereo : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereo(int cam_idx_ = 0) : cam_idx(cam_idx_) {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError()
    {
        const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
        const Eigen::Vector3d obs(_measurement);
        _error = obs - VPose->estimate().ProjectStereo(VPoint->estimate(), cam_idx);
    }

    virtual void linearizeOplus();

    Eigen::Matrix<double, 3, 9> GetJacobian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 9> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 6>(0, 3) = _jacobianOplusXj;
        return J;
    }

    Eigen::Matrix<double, 9, 9> GetHessian()
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 9> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 6>(0, 3) = _jacobianOplusXj;
        return J.transpose() * information() * J;
    }

public:
    const int cam_idx;
};

class EdgeStereoOnlyPose : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereoOnlyPose(const Eigen::Vector3f& Xw_, int cam_idx_ = 0) : Xw(Xw_.cast<double>()), cam_idx(cam_idx_) {}

    virtual bool read(std::istream& is) { return false; }
    virtual bool write(std::ostream& os) const { return false; }

    void computeError()
    {
        const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);
        const Eigen::Vector3d obs(_measurement);
        _error = obs - VPose->estimate().ProjectStereo(Xw, cam_idx);
    }

    virtual void linearizeOplus();

    Eigen::Matrix<double, 6, 6> GetHessian()
    {
        linearizeOplus();
        return _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
    }

public:
    const Eigen::Vector3d Xw;  // 3D point coordinates
    const int cam_idx;
};


}  // namespace ORB_SLAM3

#endif  // G2OTYPES_H
