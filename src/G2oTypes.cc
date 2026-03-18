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

#include "G2oTypes.h"

#include "Converter.h"
#include "Frame.h"
#include "KeyFrame.h"
namespace ORB_SLAM3
{

ImuCamPose::ImuCamPose(KeyFrame* pKF) : its(0)
{
    const int num_cams = 1;
    tcw.resize(num_cams);
    Rcw.resize(num_cams);
    tcb.resize(num_cams);
    Rcb.resize(num_cams);
    Rbc.resize(num_cams);
    tbc.resize(num_cams);
    pCamera.resize(num_cams);

    tcw[0] = pKF->GetTranslation().cast<double>();
    Rcw[0] = pKF->GetRotation().cast<double>();
    // No IMU: body frame = camera frame, Tbc = Identity
    Rcb[0] = Eigen::Matrix3d::Identity();
    tcb[0] = Eigen::Vector3d::Zero();
    Rbc[0] = Eigen::Matrix3d::Identity();
    tbc[0] = Eigen::Vector3d::Zero();
    pCamera[0] = pKF->mpCamera;
    bf = pKF->mbf;

    Rwb = Rcw[0].transpose();
    twb = -Rwb * tcw[0];
}

ImuCamPose::ImuCamPose(Frame* pF) : its(0)
{
    const int num_cams = 1;
    tcw.resize(num_cams);
    Rcw.resize(num_cams);
    tcb.resize(num_cams);
    Rcb.resize(num_cams);
    Rbc.resize(num_cams);
    tbc.resize(num_cams);
    pCamera.resize(num_cams);

    tcw[0] = pF->GetPose().translation().cast<double>();
    Rcw[0] = pF->GetPose().rotationMatrix().cast<double>();
    // No IMU: body frame = camera frame, Tbc = Identity
    Rcb[0] = Eigen::Matrix3d::Identity();
    tcb[0] = Eigen::Vector3d::Zero();
    Rbc[0] = Eigen::Matrix3d::Identity();
    tbc[0] = Eigen::Vector3d::Zero();
    pCamera[0] = pF->mpCamera;
    bf = pF->mbf;

    Rwb = Rcw[0].transpose();
    twb = -Rwb * tcw[0];
}

ImuCamPose::ImuCamPose(Eigen::Matrix3d& _Rwc, Eigen::Vector3d& _twc, KeyFrame* pKF) : its(0)
{
    tcw.resize(1);
    Rcw.resize(1);
    tcb.resize(1);
    Rcb.resize(1);
    Rbc.resize(1);
    tbc.resize(1);
    pCamera.resize(1);

    // No IMU: body frame = camera frame, Tbc = Identity
    Rcb[0] = Eigen::Matrix3d::Identity();
    tcb[0] = Eigen::Vector3d::Zero();
    Rbc[0] = Eigen::Matrix3d::Identity();
    tbc[0] = Eigen::Vector3d::Zero();
    Rwb = _Rwc;
    twb = _twc;
    Rcw[0] = _Rwc.transpose();
    tcw[0] = -Rcw[0] * _twc;
    pCamera[0] = pKF->mpCamera;
    bf = pKF->mbf;
}

void ImuCamPose::SetParam(const std::vector<Eigen::Matrix3d>& _Rcw, const std::vector<Eigen::Vector3d>& _tcw,
                          const std::vector<Eigen::Matrix3d>& _Rbc, const std::vector<Eigen::Vector3d>& _tbc,
                          const double& _bf)
{
    Rbc = _Rbc;
    tbc = _tbc;
    Rcw = _Rcw;
    tcw = _tcw;
    const int num_cams = Rbc.size();
    Rcb.resize(num_cams);
    tcb.resize(num_cams);

    for (int i = 0; i < tcb.size(); i++)
    {
        Rcb[i] = Rbc[i].transpose();
        tcb[i] = -Rcb[i] * tbc[i];
    }
    Rwb = Rcw[0].transpose() * Rcb[0];
    twb = Rcw[0].transpose() * (tcb[0] - tcw[0]);

    bf = _bf;
}

Eigen::Vector2d ImuCamPose::Project(const Eigen::Vector3d& Xw, int cam_idx) const
{
    Eigen::Vector3d Xc = Rcw[cam_idx] * Xw + tcw[cam_idx];

    return pCamera[cam_idx]->project(Xc);
}

Eigen::Vector3d ImuCamPose::ProjectStereo(const Eigen::Vector3d& Xw, int cam_idx) const
{
    Eigen::Vector3d Pc = Rcw[cam_idx] * Xw + tcw[cam_idx];
    Eigen::Vector3d pc;
    double invZ = 1 / Pc(2);
    pc.head(2) = pCamera[cam_idx]->project(Pc);
    pc(2) = pc(0) - bf * invZ;
    return pc;
}

bool ImuCamPose::isDepthPositive(const Eigen::Vector3d& Xw, int cam_idx) const
{
    return (Rcw[cam_idx].row(2) * Xw + tcw[cam_idx](2)) > 0.0;
}

void ImuCamPose::Update(const double* pu)
{
    Eigen::Vector3d ur, ut;
    ur << pu[0], pu[1], pu[2];
    ut << pu[3], pu[4], pu[5];

    // Update body pose
    twb += Rwb * ut;
    Rwb = Rwb * ExpSO3(ur);

    // Normalize rotation after 5 updates
    its++;
    if (its >= 3)
    {
        NormalizeRotation(Rwb);
        its = 0;
    }

    // Update camera poses
    const Eigen::Matrix3d Rbw = Rwb.transpose();
    const Eigen::Vector3d tbw = -Rbw * twb;

    for (int i = 0; i < pCamera.size(); i++)
    {
        Rcw[i] = Rcb[i] * Rbw;
        tcw[i] = Rcb[i] * tbw + tcb[i];
    }
}

InvDepthPoint::InvDepthPoint(double _rho, double _u, double _v, KeyFrame* pHostKF)
    : u(_u), v(_v), rho(_rho), fx(pHostKF->fx), fy(pHostKF->fy), cx(pHostKF->cx), cy(pHostKF->cy), bf(pHostKF->mbf)
{
}

void InvDepthPoint::Update(const double* pu)
{
    rho += *pu;
}

bool VertexPose::read(std::istream& is)
{
    std::vector<Eigen::Matrix<double, 3, 3>> Rcw;
    std::vector<Eigen::Matrix<double, 3, 1>> tcw;
    std::vector<Eigen::Matrix<double, 3, 3>> Rbc;
    std::vector<Eigen::Matrix<double, 3, 1>> tbc;

    const int num_cams = _estimate.Rbc.size();
    for (int idx = 0; idx < num_cams; idx++)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
                is >> Rcw[idx](i, j);
        }
        for (int i = 0; i < 3; i++)
        {
            is >> tcw[idx](i);
        }

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
                is >> Rbc[idx](i, j);
        }
        for (int i = 0; i < 3; i++)
        {
            is >> tbc[idx](i);
        }

        float nextParam;
        for (size_t i = 0; i < _estimate.pCamera[idx]->size(); i++)
        {
            is >> nextParam;
            _estimate.pCamera[idx]->setParameter(nextParam, i);
        }
    }

    double bf;
    is >> bf;
    _estimate.SetParam(Rcw, tcw, Rbc, tbc, bf);
    updateCache();

    return true;
}

bool VertexPose::write(std::ostream& os) const
{
    std::vector<Eigen::Matrix<double, 3, 3>> Rcw = _estimate.Rcw;
    std::vector<Eigen::Matrix<double, 3, 1>> tcw = _estimate.tcw;

    std::vector<Eigen::Matrix<double, 3, 3>> Rbc = _estimate.Rbc;
    std::vector<Eigen::Matrix<double, 3, 1>> tbc = _estimate.tbc;

    const int num_cams = tcw.size();

    for (int idx = 0; idx < num_cams; idx++)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
                os << Rcw[idx](i, j) << " ";
        }
        for (int i = 0; i < 3; i++)
        {
            os << tcw[idx](i) << " ";
        }

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
                os << Rbc[idx](i, j) << " ";
        }
        for (int i = 0; i < 3; i++)
        {
            os << tbc[idx](i) << " ";
        }

        for (size_t i = 0; i < _estimate.pCamera[idx]->size(); i++)
        {
            os << _estimate.pCamera[idx]->getParameter(i) << " ";
        }
    }

    os << _estimate.bf << " ";

    return os.good();
}

void EdgeMono::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw * VPoint->estimate() + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

    const Eigen::Matrix<double, 2, 3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    _jacobianOplusXi = -proj_jac * Rcw;

    Eigen::Matrix<double, 3, 6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);

    SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x, 0.0, 0.0, 0.0, 1.0;

    _jacobianOplusXj = proj_jac * Rcb * SE3deriv;  // TODO optimize this product
}

void EdgeMonoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

    const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw * Xw + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];

    Eigen::Matrix<double, 2, 3> proj_jac = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);

    Eigen::Matrix<double, 3, 6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x, 0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv;  // symbol different becasue of update mode
}

void EdgeStereo::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[1]);
    const g2o::VertexSBAPointXYZ* VPoint = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);

    const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw * VPoint->estimate() + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];
    const double bf = VPose->estimate().bf;
    const double inv_z2 = 1.0 / (Xc(2) * Xc(2));

    Eigen::Matrix<double, 3, 3> proj_jac;
    proj_jac.block<2, 3>(0, 0) = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
    proj_jac(2, 2) += bf * inv_z2;

    _jacobianOplusXi = -proj_jac * Rcw;

    Eigen::Matrix<double, 3, 6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);

    SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x, 0.0, 0.0, 0.0, 1.0;

    _jacobianOplusXj = proj_jac * Rcb * SE3deriv;
}

void EdgeStereoOnlyPose::linearizeOplus()
{
    const VertexPose* VPose = static_cast<const VertexPose*>(_vertices[0]);

    const Eigen::Matrix3d& Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d& tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc = Rcw * Xw + tcw;
    const Eigen::Vector3d Xb = VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d& Rcb = VPose->estimate().Rcb[cam_idx];
    const double bf = VPose->estimate().bf;
    const double inv_z2 = 1.0 / (Xc(2) * Xc(2));

    Eigen::Matrix<double, 3, 3> proj_jac;
    proj_jac.block<2, 3>(0, 0) = VPose->estimate().pCamera[cam_idx]->projectJac(Xc);
    proj_jac.block<1, 3>(2, 0) = proj_jac.block<1, 3>(0, 0);
    proj_jac(2, 2) += bf * inv_z2;

    Eigen::Matrix<double, 3, 6> SE3deriv;
    double x = Xb(0);
    double y = Xb(1);
    double z = Xb(2);
    SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x, 0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi = proj_jac * Rcb * SE3deriv;
}

// SO3 FUNCTIONS
Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w)
{
    return ExpSO3(w[0], w[1], w[2]);
}

Eigen::Matrix3d ExpSO3(const double x, const double y, const double z)
{
    const double d2 = x * x + y * y + z * z;
    const double d = sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5)
    {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
        return NormalizeRotation(res);
    }
    else
    {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W * sin(d) / d + W * W * (1.0 - cos(d)) / d2;
        return NormalizeRotation(res);
    }
}

Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R)
{
    const double tr = R(0, 0) + R(1, 1) + R(2, 2);
    Eigen::Vector3d w;
    w << (R(2, 1) - R(1, 2)) / 2, (R(0, 2) - R(2, 0)) / 2, (R(1, 0) - R(0, 1)) / 2;
    const double costheta = (tr - 1.0) * 0.5f;
    if (costheta > 1 || costheta < -1)
        return w;
    const double theta = acos(costheta);
    const double s = sin(theta);
    if (fabs(s) < 1e-5)
        return w;
    else
        return theta * w / s;
}

Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d& v)
{
    return InverseRightJacobianSO3(v[0], v[1], v[2]);
}

Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x * x + y * y + z * z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5)
        return Eigen::Matrix3d::Identity();
    else
        return Eigen::Matrix3d::Identity() + W / 2 + W * W * (1.0 / d2 - (1.0 + cos(d)) / (2.0 * d * sin(d)));
}

Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d& v)
{
    return RightJacobianSO3(v[0], v[1], v[2]);
}

Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x * x + y * y + z * z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
    if (d < 1e-5)
    {
        return Eigen::Matrix3d::Identity();
    }
    else
    {
        return Eigen::Matrix3d::Identity() - W * (1.0 - cos(d)) / d2 + W * W * (d - sin(d)) / (d2 * d);
    }
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d W;
    W << 0.0, -w[2], w[1], w[2], 0.0, -w[0], -w[1], w[0], 0.0;
    return W;
}

}  // namespace ORB_SLAM3
