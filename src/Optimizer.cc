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

#include "Optimizer.h"

#include "GTSAMTypes.h"
#include "Verbose.h"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <list>
#include <mutex>
#include <string>
#include <tuple>

namespace ORB_SLAM3
{

enum class InertialOptMode
{
    Full,             // optimize pose, vel, bias, gravity, scale
    BiasOnly,         // fix poses; optimize vel, bias
    GravityScaleOnly  // fix pose, vel, bias; optimize gravity, scale
};

namespace
{

void InertialOptimizationImpl(Map* pMap, InertialOptMode mode, Eigen::Matrix3d* pRwg, double* pScale,
                              Eigen::Vector3d* pBg, Eigen::Vector3d* pBa, float priorG, float priorA)
{
    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const bool wantBias = (pBg != nullptr && pBa != nullptr);
    const bool wantGravScale = (pRwg != nullptr && pScale != nullptr);

    if (vpKFs.size() < 2)
    {
        if (wantBias && !vpKFs.empty() && vpKFs.front()->bImu)
        {
            IMU::Bias b = vpKFs.front()->GetImuBias();
            *pBg = Eigen::Vector3d(b.bwx, b.bwy, b.bwz);
            *pBa = Eigen::Vector3d(b.bax, b.bay, b.baz);
        }
        return;
    }

    std::vector<KeyFrame*> vpKF = vpKFs;
    std::sort(vpKF.begin(), vpKF.end(), [](KeyFrame* a, KeyFrame* b) { return a->mnId < b->mnId; });
    const long unsigned int maxKFid = pMap->GetMaxKFid();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    const bool fixPose = (mode == InertialOptMode::BiasOnly || mode == InertialOptMode::GravityScaleOnly);
    const bool fixVel = (mode == InertialOptMode::GravityScaleOnly);
    const bool fixBias = (mode == InertialOptMode::GravityScaleOnly);
    const bool fixGravScale = (mode == InertialOptMode::BiasOnly);

    gtsam::SharedNoiseModel tightPose = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);
    gtsam::SharedNoiseModel tightVel = gtsam::noiseModel::Isotropic::Sigma(3, 1e-6);
    gtsam::SharedNoiseModel tightBias = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);
    gtsam::SharedNoiseModel tightGrav = gtsam::noiseModel::Isotropic::Sigma(3, 1e-6);
    gtsam::SharedNoiseModel tightScale = gtsam::noiseModel::Isotropic::Sigma(1, 1e-6);

    for (KeyFrame* pKFi : vpKF)
    {
        if (!pKFi->bImu)
        {
            continue;
        }
        gtsam::Pose3 Twb = sophusToGTSAMPose(pKFi->GetImuPose());
        gtsam::Key pk = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
        initial.insert(pk, Twb);
        if (fixPose)
        {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(pk, Twb, tightPose));
        }
        Eigen::Vector3d vel = pKFi->GetVelocity().cast<double>();
        gtsam::Key vk = velKey(static_cast<uint32_t>(pKFi->mnId));
        initial.insert(vk, vel);
        if (fixVel)
        {
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(vk, vel, tightVel));
        }
        gtsam::Key bk = biasKey(static_cast<uint32_t>(maxKFid + 3 * pKFi->mnId + 2));
        gtsam::imuBias::ConstantBias bias = toGTSAMBias(pKFi->GetImuBias());
        initial.insert(bk, bias);
        if (fixBias)
        {
            graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bk, bias, tightBias));
        }
    }

    Eigen::Matrix3d RwgInit = Eigen::Matrix3d::Identity();
    double scaleInit = 1.0;
    if (wantGravScale && pRwg && pScale)
    {
        RwgInit = *pRwg;
        scaleInit = *pScale;
    }
    Eigen::Vector3d gDir = RwgInit.transpose() * Eigen::Vector3d(0, 0, -1);
    if (gDir.norm() < 1e-6)
    {
        gDir = Eigen::Vector3d(0, 0, -1);
    }
    gDir.normalize();
    Eigen::Quaterniond qGrav = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, -1), gDir);
    gtsam::Rot3 gravRotInit(qGrav.toRotationMatrix());
    initial.insert(gravKey(), gravRotInit);
    initial.insert(scaleKey(), std::log(std::max(scaleInit, 1e-3)));
    if (fixGravScale)
    {
        graph.add(gtsam::PriorFactor<gtsam::Rot3>(gravKey(), gravRotInit, tightGrav));
        graph.add(gtsam::PriorFactor<double>(scaleKey(), std::log(1.0), tightScale));
    }

    for (size_t i = 0; i < vpKF.size(); i++)
    {
        KeyFrame* pKFi = vpKF[i];
        if (!pKFi->bImu || !pKFi->mPrevKF || !pKFi->mPrevKF->bImu || !pKFi->mpImuPreintegrated)
        {
            continue;
        }
        KeyFrame* pPrev = pKFi->mPrevKF;
        pKFi->mpImuPreintegrated->SetNewBias(pPrev->GetImuBias());
        gtsam::Key p1 = imuPoseKey(static_cast<uint32_t>(pPrev->mnId));
        gtsam::Key v1 = velKey(static_cast<uint32_t>(pPrev->mnId));
        gtsam::Key b1 = biasKey(static_cast<uint32_t>(maxKFid + 3 * pPrev->mnId + 2));
        gtsam::Key p2 = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
        gtsam::Key v2 = velKey(static_cast<uint32_t>(pKFi->mnId));
        graph.add(
            boost::make_shared<InertialGSFactor>(p1, v1, b1, p2, v2, gravKey(), scaleKey(), pKFi->mpImuPreintegrated));
        gtsam::Matrix6 Info6 = gtsam::Matrix6::Zero();
        Info6.block<3, 3>(0, 0) = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        Info6.block<3, 3>(3, 3) = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            b1, biasKey(static_cast<uint32_t>(maxKFid + 3 * pKFi->mnId + 2)), gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Gaussian::Information(Info6)));
    }

    if (!fixBias)
    {
        gtsam::Vector precisions =
            (gtsam::Vector(6) << static_cast<double>(priorA), static_cast<double>(priorA), static_cast<double>(priorA),
             static_cast<double>(priorG), static_cast<double>(priorG), static_cast<double>(priorG))
                .finished();
        graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
            biasKey(static_cast<uint32_t>(maxKFid + 3 * vpKF.front()->mnId + 2)),
            toGTSAMBias(vpKF.front()->GetImuBias()), gtsam::noiseModel::Diagonal::Precisions(precisions)));
    }

    const int maxIter = (mode == InertialOptMode::Full) ? 100 : 50;
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(maxIter);
    if (mode == InertialOptMode::Full)
    {
        params.setlambdaInitial(1e-5);
    }
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, params);
    gtsam::Values result = opt.optimize();

    if (wantGravScale && pRwg && pScale)
    {
        *pRwg = result.at<gtsam::Rot3>(gravKey()).matrix().transpose();
        *pScale = std::exp(result.atDouble(scaleKey()));
    }
    if (wantBias && pBg && pBa)
    {
        gtsam::imuBias::ConstantBias biasEst = result.at<gtsam::imuBias::ConstantBias>(
            biasKey(static_cast<uint32_t>(maxKFid + 3 * vpKF.front()->mnId + 2)));
        *pBg = Eigen::Vector3d(biasEst.gyroscope().x(), biasEst.gyroscope().y(), biasEst.gyroscope().z());
        *pBa = Eigen::Vector3d(biasEst.accelerometer().x(), biasEst.accelerometer().y(), biasEst.accelerometer().z());
    }
}

}  // namespace

bool sortByVal(const std::pair<MapPoint*, int>& a, const std::pair<MapPoint*, int>& b)
{
    return (a.second < b.second);
}

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF,
                                       const bool bRobust)
{
    std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    std::vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(const std::vector<KeyFrame*>& vpKFs, const std::vector<MapPoint*>& vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    std::vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    Map* pMap = vpKFs[0]->GetMap();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::KeyVector landmarkKeys;

    unsigned long maxKFid = 0;
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if (pKF->isBad())
        {
            continue;
        }
        Sophus::SE3f Tcw = pKF->GetPose();
        gtsam::Pose3 Twc = sophusToGTSAMPose(Tcw.inverse());
        initial.insert(poseKey(static_cast<uint32_t>(pKF->mnId)), Twc);
        if (pKF->mnId == pMap->GetInitKFid())
        {
            gtsam::SharedNoiseModel priorNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(poseKey(static_cast<uint32_t>(pKF->mnId)), Twc, priorNoise));
        }
        if (pKF->mnId > maxKFid)
        {
            maxKFid = pKF->mnId;
        }
    }

    const double thHuber2D = std::sqrt(5.99);
    const double thHuber3D = std::sqrt(7.815);

    for (size_t i = 0; i < vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if (pMP->isBad())
        {
            continue;
        }
        gtsam::Point3 Xw(pMP->GetWorldPos().cast<double>());
        const gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + maxKFid + 1));
        initial.insert(pk, Xw);
        landmarkKeys.push_back(pk);

        const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();
        int nEdges = 0;

        for (const auto& mit : observations)
        {
            KeyFrame* pKF = mit.first;
            if (pKF->isBad() || pKF->mnId > maxKFid)
            {
                continue;
            }
            const int leftIndex = std::get<0>(mit.second);
            if (leftIndex == -1)
            {
                continue;
            }
            const gtsam::Key poseK = poseKey(static_cast<uint32_t>(pKF->mnId));
            const double invSigma2 = static_cast<double>(pKF->mvInvLevelSigma2[pKF->mvKeysUn[leftIndex].octave]);

            if (pKF->mvuRight[leftIndex] < 0)
            {
                Eigen::Vector2d obs(pKF->mvKeysUn[leftIndex].pt.x, pKF->mvKeysUn[leftIndex].pt.y);
                gtsam::SharedNoiseModel noise =
                    bRobust ? makeHuberNoise(2, 5.99, invSigma2) : makeIsotropicNoise(2, invSigma2);
                boost::shared_ptr<gtsam::Cal3_S2> cal = boost::make_shared<gtsam::Cal3_S2>(toGTSAMCal(pKF->mpCamera));
                graph.add(
                    boost::make_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                        obs, noise, poseK, pk, cal));
                nEdges++;
            }
            else
            {
                const float kp_ur = pKF->mvuRight[leftIndex];
                gtsam::StereoPoint2 obs(pKF->mvKeysUn[leftIndex].pt.x, pKF->mvKeysUn[leftIndex].pt.y, kp_ur);
                gtsam::SharedNoiseModel noise =
                    bRobust ? makeHuberNoise(3, 7.815, invSigma2) : makeIsotropicNoise(3, invSigma2);
                boost::shared_ptr<gtsam::Cal3_S2Stereo> cal =
                    boost::make_shared<gtsam::Cal3_S2Stereo>(toGTSAMStereoCal(pKF->mpCamera, pKF->mbf));
                graph.add(boost::make_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(obs, noise, poseK,
                                                                                                      pk, cal));
                nEdges++;
            }
        }

        vbNotIncludedMP[i] = (nEdges == 0);
        if (nEdges == 0)
        {
            initial.erase(pk);
            landmarkKeys.pop_back();
        }
    }

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(nIterations);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)

    gtsam::Ordering ordering = gtsam::Ordering::ColamdConstrainedFirst(graph, landmarkKeys);
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, ordering, params);

    gtsam::Values result;
    if (pbStopFlag)
    {
        for (int it = 0; it < nIterations; it++)
        {
            if (*pbStopFlag)
            {
                break;
            }
            opt.iterate();
        }
        result = opt.values();
    }
    else
    {
        result = opt.optimize();
    }

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if (pKF->isBad())
        {
            continue;
        }
        gtsam::Pose3 Twc = result.at<gtsam::Pose3>(poseKey(static_cast<uint32_t>(pKF->mnId)));
        Sophus::SE3f Tcw = gtsamToSophusPose(Twc).inverse();
        if (nLoopKF == pMap->GetOriginKF()->mnId)
        {
            pKF->SetPose(Tcw);
        }
        else
        {
            pKF->mTcwGBA = Tcw.cast<double>().cast<float>();
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    for (size_t i = 0; i < vpMP.size(); i++)
    {
        if (vbNotIncludedMP[i])
        {
            continue;
        }
        MapPoint* pMP = vpMP[i];
        if (pMP->isBad())
        {
            continue;
        }
        gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + maxKFid + 1));
        if (!result.exists(pk))
        {
            continue;
        }
        gtsam::Point3 Xw = result.at<gtsam::Point3>(pk);
        if (nLoopKF == pMap->GetOriginKF()->mnId)
        {
            pMP->SetWorldPos(Xw.cast<float>());
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA = Xw.cast<float>();
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }
}

void Optimizer::FullInertialBA(Map* pMap, int its, const bool bFixLocal, const long unsigned int nLoopId,
                               bool* pbStopFlag, bool bInit, float priorG, float priorA, Eigen::VectorXd* vSingVal,
                               bool* bHess)
{
    (void)vSingVal;
    (void)bHess;
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::KeyVector landmarkKeys;

    const gtsam::Key commonBiasKey = biasKey(static_cast<uint32_t>(4 * maxKFid + 2));
    int nNonFixed = 0;

    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
        {
            continue;
        }
        bool bFixed = false;
        if (bFixLocal)
        {
            bFixed = (pKFi->mnBALocalForKF >= (maxKFid - 1)) || (pKFi->mnBAFixedForKF >= (maxKFid - 1));
            if (!bFixed)
            {
                nNonFixed++;
            }
        }

        gtsam::Pose3 Twb = sophusToGTSAMPose(pKFi->GetImuPose());
        initial.insert(imuPoseKey(static_cast<uint32_t>(pKFi->mnId)), Twb);
        if (bFixed)
        {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(imuPoseKey(static_cast<uint32_t>(pKFi->mnId)), Twb,
                                                       gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)));
        }

        if (pKFi->bImu)
        {
            initial.insert(velKey(static_cast<uint32_t>(pKFi->mnId)),
                           Eigen::Vector3d(pKFi->GetVelocity().cast<double>()));
            if (bInit)
            {
                initial.insert(commonBiasKey, toGTSAMBias(pKFi->GetImuBias()));
            }
            else
            {
                initial.insert(biasKey(static_cast<uint32_t>(maxKFid + 3 * (pKFi->mnId) + 2)),
                               toGTSAMBias(pKFi->GetImuBias()));
            }
        }
    }

    if (bFixLocal && nNonFixed < 3)
    {
        return;
    }
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if (!pKFi->mPrevKF || pKFi->mnId > maxKFid || pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
        {
            continue;
        }
        if (!pKFi->bImu || !pKFi->mPrevKF->bImu)
        {
            continue;
        }
        pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());

        gtsam::Key p1 = imuPoseKey(static_cast<uint32_t>(pKFi->mPrevKF->mnId));
        gtsam::Key v1 = velKey(static_cast<uint32_t>(pKFi->mPrevKF->mnId));
        gtsam::Key b1 = bInit ? commonBiasKey : biasKey(static_cast<uint32_t>(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2));
        gtsam::Key p2 = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
        gtsam::Key v2 = velKey(static_cast<uint32_t>(pKFi->mnId));

        graph.add(boost::make_shared<InertialFactor>(p1, v1, b1, p2, v2, pKFi->mpImuPreintegrated));

        if (!bInit)
        {
            gtsam::Matrix6 Info6 = gtsam::Matrix6::Zero();
            Info6.block<3, 3>(0, 0) = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
            Info6.block<3, 3>(3, 3) = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
            auto rwNoise = gtsam::noiseModel::Gaussian::Information(Info6);
            graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
                b1, biasKey(static_cast<uint32_t>(maxKFid + 3 * (pKFi->mnId) + 2)), gtsam::imuBias::ConstantBias(),
                rwNoise));
        }
    }

    if (bInit)
    {
        gtsam::Vector precisions = (gtsam::Vector(6) << priorA, priorA, priorA, priorG, priorG, priorG).finished();
        gtsam::SharedNoiseModel priorBiasNoise = gtsam::noiseModel::Diagonal::Precisions(precisions);
        graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(commonBiasKey, gtsam::imuBias::ConstantBias(),
                                                                   priorBiasNoise));
    }

    const unsigned long iniMPid = maxKFid * 5;
    std::vector<bool> vbNotIncludedMP(vpMPs.size(), false);

    for (size_t i = 0; i < vpMPs.size(); i++)
    {
        MapPoint* pMP = vpMPs[i];
        gtsam::Point3 Xw(pMP->GetWorldPos().cast<double>());
        const gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + iniMPid + 1));
        initial.insert(pk, Xw);
        landmarkKeys.push_back(pk);

        const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();
        bool bAllFixed = true;
        int nEdges = 0;

        for (const auto& mit : observations)
        {
            KeyFrame* pKFi = mit.first;
            if (pKFi->mnId > maxKFid || pKFi->isBad())
            {
                continue;
            }
            const int leftIndex = std::get<0>(mit.second);
            if (leftIndex == -1)
            {
                continue;
            }
            const gtsam::Key poseK = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
            if (!initial.exists(poseK))
            {
                continue;
            }
            bAllFixed = false;
            const double invSigma2 = static_cast<double>(pKFi->mvInvLevelSigma2[pKFi->mvKeysUn[leftIndex].octave]);
            gtsam::Pose3 Tbc_kf = sophusToGTSAMPose(pKFi->mImuCalib.mTbc);

            if (pKFi->mvuRight[leftIndex] < 0)
            {
                Eigen::Vector2d obs(pKFi->mvKeysUn[leftIndex].pt.x, pKFi->mvKeysUn[leftIndex].pt.y);
                gtsam::SharedNoiseModel noise = makeHuberNoise(2, 5.991, invSigma2);
                auto cal = boost::make_shared<gtsam::Cal3_S2>(toGTSAMCal(pKFi->mpCamera));
                graph.add(
                    boost::make_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                        obs, noise, poseK, pk, cal, Tbc_kf));
                nEdges++;
            }
            else
            {
                const float kp_ur = pKFi->mvuRight[leftIndex];
                gtsam::StereoPoint2 obs(pKFi->mvKeysUn[leftIndex].pt.x, pKFi->mvKeysUn[leftIndex].pt.y, kp_ur);
                gtsam::SharedNoiseModel noise = makeHuberNoise(3, 7.815, invSigma2);
                auto cal = boost::make_shared<gtsam::Cal3_S2Stereo>(toGTSAMStereoCal(pKFi->mpCamera, pKFi->mbf));
                graph.add(boost::make_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(obs, noise, poseK,
                                                                                                      pk, cal, Tbc_kf));
                nEdges++;
            }
        }

        vbNotIncludedMP[i] = (nEdges == 0);
        if (nEdges == 0)
        {
            initial.erase(pk);
            landmarkKeys.pop_back();
        }
    }

    if (pbStopFlag && *pbStopFlag)
    {
        return;
    }
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(its);
    params.setlambdaInitial(1e-5);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    gtsam::Ordering ordering = landmarkKeys.empty() ? gtsam::Ordering::Colamd(graph)
                                                    : gtsam::Ordering::ColamdConstrainedFirst(graph, landmarkKeys);
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, ordering, params);

    gtsam::Values result;
    if (pbStopFlag)
    {
        for (int it = 0; it < its; it++)
        {
            if (*pbStopFlag)
            {
                break;
            }
            opt.iterate();
        }
        result = opt.values();
    }
    else
    {
        result = opt.optimize();
    }
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if (pKFi->mnId > maxKFid)
        {
            continue;
        }
        gtsam::Pose3 Twb = result.at<gtsam::Pose3>(imuPoseKey(static_cast<uint32_t>(pKFi->mnId)));
        Sophus::SE3f Twb_s = gtsamToSophusPose(Twb);
        Sophus::SE3f Tcw = (Twb_s * pKFi->mImuCalib.mTbc).inverse();
        if (nLoopId == 0)
        {
            pKFi->SetPose(Tcw);
        }
        else
        {
            pKFi->mTcwGBA = Tcw;
            pKFi->mnBAGlobalForKF = nLoopId;
        }
        if (pKFi->bImu)
        {
            Eigen::Vector3d v = result.at<gtsam::Vector3>(velKey(static_cast<uint32_t>(pKFi->mnId)));
            if (nLoopId == 0)
            {
                pKFi->SetVelocity(v.cast<float>());
            }
            else
            {
                pKFi->mVwbGBA = v.cast<float>();
            }
            gtsam::imuBias::ConstantBias bias = bInit ? result.at<gtsam::imuBias::ConstantBias>(commonBiasKey)
                                                      : result.at<gtsam::imuBias::ConstantBias>(biasKey(
                                                            static_cast<uint32_t>(maxKFid + 3 * (pKFi->mnId) + 2)));
            IMU::Bias b = fromGTSAMBias(bias);
            if (nLoopId == 0)
            {
                pKFi->SetNewBias(b);
            }
            else
            {
                pKFi->mBiasGBA = b;
            }
        }
    }

    for (size_t i = 0; i < vpMPs.size(); i++)
    {
        if (vbNotIncludedMP[i])
        {
            continue;
        }
        MapPoint* pMP = vpMPs[i];
        gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + iniMPid + 1));
        if (!result.exists(pk))
        {
            continue;
        }
        gtsam::Point3 Xw = result.at<gtsam::Point3>(pk);
        if (nLoopId == 0)
        {
            pMP->SetWorldPos(Xw.cast<float>());
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA = Xw.cast<float>();
            pMP->mnBAGlobalForKF = nLoopId;
        }
    }
    pMap->IncreaseChangeIndex();
}

int Optimizer::PoseOptimization(Frame* pFrame)
{
    const gtsam::Key poseK = poseKey(0);
    const int N = pFrame->N;

    std::vector<boost::shared_ptr<PinholeMonoPoseTcwFactor>> vpFactorsMono;
    std::vector<boost::shared_ptr<PinholeStereoPoseTcwFactor>> vpFactorsStereo;
    std::vector<size_t> vnIndexMono, vnIndexStereo;
    int nInitialCorrespondences = 0;

    gtsam::Pose3 Tcw = sophusToGTSAMPose(pFrame->GetPose());
    gtsam::Values initial;
    initial.insert(poseK, Tcw);

    {
        std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);
        for (int i = 0; i < N; i++)
        {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if (!pMP)
            {
                continue;
            }
            Eigen::Vector3d Xw = pMP->GetWorldPos().cast<double>();
            const double invSigma2 = static_cast<double>(pFrame->mvInvLevelSigma2[pFrame->mvKeysUn[i].octave]);

            if (pFrame->mvuRight[i] < 0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;
                Eigen::Vector2d obs(pFrame->mvKeysUn[i].pt.x, pFrame->mvKeysUn[i].pt.y);
                auto noise = makeHuberNoise(2, 5.991, invSigma2);
                vpFactorsMono.push_back(
                    boost::make_shared<PinholeMonoPoseTcwFactor>(poseK, Xw, obs, noise, pFrame->mpCamera));
                vnIndexMono.push_back(static_cast<size_t>(i));
            }
            else
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;
                Eigen::Vector3d obs(pFrame->mvKeysUn[i].pt.x, pFrame->mvKeysUn[i].pt.y, pFrame->mvuRight[i]);
                auto noise = makeHuberNoise(3, 7.815, invSigma2);
                vpFactorsStereo.push_back(boost::make_shared<PinholeStereoPoseTcwFactor>(poseK, Xw, obs, pFrame->mbf,
                                                                                         noise, pFrame->mpCamera));
                vnIndexStereo.push_back(static_cast<size_t>(i));
            }
        }
    }

    if (nInitialCorrespondences < 3)
    {
        return 0;
    }
    const int its[4] = {10, 10, 10, 10};
    const double chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const double chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    int nBad = 0;

    for (int it = 0; it < 4; it++)
    {
        gtsam::NonlinearFactorGraph graph;
        for (size_t i = 0; i < vpFactorsMono.size(); i++)
        {
            if (!pFrame->mvbOutlier[vnIndexMono[i]])
            {
                graph.add(vpFactorsMono[i]);
            }
        }
        for (size_t i = 0; i < vpFactorsStereo.size(); i++)
        {
            if (!pFrame->mvbOutlier[vnIndexStereo[i]])
            {
                graph.add(vpFactorsStereo[i]);
            }
        }
        gtsam::LevenbergMarquardtParams params;
        params.setMaxIterations(its[it]);
        params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
        gtsam::LevenbergMarquardtOptimizer opt(graph, initial, params);
        gtsam::Values result = opt.optimize();
        initial = result;

        nBad = 0;
        for (size_t i = 0; i < vpFactorsMono.size(); i++)
        {
            double err = 2.0 * vpFactorsMono[i]->error(result);
            if (err > chi2Mono[it])
            {
                pFrame->mvbOutlier[vnIndexMono[i]] = true;
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[vnIndexMono[i]] = false;
            }
        }
        for (size_t i = 0; i < vpFactorsStereo.size(); i++)
        {
            double err = 2.0 * vpFactorsStereo[i]->error(result);
            if (err > chi2Stereo[it])
            {
                pFrame->mvbOutlier[vnIndexStereo[i]] = true;
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[vnIndexStereo[i]] = false;
            }
        }

        if (graph.size() < 10)
        {
            break;
        }
    }

    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << pFrame->mnId << "] POSE_OPTIMIZATION: nInitialCorrespondences=" << nInitialCorrespondences
        << std::endl;
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << pFrame->mnId << "] POSE_OPTIMIZATION: nMatches=" << nInitialCorrespondences - nBad << std::endl;
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << "[" << pFrame->mnId << "] POSE_OPTIMIZATION: nOutliers=" << nBad << std::endl;

    gtsam::Pose3 Tcw_final = initial.at<gtsam::Pose3>(poseK);
    pFrame->SetPose(gtsamToSophusPose(Tcw_final));
    return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF,
                                      int& num_MPs, int& num_edges)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    std::list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    Map* pCurrentMap = pKF->GetMap();

    const std::vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
        {
            lLocalKeyFrames.push_back(pKFi);
        }
    }

    // Local MapPoints seen in Local KeyFrames
    num_fixedKF = 0;
    std::list<MapPoint*> lLocalMapPoints;
    std::set<MapPoint*> sNumObsMP;
    for (std::list<KeyFrame*>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if (pKFi->mnId == pMap->GetInitKFid())
        {
            num_fixedKF = 1;
        }
        std::vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            MapPoint* pMP = *vit;
            if (pMP)
            {
                if (!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                {

                    if (pMP->mnBALocalForKF != pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
                }
            }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    std::list<KeyFrame*> lFixedCameras;
    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        std::map<KeyFrame*, std::tuple<int, int>> observations = (*lit)->GetObservations();
        for (std::map<KeyFrame*, std::tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
            {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                {
                    lFixedCameras.push_back(pKFi);
                }
            }
        }
    }
    num_fixedKF = lFixedCameras.size() + num_fixedKF;

    if (num_fixedKF == 0)
    {
        return;
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::KeyVector landmarkKeys;
    unsigned long maxKFid = 0;

    pCurrentMap->msOptKFs.clear();
    pCurrentMap->msFixedKFs.clear();

    for (KeyFrame* pKFi : lLocalKeyFrames)
    {
        gtsam::Pose3 Tcw = sophusToGTSAMPose(pKFi->GetPose());
        initial.insert(poseKey(static_cast<uint32_t>(pKFi->mnId)), Tcw);
        if (pKFi->mnId == pMap->GetInitKFid())
        {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(poseKey(static_cast<uint32_t>(pKFi->mnId)), Tcw,
                                                       gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)));
        }
        if (pKFi->mnId > maxKFid)
        {
            maxKFid = pKFi->mnId;
        }
        pCurrentMap->msOptKFs.insert(pKFi->mnId);
    }
    num_OptKF = static_cast<int>(lLocalKeyFrames.size());

    for (KeyFrame* pKFi : lFixedCameras)
    {
        gtsam::Pose3 Tcw = sophusToGTSAMPose(pKFi->GetPose());
        initial.insert(poseKey(static_cast<uint32_t>(pKFi->mnId)), Tcw);
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(poseKey(static_cast<uint32_t>(pKFi->mnId)), Tcw,
                                                   gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)));
        if (pKFi->mnId > maxKFid)
        {
            maxKFid = pKFi->mnId;
        }
        pCurrentMap->msFixedKFs.insert(pKFi->mnId);
    }

    int nPoints = 0;
    int nEdges = 0;
    std::vector<std::tuple<KeyFrame*, MapPoint*, int>> monoEdges;
    std::vector<std::tuple<KeyFrame*, MapPoint*, int>> stereoEdges;

    for (MapPoint* pMP : lLocalMapPoints)
    {
        gtsam::Point3 Xw(pMP->GetWorldPos().cast<double>());
        const gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + maxKFid + 1));
        initial.insert(pk, Xw);
        landmarkKeys.push_back(pk);
        nPoints++;

        for (const auto& mit : pMP->GetObservations())
        {
            KeyFrame* pKFi = mit.first;
            if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
            {
                continue;
            }
            const int leftIndex = std::get<0>(mit.second);
            if (leftIndex == -1)
            {
                continue;
            }
            const gtsam::Key poseK = poseKey(static_cast<uint32_t>(pKFi->mnId));
            const double invSigma2 = static_cast<double>(pKFi->mvInvLevelSigma2[pKFi->mvKeysUn[leftIndex].octave]);

            if (pKFi->mvuRight[leftIndex] < 0)
            {
                Eigen::Vector2d obs(pKFi->mvKeysUn[leftIndex].pt.x, pKFi->mvKeysUn[leftIndex].pt.y);
                gtsam::SharedNoiseModel noise = makeHuberNoise(2, 5.991, invSigma2);
                assert(pKFi->mpCamera->GetType() == GeometricCamera::CAM_PINHOLE);
                graph.add(boost::make_shared<PinholeMonoTcwFactor>(poseK, pk, obs, noise, pKFi->mpCamera));

                nEdges++;
                monoEdges.push_back(std::make_tuple(pKFi, pMP, leftIndex));
            }
            else
            {
                const float kp_ur = pKFi->mvuRight[leftIndex];
                gtsam::StereoPoint2 obs(pKFi->mvKeysUn[leftIndex].pt.x, pKFi->mvKeysUn[leftIndex].pt.y, kp_ur);
                gtsam::SharedNoiseModel noise = makeHuberNoise(3, 7.815, invSigma2);
                assert(pKFi->mpCamera->GetType() == GeometricCamera::CAM_PINHOLE);
                graph.add(boost::make_shared<PinholeStereoTcwFactor>(
                    poseK, pk, Eigen::Vector3d(obs.uL(), obs.v(), obs.uR()), pKFi->mbf, noise, pKFi->mpCamera));

                nEdges++;
                stereoEdges.push_back(std::make_tuple(pKFi, pMP, leftIndex));
            }
        }
    }
    num_edges = nEdges;
    num_MPs = nPoints;

    if (pbStopFlag && *pbStopFlag)
    {
        return;
    }
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(10);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    if (pMap->IsInertial())
    {
        params.setlambdaInitial(100.0);
    }
    gtsam::Ordering ordering = gtsam::Ordering::ColamdConstrainedFirst(graph, landmarkKeys);
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, ordering, params);
    gtsam::Values result;
    if (pbStopFlag)
    {
        for (int it = 0; it < 10; it++)
        {
            if (*pbStopFlag)
            {
                break;
            }
            opt.iterate();
        }
        result = opt.values();
    }
    else
    {
        result = opt.optimize();
    }
    (void)monoEdges;
    (void)stereoEdges;

    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

    for (KeyFrame* pKFi : lLocalKeyFrames)
    {
        gtsam::Key k = poseKey(static_cast<uint32_t>(pKFi->mnId));
        if (result.exists(k))
        {
            gtsam::Pose3 Tcw = result.at<gtsam::Pose3>(k);
            pKFi->SetPose(gtsamToSophusPose(Tcw));
        }
    }

    for (MapPoint* pMP : lLocalMapPoints)
    {
        gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + maxKFid + 1));
        if (result.exists(pk))
        {
            pMP->SetWorldPos(result.at<gtsam::Point3>(pk).cast<float>());
            pMP->UpdateNormalAndDepth();
        }
    }
    pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose& CorrectedSim3,
                                       const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections,
                                       const bool& bFixScale)
{
    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();
    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    std::vector<gtsam::Similarity3> vScw(nMaxKFid + 1);
    std::vector<gtsam::Similarity3> vCorrectedSwc(nMaxKFid + 1);

    const int minFeat = 100;
    gtsam::SharedNoiseModel sim3Noise = gtsam::noiseModel::Isotropic::Sigma(7, 1.0);

    for (KeyFrame* pKF : vpKFs)
    {
        if (pKF->isBad())
        {
            continue;
        }
        const int nIDi = pKF->mnId;
        gtsam::Similarity3 Siw;
        auto it = CorrectedSim3.find(pKF);
        if (it != CorrectedSim3.end())
        {
            Siw = it->second;
            vScw[nIDi] = Siw;
        }
        else
        {
            Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
            Siw = gtsam::Similarity3(gtsam::Rot3(Tcw.rotationMatrix()), gtsam::Point3(Tcw.translation()), 1.0);
            vScw[nIDi] = Siw;
        }
        initial.insert(sim3Key(nIDi), Siw);
        if (pKF->mnId == pMap->GetInitKFid())
        {
            graph.add(gtsam::PriorFactor<gtsam::Similarity3>(sim3Key(nIDi), Siw,
                                                             gtsam::noiseModel::Isotropic::Sigma(7, 1e-6)));
        }
    }

    std::set<std::pair<long unsigned int, long unsigned int>> sInsertedEdges;

    for (const auto& mit : LoopConnections)
    {
        KeyFrame* pKF = mit.first;
        const long unsigned int nIDi = pKF->mnId;
        const gtsam::Similarity3 Swi = vScw[nIDi].inverse();
        for (KeyFrame* pKFn : mit.second)
        {
            const long unsigned int nIDj = pKFn->mnId;
            if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(pKFn) < minFeat)
            {
                continue;
            }
            gtsam::Similarity3 Sji = vScw[nIDj].compose(Swi);
            graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::Similarity3>>(sim3Key(nIDi), sim3Key(nIDj), Sji,
                                                                                   sim3Noise));
            sInsertedEdges.insert(std::make_pair(std::min(nIDi, nIDj), std::max(nIDi, nIDj)));
        }
    }

    for (KeyFrame* pKF : vpKFs)
    {
        if (pKF->isBad())
        {
            continue;
        }
        const int nIDi = pKF->mnId;
        gtsam::Similarity3 Swi;
        auto iti = NonCorrectedSim3.find(pKF);
        if (iti != NonCorrectedSim3.end())
        {
            Swi = iti->second.inverse();
        }
        else
        {
            Swi = vScw[nIDi].inverse();
        }
        KeyFrame* pParentKF = pKF->GetParent();
        if (pParentKF)
        {
            int nIDj = pParentKF->mnId;
            gtsam::Similarity3 Sjw;
            auto itj = NonCorrectedSim3.find(pParentKF);
            if (itj != NonCorrectedSim3.end())
            {
                Sjw = itj->second;
            }
            else
            {
                Sjw = vScw[nIDj];
            }
            graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::Similarity3>>(sim3Key(nIDi), sim3Key(nIDj),
                                                                                   Sjw.compose(Swi), sim3Noise));
        }

        for (KeyFrame* pLKF : pKF->GetLoopEdges())
        {
            if (pLKF->mnId >= pKF->mnId)
            {
                continue;
            }
            gtsam::Similarity3 Slw;
            auto itl = NonCorrectedSim3.find(pLKF);
            if (itl != NonCorrectedSim3.end())
            {
                Slw = itl->second;
            }
            else
            {
                Slw = vScw[pLKF->mnId];
            }
            graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::Similarity3>>(sim3Key(nIDi), sim3Key(pLKF->mnId),
                                                                                   Slw.compose(Swi), sim3Noise));
        }

        for (KeyFrame* pKFn : pKF->GetCovisiblesByWeight(minFeat))
        {
            if (!pKFn || pKFn == pParentKF || pKF->hasChild(pKFn) || pKFn->isBad() || pKFn->mnId >= pKF->mnId)
            {
                continue;
            }
            if (sInsertedEdges.count(std::make_pair(std::min(pKF->mnId, pKFn->mnId), std::max(pKF->mnId, pKFn->mnId))))
            {
                continue;
            }
            gtsam::Similarity3 Snw;
            auto itn = NonCorrectedSim3.find(pKFn);
            if (itn != NonCorrectedSim3.end())
            {
                Snw = itn->second;
            }
            else
            {
                Snw = vScw[pKFn->mnId];
            }
            graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::Similarity3>>(sim3Key(nIDi), sim3Key(pKFn->mnId),
                                                                                   Snw.compose(Swi), sim3Noise));
        }

        if (pKF->bImu && pKF->mPrevKF)
        {
            gtsam::Similarity3 Spw;
            auto itp = NonCorrectedSim3.find(pKF->mPrevKF);
            if (itp != NonCorrectedSim3.end())
            {
                Spw = itp->second;
            }
            else
            {
                Spw = vScw[pKF->mPrevKF->mnId];
            }
            graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::Similarity3>>(
                sim3Key(nIDi), sim3Key(pKF->mPrevKF->mnId), Spw.compose(Swi), sim3Noise));
        }
    }

    gtsam::LevenbergMarquardtParams params;
    params.setlambdaInitial(1e-16);
    params.setMaxIterations(20);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, params);
    gtsam::Values result = opt.optimize();

    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    for (KeyFrame* pKFi : vpKFs)
    {
        if (pKFi->isBad())
        {
            continue;
        }
        const int nIDi = pKFi->mnId;
        gtsam::Similarity3 CorrectedSiw = result.at<gtsam::Similarity3>(sim3Key(nIDi));
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
        double s = CorrectedSiw.scale();
        Eigen::Vector3d t = Eigen::Vector3d(CorrectedSiw.translation().x(), CorrectedSiw.translation().y(),
                                            CorrectedSiw.translation().z()) /
                            s;
        Sophus::SE3f Tiw(CorrectedSiw.rotation().matrix().cast<float>(), t.cast<float>());
        pKFi->SetPose(Tiw);
    }

    for (MapPoint* pMP : vpMPs)
    {
        if (pMP->isBad())
        {
            continue;
        }
        int nIDr =
            (pMP->mnCorrectedByKF == pCurKF->mnId) ? pMP->mnCorrectedReference : pMP->GetReferenceKeyFrame()->mnId;
        const gtsam::Similarity3& Srw = vScw[nIDr];
        const gtsam::Similarity3& correctedSwr = vCorrectedSwc[nIDr];
        gtsam::Point3 eigP3Dw(pMP->GetWorldPos().cast<double>());
        gtsam::Point3 inCam = Srw.transformFrom(eigP3Dw);
        gtsam::Point3 eigCorrectedP3Dw = correctedSwr.transformFrom(inCam);
        pMP->SetWorldPos(Eigen::Vector3f(static_cast<float>(eigCorrectedP3Dw.x()),
                                         static_cast<float>(eigCorrectedP3Dw.y()),
                                         static_cast<float>(eigCorrectedP3Dw.z())));
        pMP->UpdateNormalAndDepth();
    }
    pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(KeyFrame* pCurKF, std::vector<KeyFrame*>& vpFixedKFs,
                                       std::vector<KeyFrame*>& vpFixedCorrectedKFs,
                                       std::vector<KeyFrame*>& vpNonFixedKFs, std::vector<MapPoint*>& vpNonCorrectedMPs)
{
    Map* pMap = pCurKF->GetMap();
    const unsigned int nMaxKFid = pMap->GetMaxKFid();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    std::vector<gtsam::Similarity3> vScw(nMaxKFid + 1);
    std::vector<gtsam::Similarity3> vCorrectedSwc(nMaxKFid + 1);
    std::vector<bool> vpGoodPose(nMaxKFid + 1);
    std::vector<bool> vpBadPose(nMaxKFid + 1);
    const int minFeat = 100;
    gtsam::SharedNoiseModel sim3Noise = gtsam::noiseModel::Isotropic::Sigma(7, 1.0);

    for (KeyFrame* pKFi : vpFixedKFs)
    {
        if (pKFi->isBad())
        {
            continue;
        }
        const int nIDi = pKFi->mnId;
        Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
        gtsam::Similarity3 Siw(gtsam::Rot3(Tcw.rotationMatrix()), gtsam::Point3(Tcw.translation()), 1.0);
        vCorrectedSwc[nIDi] = Siw.inverse();
        initial.insert(sim3Key(nIDi), Siw);
        graph.add(
            gtsam::PriorFactor<gtsam::Similarity3>(sim3Key(nIDi), Siw, gtsam::noiseModel::Isotropic::Sigma(7, 1e-6)));
        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi] = false;
    }

    std::set<unsigned long> sIdKF;
    for (KeyFrame* pKFi : vpFixedCorrectedKFs)
    {
        if (pKFi->isBad())
        {
            continue;
        }
        const int nIDi = pKFi->mnId;
        Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
        gtsam::Similarity3 Siw(gtsam::Rot3(Tcw.rotationMatrix()), gtsam::Point3(Tcw.translation()), 1.0);
        vCorrectedSwc[nIDi] = Siw.inverse();
        Sophus::SE3d Tcw_bef = pKFi->mTcwBefMerge.cast<double>();
        vScw[nIDi] =
            gtsam::Similarity3(gtsam::Rot3(Tcw_bef.rotationMatrix()), gtsam::Point3(Tcw_bef.translation()), 1.0);
        initial.insert(sim3Key(nIDi), Siw);
        graph.add(
            gtsam::PriorFactor<gtsam::Similarity3>(sim3Key(nIDi), Siw, gtsam::noiseModel::Isotropic::Sigma(7, 1e-6)));
        sIdKF.insert(nIDi);
        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi] = true;
    }

    for (KeyFrame* pKFi : vpNonFixedKFs)
    {
        if (pKFi->isBad())
        {
            continue;
        }
        const int nIDi = pKFi->mnId;
        if (sIdKF.count(nIDi))
        {
            continue;
        }
        Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
        gtsam::Similarity3 Siw(gtsam::Rot3(Tcw.rotationMatrix()), gtsam::Point3(Tcw.translation()), 1.0);
        vScw[nIDi] = Siw;
        initial.insert(sim3Key(nIDi), Siw);
        sIdKF.insert(nIDi);
        vpGoodPose[nIDi] = false;
        vpBadPose[nIDi] = true;
    }

    std::vector<KeyFrame*> vpKFs;
    vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() + vpNonFixedKFs.size());
    vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
    vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(), vpFixedCorrectedKFs.end());
    vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
    std::set<KeyFrame*> spKFs(vpKFs.begin(), vpKFs.end());

    for (KeyFrame* pKFi : vpKFs)
    {
        const int nIDi = pKFi->mnId;
        gtsam::Similarity3 Swi = vpBadPose[nIDi] ? vScw[nIDi].inverse() : vCorrectedSwc[nIDi];
        KeyFrame* pParentKFi = pKFi->GetParent();

        if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end())
        {
            int nIDj = pParentKFi->mnId;
            gtsam::Similarity3 Sjw =
                (vpGoodPose[nIDi] && vpGoodPose[nIDj]) ? vCorrectedSwc[nIDj].inverse() : vScw[nIDj];
            graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::Similarity3>>(sim3Key(nIDi), sim3Key(nIDj),
                                                                                   Sjw.compose(Swi), sim3Noise));
        }

        for (KeyFrame* pLKF : pKFi->GetLoopEdges())
        {
            if (spKFs.find(pLKF) == spKFs.end() || pLKF->mnId >= pKFi->mnId)
            {
                continue;
            }
            gtsam::Similarity3 Slw =
                (vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId]) ? vCorrectedSwc[pLKF->mnId].inverse() : vScw[pLKF->mnId];
            graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::Similarity3>>(sim3Key(nIDi), sim3Key(pLKF->mnId),
                                                                                   Slw.compose(Swi), sim3Noise));
        }

        const std::set<KeyFrame*> sLoopEdges = pKFi->GetLoopEdges();
        for (KeyFrame* pKFn : pKFi->GetCovisiblesByWeight(minFeat))
        {
            if (!pKFn || pKFn == pParentKFi || pKFi->hasChild(pKFn) || sLoopEdges.count(pKFn) ||
                spKFs.find(pKFn) == spKFs.end())
            {
                continue;
            }
            if (pKFn->isBad() || pKFn->mnId >= pKFi->mnId)
            {
                continue;
            }
            gtsam::Similarity3 Snw =
                (vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId]) ? vCorrectedSwc[pKFn->mnId].inverse() : vScw[pKFn->mnId];
            graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::Similarity3>>(sim3Key(nIDi), sim3Key(pKFn->mnId),
                                                                                   Snw.compose(Swi), sim3Noise));
        }
    }

    gtsam::LevenbergMarquardtParams params;
    params.setlambdaInitial(1e-16);
    params.setMaxIterations(20);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, params);
    gtsam::Values result = opt.optimize();

    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    for (KeyFrame* pKFi : vpNonFixedKFs)
    {
        if (pKFi->isBad())
        {
            continue;
        }
        const int nIDi = pKFi->mnId;
        gtsam::Similarity3 CorrectedSiw = result.at<gtsam::Similarity3>(sim3Key(nIDi));
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
        double s = CorrectedSiw.scale();
        Eigen::Vector3d t = Eigen::Vector3d(CorrectedSiw.translation().x(), CorrectedSiw.translation().y(),
                                            CorrectedSiw.translation().z()) /
                            s;
        Sophus::SE3d Tiw(CorrectedSiw.rotation().matrix(), t);
        pKFi->mTcwBefMerge = pKFi->GetPose();
        pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
        pKFi->SetPose(Tiw.cast<float>());
    }

    for (MapPoint* pMPi : vpNonCorrectedMPs)
    {
        if (pMPi->isBad())
        {
            continue;
        }
        KeyFrame* pRefKF = pMPi->GetReferenceKeyFrame();
        while (pRefKF && pRefKF->isBad())
        {
            pMPi->EraseObservation(pRefKF);
            pRefKF = pMPi->GetReferenceKeyFrame();
        }
        if (!pRefKF || !vpBadPose[pRefKF->mnId])
        {
            continue;
        }
        Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
        Sophus::SE3f Twr = pRefKF->GetPoseInverse();
        Eigen::Vector3f eigCorrectedP3Dw = Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
        pMPi->SetWorldPos(eigCorrectedP3Dw);
        pMPi->UpdateNormalAndDepth();
    }
}

int Optimizer::OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches1,
                            gtsam::Similarity3& g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double, 7, 7>& mAcumHessian, const bool bAllPoints)
{
    const Eigen::Matrix3f R1w = pKF1->GetRotation();
    const Eigen::Vector3f t1w = pKF1->GetTranslation();
    const Eigen::Matrix3f R2w = pKF2->GetRotation();
    const Eigen::Vector3f t2w = pKF2->GetTranslation();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    const gtsam::Key sim3K = sim3Key(0);
    initial.insert(sim3K, g2oS12);

    const int N = static_cast<int>(vpMatches1.size());
    const std::vector<MapPoint*>& vpMapPoints1 = pKF1->GetMapPointMatches();
    std::vector<boost::shared_ptr<Sim3ProjectionFactor>> vpF12;
    std::vector<boost::shared_ptr<InverseSim3ProjectionFactor>> vpF21;
    std::vector<size_t> vnIndexEdge;
    std::vector<bool> vbIsInKF2;
    int nCorrespondences = 0;

    const double deltaHuber = std::sqrt(static_cast<double>(th2));

    for (int i = 0; i < N; i++)
    {
        if (!vpMatches1[i])
        {
            continue;
        }
        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];
        const int i2 = std::get<0>(pMP2->GetIndexInKeyFrame(pKF2));

        Eigen::Vector3d P3D1c, P3D2c;
        if (pMP1 && pMP2 && !pMP1->isBad() && !pMP2->isBad())
        {
            P3D1c = (R1w * pMP1->GetWorldPos() + t1w).cast<double>();
            P3D2c = (R2w * pMP2->GetWorldPos() + t2w).cast<double>();
        }
        else if (pMP2 && !pMP2->isBad())
        {
            P3D2c = (R2w * pMP2->GetWorldPos() + t2w).cast<double>();
            continue;
        }
        else
        {
            continue;
        }
        if (i2 < 0 && !bAllPoints)
        {
            continue;
        }
        if (P3D2c(2) <= 0)
        {
            continue;
        }
        nCorrespondences++;
        Eigen::Vector2d obs1(pKF1->mvKeysUn[i].pt.x, pKF1->mvKeysUn[i].pt.y);
        const double invSigma1 = static_cast<double>(pKF1->mvInvLevelSigma2[pKF1->mvKeysUn[i].octave]);
        auto noise1 = makeHuberNoise(2, static_cast<double>(th2), invSigma1);
        vpF12.push_back(boost::make_shared<Sim3ProjectionFactor>(sim3K, P3D2c, obs1, noise1, pKF1->mpCamera));
        graph.add(vpF12.back());

        Eigen::Vector2d obs2;
        if (i2 >= 0)
        {
            obs2 << pKF2->mvKeysUn[i2].pt.x, pKF2->mvKeysUn[i2].pt.y;
            vbIsInKF2.push_back(true);
        }
        else
        {
            obs2 << P3D2c(0) / P3D2c(2), P3D2c(1) / P3D2c(2);
            vbIsInKF2.push_back(false);
        }
        const double invSigma2 =
            (i2 >= 0) ? static_cast<double>(pKF2->mvInvLevelSigma2[pKF2->mvKeysUn[i2].octave]) : invSigma1;
        auto noise2 = makeHuberNoise(2, static_cast<double>(th2), invSigma2);
        vpF21.push_back(boost::make_shared<InverseSim3ProjectionFactor>(sim3K, P3D1c, obs2, noise2, pKF2->mpCamera));
        graph.add(vpF21.back());
        vnIndexEdge.push_back(static_cast<size_t>(i));
    }

    if (nCorrespondences < 10)
    {
        return 0;
    }
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(5);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, params);
    gtsam::Values result = opt.optimize();

    std::vector<size_t> inlierIdx;
    for (size_t i = 0; i < vpF12.size(); i++)
    {
        double e12 = 2.0 * vpF12[i]->error(result);
        double e21 = 2.0 * vpF21[i]->error(result);
        if (e12 > th2 || e21 > th2)
        {
            vpMatches1[vnIndexEdge[i]] = nullptr;
        }
        else
        {
            inlierIdx.push_back(i);
        }
    }

    int nMoreIterations = (inlierIdx.size() < 10) ? 0 : (inlierIdx.size() == vpF12.size() ? 5 : 10);
    if (nMoreIterations > 0)
    {
        gtsam::NonlinearFactorGraph graph2;
        for (size_t i : inlierIdx)
        {
            graph2.add(vpF12[i]);
            graph2.add(vpF21[i]);
        }
        initial = result;
        gtsam::LevenbergMarquardtParams params2;
        params2.setMaxIterations(nMoreIterations);
        params2.setVerbosity("SILENT");
        gtsam::LevenbergMarquardtOptimizer opt2(graph2, initial, params2);
        result = opt2.optimize();
    }

    int nIn = 0;
    mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
    for (size_t i = 0; i < vpF12.size(); i++)
    {
        if (2.0 * vpF12[i]->error(result) > th2 || 2.0 * vpF21[i]->error(result) > th2)
        {
            vpMatches1[vnIndexEdge[i]] = nullptr;
        }
        else
        {
            nIn++;
        }
    }

    gtsam::Similarity3 S12 = result.at<gtsam::Similarity3>(sim3K);
    if (bFixScale)
    {
        S12 = gtsam::Similarity3(S12.rotation(), S12.translation(), 1.0);
    }
    g2oS12 = S12;
    return nIn;
}

void Optimizer::LocalInertialBA(KeyFrame* pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF,
                                int& num_MPs, int& num_edges, bool bLarge, bool bRecInit)
{
    Map* pCurrentMap = pKF->GetMap();

    int maxOpt = 10;
    int opt_it = 10;
    if (bLarge)
    {
        maxOpt = 25;
        opt_it = 4;
    }
    const int Nd = std::min((int)pCurrentMap->KeyFramesInMap() - 2, maxOpt);
    const unsigned long maxKFid = pKF->mnId;

    std::vector<KeyFrame*> vpOptimizableKFs;
    const std::vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
    std::list<KeyFrame*> lpOptVisKFs;

    vpOptimizableKFs.reserve(Nd);
    vpOptimizableKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    for (int i = 1; i < Nd; i++)
    {
        if (vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
        }
        else
        {
            break;
        }
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by temporal optimizable keyframes
    std::list<MapPoint*> lLocalMapPoints;
    for (int i = 0; i < N; i++)
    {
        std::vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            MapPoint* pMP = *vit;
            if (pMP && !pMP->isBad() && pMP->mnBALocalForKF != pKF->mnId)
            {
                lLocalMapPoints.push_back(pMP);
                pMP->mnBALocalForKF = pKF->mnId;
            }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    std::list<KeyFrame*> lFixedKeyFrames;
    if (vpOptimizableKFs.back()->mPrevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->mnBALocalForKF = 0;
        vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Optimizable visual KFs
    const int maxCovKF = 0;
    for (int i = 0, iend = vpNeighsKFs.size(); i < iend; i++)
    {
        if (lpOptVisKFs.size() >= maxCovKF)
        {
            break;
        }
        KeyFrame* pKFi = vpNeighsKFs[i];
        if (pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
        {
            continue;
        }
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
        {
            lpOptVisKFs.push_back(pKFi);

            std::vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
            for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            {
                MapPoint* pMP = *vit;
                if (pMP && !pMP->isBad() && pMP->mnBALocalForKF != pKF->mnId)
                {
                    lLocalMapPoints.push_back(pMP);
                    pMP->mnBALocalForKF = pKF->mnId;
                }
            }
        }
    }

    // Fixed KFs which are not covisible optimizable
    const int maxFixKF = 200;

    for (std::list<MapPoint*>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        std::map<KeyFrame*, std::tuple<int, int>> observations = (*lit)->GetObservations();
        for (std::map<KeyFrame*, std::tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
            {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad())
                {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if (lFixedKeyFrames.size() >= maxFixKF)
        {
            break;
        }
    }

    bool bNonFixed = (lFixedKeyFrames.size() == 0);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::KeyVector landmarkKeys;
    const unsigned long iniMPid = maxKFid * 5;

    auto addKFToGraph = [&](KeyFrame* pKFi, bool fixed)
    {
        gtsam::Pose3 Twb = sophusToGTSAMPose(pKFi->GetImuPose());
        initial.insert(imuPoseKey(static_cast<uint32_t>(pKFi->mnId)), Twb);
        if (fixed)
        {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(imuPoseKey(static_cast<uint32_t>(pKFi->mnId)), Twb,
                                                       gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)));
        }
        if (pKFi->bImu)
        {
            initial.insert(velKey(static_cast<uint32_t>(pKFi->mnId)),
                           Eigen::Vector3d(pKFi->GetVelocity().cast<double>()));
            gtsam::Key bk = biasKey(static_cast<uint32_t>(maxKFid + 3 * (pKFi->mnId) + 2));
            initial.insert(bk, toGTSAMBias(pKFi->GetImuBias()));
            if (fixed)
            {
                graph.add(gtsam::PriorFactor<gtsam::Vector3>(velKey(static_cast<uint32_t>(pKFi->mnId)),
                                                             pKFi->GetVelocity().cast<double>(),
                                                             gtsam::noiseModel::Isotropic::Sigma(3, 1e-6)));
                graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
                    bk, toGTSAMBias(pKFi->GetImuBias()), gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)));
            }
        }
    };

    N = static_cast<int>(vpOptimizableKFs.size());
    for (int i = 0; i < N; i++)
    {
        addKFToGraph(vpOptimizableKFs[i], false);
    }
    for (KeyFrame* pKFi : lpOptVisKFs)
    {
        addKFToGraph(pKFi, false);
    }
    for (KeyFrame* pKFi : lFixedKeyFrames)
    {
        addKFToGraph(pKFi, true);
    }
    for (int i = 0; i < N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];
        if (!pKFi->mPrevKF || !pKFi->bImu || !pKFi->mPrevKF->bImu || !pKFi->mpImuPreintegrated)
        {
            continue;
        }
        pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
        gtsam::Key p1 = imuPoseKey(static_cast<uint32_t>(pKFi->mPrevKF->mnId));
        gtsam::Key v1 = velKey(static_cast<uint32_t>(pKFi->mPrevKF->mnId));
        gtsam::Key b1 = biasKey(static_cast<uint32_t>(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2));
        gtsam::Key p2 = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
        gtsam::Key v2 = velKey(static_cast<uint32_t>(pKFi->mnId));
        gtsam::Key b2 = biasKey(static_cast<uint32_t>(maxKFid + 3 * (pKFi->mnId) + 2));
        graph.add(boost::make_shared<InertialFactor>(p1, v1, b1, p2, v2, pKFi->mpImuPreintegrated));
        gtsam::Matrix6 Info6 = gtsam::Matrix6::Zero();
        Info6.block<3, 3>(0, 0) = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        Info6.block<3, 3>(3, 3) = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            b1, b2, gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Gaussian::Information(Info6)));
    }

    std::map<int, int> mVisEdges;
    for (int i = 0; i < N; i++)
    {
        mVisEdges[vpOptimizableKFs[i]->mnId] = 0;
    }
    for (KeyFrame* pKFi : lFixedKeyFrames)
    {
        mVisEdges[pKFi->mnId] = 0;
    }
    std::vector<std::tuple<KeyFrame*, MapPoint*, bool>> monoEdges;
    std::vector<std::tuple<KeyFrame*, MapPoint*>> stereoEdges;
    for (MapPoint* pMP : lLocalMapPoints)
    {
        gtsam::Point3 Xw(pMP->GetWorldPos().cast<double>());
        gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + iniMPid + 1));
        initial.insert(pk, Xw);
        landmarkKeys.push_back(pk);
        for (const auto& mit : pMP->GetObservations())
        {
            KeyFrame* pKFi = mit.first;
            if ((pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) || pKFi->isBad() ||
                pKFi->GetMap() != pCurrentMap)
            {
                continue;
            }
            const int leftIndex = std::get<0>(mit.second);
            if (leftIndex == -1)
            {
                continue;
            }
            gtsam::Key poseK = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
            gtsam::Pose3 Tbc_kf = sophusToGTSAMPose(pKFi->mImuCalib.mTbc);
            double invSigma2 = static_cast<double>(pKFi->mvInvLevelSigma2[pKFi->mvKeysUn[leftIndex].octave]);
            if (pKFi->mpCamera->GetType() == GeometricCamera::CAM_PINHOLE)
            {
                Eigen::Vector2d obs(pKFi->mvKeysUn[leftIndex].pt.x, pKFi->mvKeysUn[leftIndex].pt.y);
                invSigma2 /= static_cast<double>(pKFi->mpCamera->uncertainty2(obs));
            }
            if (pKFi->mvuRight[leftIndex] < 0)
            {
                mVisEdges[pKFi->mnId]++;
                Eigen::Vector2d obs(pKFi->mvKeysUn[leftIndex].pt.x, pKFi->mvKeysUn[leftIndex].pt.y);
                gtsam::SharedNoiseModel noise = makeHuberNoise(2, 5.991, invSigma2);
                auto cal = boost::make_shared<gtsam::Cal3_S2>(toGTSAMCal(pKFi->mpCamera));
                graph.add(
                    boost::make_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                        obs, noise, poseK, pk, cal, Tbc_kf));
                monoEdges.push_back(std::make_tuple(pKFi, pMP, pMP->mTrackDepth < 10.f));
            }
            else
            {
                mVisEdges[pKFi->mnId]++;
                gtsam::StereoPoint2 obs(pKFi->mvKeysUn[leftIndex].pt.x, pKFi->mvKeysUn[leftIndex].pt.y,
                                        pKFi->mvuRight[leftIndex]);
                if (pKFi->mpCamera->GetType() == GeometricCamera::CAM_PINHOLE)
                {
                    invSigma2 /= static_cast<double>(pKFi->mpCamera->uncertainty2(Eigen::Vector2d(obs.uL(), obs.v())));
                }
                gtsam::SharedNoiseModel noise = makeHuberNoise(3, 7.815, invSigma2);
                auto cal = boost::make_shared<gtsam::Cal3_S2Stereo>(toGTSAMStereoCal(pKFi->mpCamera, pKFi->mbf));
                graph.add(boost::make_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(obs, noise, poseK,
                                                                                                      pk, cal, Tbc_kf));
                stereoEdges.push_back(std::make_tuple(pKFi, pMP));
            }
        }
    }

    for (const auto& me : mVisEdges)
    {
        assert(me.second >= 3);
    }
    double err0 = graph.error(initial);
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(opt_it);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    if (bLarge)
    {
        params.setlambdaInitial(1e-2);
    }
    else
    {
        params.setlambdaInitial(1e0);
    }
    gtsam::Ordering ordering = gtsam::Ordering::ColamdConstrainedFirst(graph, landmarkKeys);
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, ordering, params);
    gtsam::Values result;
    if (pbStopFlag)
    {
        for (int it = 0; it < opt_it; it++)
        {
            if (*pbStopFlag)
            {
                break;
            }
            opt.iterate();
        }
        result = opt.values();
    }
    else
    {
        result = opt.optimize();
    }
    double err_end = graph.error(result);

    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    if ((2 * err0 < err_end || std::isnan(err0) || std::isnan(err_end)) && !bLarge)
    {
        return;
    }
    size_t monoIdx = 0, stereoIdx = 0;
    for (const auto& f : graph)
    {
        if (dynamic_cast<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>*>(f.get()))
        {
            if (monoIdx < monoEdges.size())
            {
                KeyFrame* pKFi = std::get<0>(monoEdges[monoIdx]);
                MapPoint* pMP = std::get<1>(monoEdges[monoIdx]);
                bool bClose = std::get<2>(monoEdges[monoIdx]);
                double chi2 = 2.0 * f->error(result);
                if (!pMP->isBad() && ((chi2 > 5.991 && !bClose) || (chi2 > 1.5 * 5.991 && bClose)))
                {
                    pKFi->EraseMapPointMatch(pMP);
                    pMP->EraseObservation(pKFi);
                }
                monoIdx++;
            }
        }
        else if (dynamic_cast<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>*>(f.get()))
        {
            if (stereoIdx < stereoEdges.size())
            {
                KeyFrame* pKFi = std::get<0>(stereoEdges[stereoIdx]);
                MapPoint* pMP = std::get<1>(stereoEdges[stereoIdx]);
                if (!pMP->isBad() && 2.0 * f->error(result) > 7.815)
                {
                    pKFi->EraseMapPointMatch(pMP);
                    pMP->EraseObservation(pKFi);
                }
                stereoIdx++;
            }
        }
    }

    for (KeyFrame* pKFi : lFixedKeyFrames)
    {
        pKFi->mnBAFixedForKF = 0;
    }
    for (KeyFrame* pKFi : vpOptimizableKFs)
    {
        gtsam::Pose3 Twb = result.at<gtsam::Pose3>(imuPoseKey(static_cast<uint32_t>(pKFi->mnId)));
        Sophus::SE3f Tcw = (gtsamToSophusPose(Twb) * pKFi->mImuCalib.mTbc).inverse();
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF = 0;
        if (pKFi->bImu)
        {
            pKFi->SetVelocity(result.at<gtsam::Vector3>(velKey(static_cast<uint32_t>(pKFi->mnId))).cast<float>());
            pKFi->SetNewBias(fromGTSAMBias(result.at<gtsam::imuBias::ConstantBias>(
                biasKey(static_cast<uint32_t>(maxKFid + 3 * (pKFi->mnId) + 2)))));
        }
    }
    for (KeyFrame* pKFi : lpOptVisKFs)
    {
        gtsam::Pose3 Twb = result.at<gtsam::Pose3>(imuPoseKey(static_cast<uint32_t>(pKFi->mnId)));
        pKFi->SetPose((gtsamToSophusPose(Twb) * pKFi->mImuCalib.mTbc).inverse());
        pKFi->mnBALocalForKF = 0;
    }
    for (MapPoint* pMP : lLocalMapPoints)
    {
        gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + iniMPid + 1));
        if (result.exists(pk))
        {
            pMP->SetWorldPos(result.at<gtsam::Point3>(pk).cast<float>());
            pMP->UpdateNormalAndDepth();
        }
    }
    pMap->IncreaseChangeIndex();
}

Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd& H, const int& start, const int& end)
{
    // Goal
    // a  | ab | ac       a*  | 0 | ac*
    // ba | b  | bc  -->  0   | 0 | 0
    // ca | cb | c        ca* | 0 | c*

    // Size of block before block to marginalize
    const int a = start;
    // Size of block to marginalize
    const int b = end - start + 1;
    // Size of block after block to marginalize
    const int c = H.cols() - (end + 1);

    // Reorder as follows:
    // a  | ab | ac       a  | ac | ab
    // ba | b  | bc  -->  ca | c  | cb
    // ca | cb | c        ba | bc | b

    Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
    if (a > 0)
    {
        Hn.block(0, 0, a, a) = H.block(0, 0, a, a);
        Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
        Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
    }
    if (a > 0 && c > 0)
    {
        Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
        Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
    }
    if (c > 0)
    {
        Hn.block(a, a, c, c) = H.block(a + b, a + b, c, c);
        Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
        Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
    }
    Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

    // Perform marginalization (Schur complement)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a + c, a + c, b, b), Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
    for (int i = 0; i < b; ++i)
    {
        if (singularValues_inv(i) > 1e-6)
        {
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        }
        else
        {
            singularValues_inv(i) = 0;
        }
    }
    Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
    Hn.block(0, 0, a + c, a + c) =
        Hn.block(0, 0, a + c, a + c) - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
    Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
    Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
    Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

    // Inverse reorder
    // a*  | ac* | 0       a*  | 0 | ac*
    // ca* | c*  | 0  -->  0   | 0 | 0
    // 0   | 0   | 0       ca* | 0 | c*
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
    if (a > 0)
    {
        res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
        res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
        res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
    }
    if (a > 0 && c > 0)
    {
        res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
        res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
    }
    if (c > 0)
    {
        res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
        res.block(a + b, a, c, b) = Hn.block(a, a + c, c, b);
        res.block(a, a + b, b, c) = Hn.block(a + c, a, b, c);
    }

    res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

    return res;
}

void Optimizer::InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale, Eigen::Vector3d& bg,
                                     Eigen::Vector3d& ba, bool bMono, Eigen::MatrixXd& covInertial, bool bFixedVel,
                                     bool bGauss, float priorG, float priorA)
{
    (void)bMono;
    (void)covInertial;
    (void)bFixedVel;
    (void)bGauss;
    InertialOptimizationImpl(pMap, InertialOptMode::Full, &Rwg, &scale, &bg, &ba, priorG, priorA);
}

void Optimizer::InertialOptimization(Map* pMap, Eigen::Vector3d& bg, Eigen::Vector3d& ba, float priorG, float priorA)
{
    InertialOptimizationImpl(pMap, InertialOptMode::BiasOnly, nullptr, nullptr, &bg, &ba, priorG, priorA);
}

void Optimizer::InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale)
{
    InertialOptimizationImpl(pMap, InertialOptMode::GravityScaleOnly, &Rwg, &scale, nullptr, nullptr, 1e2f, 1e6f);
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pMainKF, std::vector<KeyFrame*> vpAdjustKF,
                                      std::vector<KeyFrame*> vpFixedKF, bool* pbStopFlag)
{
    (void)pbStopFlag;
    std::vector<MapPoint*> vpMPs;
    long unsigned int maxKFid = 0;
    Map* pCurrentMap = pMainKF->GetMap();

    for (KeyFrame* pKFi : vpFixedKF)
    {
        if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            continue;
        }
        pKFi->mnBALocalForMerge = pMainKF->mnId;
        if (pKFi->mnId > maxKFid)
        {
            maxKFid = pKFi->mnId;
        }
        for (MapPoint* pMPi : pKFi->GetMapPoints())
        {
            if (pMPi && !pMPi->isBad() && pMPi->GetMap() == pCurrentMap && pMPi->mnBALocalForMerge != pMainKF->mnId)
            {
                vpMPs.push_back(pMPi);
                pMPi->mnBALocalForMerge = pMainKF->mnId;
            }
        }
    }
    for (KeyFrame* pKFi : vpAdjustKF)
    {
        if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            continue;
        }
        pKFi->mnBALocalForMerge = pMainKF->mnId;
        if (pKFi->mnId > maxKFid)
        {
            maxKFid = pKFi->mnId;
        }
        for (MapPoint* pMPi : pKFi->GetMapPoints())
        {
            if (pMPi && !pMPi->isBad() && pMPi->GetMap() == pCurrentMap && pMPi->mnBALocalForMerge != pMainKF->mnId)
            {
                vpMPs.push_back(pMPi);
                pMPi->mnBALocalForMerge = pMainKF->mnId;
            }
        }
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::KeyVector landmarkKeys;
    std::map<KeyFrame*, int> mpObsKFs;
    std::map<MapPoint*, int> mpObsMPs;
    struct EdgeRec
    {
        KeyFrame* pKF;
        MapPoint* pMP;
        boost::shared_ptr<gtsam::NonlinearFactor> factor;
    };
    std::vector<EdgeRec> monoRecs, stereoRecs;

    for (KeyFrame* pKFi : vpFixedKF)
    {
        if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            continue;
        }
        Sophus::SE3f Tcw = pKFi->GetPose();
        gtsam::Pose3 Twc = sophusToGTSAMPose(Tcw.inverse());
        gtsam::Key k = poseKey(static_cast<uint32_t>(pKFi->mnId));
        initial.insert(k, Twc);
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(k, Twc, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)));
    }
    for (KeyFrame* pKFi : vpAdjustKF)
    {
        if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            continue;
        }
        gtsam::Pose3 Twc = sophusToGTSAMPose(pKFi->GetPose().inverse());
        initial.insert(poseKey(static_cast<uint32_t>(pKFi->mnId)), Twc);
    }

    for (MapPoint* pMPi : vpMPs)
    {
        if (pMPi->isBad())
        {
            continue;
        }
        gtsam::Point3 Xw(pMPi->GetWorldPos().cast<double>());
        gtsam::Key pk = pointKey(static_cast<uint32_t>(pMPi->mnId + maxKFid + 1));
        initial.insert(pk, Xw);
        landmarkKeys.push_back(pk);

        for (const auto& mit : pMPi->GetObservations())
        {
            KeyFrame* pKF = mit.first;
            if (pKF->isBad() || pKF->mnBALocalForMerge != pMainKF->mnId || !pKF->GetMapPoint(std::get<0>(mit.second)))
            {
                continue;
            }
            const int idx = std::get<0>(mit.second);
            const cv::KeyPoint& kpUn = pKF->mvKeysUn[idx];
            double invSigma2 = static_cast<double>(pKF->mvInvLevelSigma2[kpUn.octave]);
            gtsam::Key poseK = poseKey(static_cast<uint32_t>(pKF->mnId));

            if (pKF->mvuRight[idx] < 0)
            {
                mpObsMPs[pMPi]++;
                Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                gtsam::SharedNoiseModel noise = makeHuberNoise(2, 5.991, invSigma2);
                boost::shared_ptr<gtsam::NonlinearFactor> f;
                auto cal = boost::make_shared<gtsam::Cal3_S2>(toGTSAMCal(pKF->mpCamera));
                f = boost::make_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                    obs, noise, poseK, pk, cal);
                graph.add(f);
                monoRecs.push_back({pKF, pMPi, f});
                mpObsKFs[pKF]++;
            }
            else
            {
                mpObsMPs[pMPi] += 2;
                gtsam::StereoPoint2 obs(kpUn.pt.x, kpUn.pt.y, pKF->mvuRight[idx]);
                gtsam::SharedNoiseModel noise = makeHuberNoise(3, 7.815, invSigma2);
                boost::shared_ptr<gtsam::NonlinearFactor> f;
                auto cal = boost::make_shared<gtsam::Cal3_S2Stereo>(toGTSAMStereoCal(pKF->mpCamera, pKF->mbf));
                f = boost::make_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(obs, noise, poseK, pk,
                                                                                                cal);
                graph.add(f);
                stereoRecs.push_back({pKF, pMPi, f});
                mpObsKFs[pKF]++;
            }
        }
    }

    if (pbStopFlag && *pbStopFlag)
    {
        return;
    }
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(10);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    gtsam::Ordering ordering = gtsam::Ordering::ColamdConstrainedFirst(graph, landmarkKeys);
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, ordering, params);
    gtsam::Values result = opt.optimize();

    std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
    for (const EdgeRec& r : monoRecs)
    {
        if (r.pMP->isBad())
        {
            continue;
        }
        double err = 2.0 * r.factor->error(result);
        if (err > 5.991)
        {
            vToErase.push_back({r.pKF, r.pMP});
        }
    }
    for (const EdgeRec& r : stereoRecs)
    {
        if (r.pMP->isBad())
        {
            continue;
        }
        if (2.0 * r.factor->error(result) > 7.815)
        {
            vToErase.push_back({r.pKF, r.pMP});
        }
    }

    std::unique_lock<std::mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);
    for (const auto& p : vToErase)
    {
        p.first->EraseMapPointMatch(p.second);
        p.second->EraseObservation(p.first);
    }

    for (KeyFrame* pKFi : vpAdjustKF)
    {
        if (pKFi->isBad())
        {
            continue;
        }
        gtsam::Key k = poseKey(static_cast<uint32_t>(pKFi->mnId));
        if (result.exists(k))
        {
            pKFi->SetPose(gtsamToSophusPose(result.at<gtsam::Pose3>(k)).inverse());
        }
    }
    for (MapPoint* pMPi : vpMPs)
    {
        if (pMPi->isBad())
        {
            continue;
        }
        gtsam::Key pk = pointKey(static_cast<uint32_t>(pMPi->mnId + maxKFid + 1));
        if (result.exists(pk))
        {
            pMPi->SetWorldPos(result.at<gtsam::Point3>(pk).cast<float>());
            pMPi->UpdateNormalAndDepth();
        }
    }
}

void Optimizer::MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool* pbStopFlag, Map* pMap,
                                LoopClosing::KeyFrameAndPose& corrPoses)
{
    const int Nd = 6;
    const unsigned long maxKFid = pCurrKF->mnId;

    std::vector<KeyFrame*> vpOptimizableKFs;
    vpOptimizableKFs.reserve(2 * Nd);

    // For cov KFS, inertial parameters are not optimized
    const int maxCovKF = 30;
    std::vector<KeyFrame*> vpOptimizableCovKFs;
    vpOptimizableCovKFs.reserve(maxCovKF);

    // Add sliding window for current KF
    vpOptimizableKFs.push_back(pCurrKF);
    pCurrKF->mnBALocalForKF = pCurrKF->mnId;
    for (int i = 1; i < Nd; i++)
    {
        if (vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }
        else
        {
            break;
        }
    }

    std::list<KeyFrame*> lFixedKeyFrames;
    if (vpOptimizableKFs.back()->mPrevKF)
    {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
    }
    else
    {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Add temporal neighbours to merge KF (previous and next KFs)
    vpOptimizableKFs.push_back(pMergeKF);
    pMergeKF->mnBALocalForKF = pCurrKF->mnId;

    // Previous KFs
    for (int i = 1; i < (Nd / 2); i++)
    {
        if (vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }
        else
        {
            break;
        }
    }

    // We fix just once the old map
    if (vpOptimizableKFs.back()->mPrevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->mnBALocalForKF = 0;
        vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Next KFs
    if (pMergeKF->mNextKF)
    {
        vpOptimizableKFs.push_back(pMergeKF->mNextKF);
        vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    }

    while (vpOptimizableKFs.size() < (2 * Nd))
    {
        if (vpOptimizableKFs.back()->mNextKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }
        else
        {
            break;
        }
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by optimizable keyframes
    std::list<MapPoint*> lLocalMapPoints;
    std::map<MapPoint*, int> mLocalObs;
    for (int i = 0; i < N; i++)
    {
        std::vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for (std::vector<MapPoint*>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
        {
            // Using mnBALocalForKF we avoid redundance here, one MP can not be added several times to lLocalMapPoints
            MapPoint* pMP = *vit;
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    if (pMP->mnBALocalForKF != pCurrKF->mnId)
                    {
                        mLocalObs[pMP] = 1;
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pCurrKF->mnId;
                    }
                    else
                    {
                        mLocalObs[pMP]++;
                    }
                }
            }
        }
    }

    std::vector<std::pair<MapPoint*, int>> pairs;
    pairs.reserve(mLocalObs.size());
    for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr)
    {
        pairs.push_back(*itr);
    }
    sort(pairs.begin(), pairs.end(), sortByVal);

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    int i = 0;
    for (std::vector<std::pair<MapPoint*, int>>::iterator lit = pairs.begin(), lend = pairs.end(); lit != lend;
         lit++, i++)
    {
        std::map<KeyFrame*, std::tuple<int, int>> observations = lit->first->GetObservations();
        if (i >= maxCovKF)
        {
            break;
        }
        for (std::map<KeyFrame*, std::tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pCurrKF->mnId &&
                pKFi->mnBAFixedForKF != pCurrKF->mnId)  // If optimizable or already included...
            {
                pKFi->mnBALocalForKF = pCurrKF->mnId;
                if (!pKFi->isBad())
                {
                    vpOptimizableCovKFs.push_back(pKFi);
                    break;
                }
            }
        }
    }

    const unsigned long iniMPid = maxKFid * 5;
    const int Ncov = static_cast<int>(vpOptimizableCovKFs.size());
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::KeyVector landmarkKeys;
    gtsam::Pose3 Tbc = sophusToGTSAMPose(pCurrKF->mImuCalib.mTbc);

    auto addKFPrior = [&](KeyFrame* pKFi)
    {
        gtsam::Pose3 Twb = sophusToGTSAMPose(pKFi->GetImuPose());
        gtsam::Key pk = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
        initial.insert(pk, Twb);
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(pk, Twb, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)));
        if (pKFi->bImu)
        {
            gtsam::Key vk = velKey(static_cast<uint32_t>(pKFi->mnId));
            gtsam::Key bk = biasKey(static_cast<uint32_t>(pKFi->mnId));
            initial.insert(vk, Eigen::Vector3d(pKFi->GetVelocity().cast<double>()));
            initial.insert(bk, toGTSAMBias(pKFi->GetImuBias()));
            graph.add(gtsam::PriorFactor<gtsam::Vector3>(vk, Eigen::Vector3d(pKFi->GetVelocity().cast<double>()),
                                                         gtsam::noiseModel::Isotropic::Sigma(3, 1e-6)));
            graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(bk, toGTSAMBias(pKFi->GetImuBias()),
                                                                       gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)));
        }
    };

    for (KeyFrame* pKFi : vpOptimizableKFs)
    {
        gtsam::Pose3 Twb = sophusToGTSAMPose(pKFi->GetImuPose());
        initial.insert(imuPoseKey(static_cast<uint32_t>(pKFi->mnId)), Twb);
        if (pKFi->bImu)
        {
            initial.insert(velKey(static_cast<uint32_t>(pKFi->mnId)),
                           Eigen::Vector3d(pKFi->GetVelocity().cast<double>()));
            initial.insert(biasKey(static_cast<uint32_t>(pKFi->mnId)), toGTSAMBias(pKFi->GetImuBias()));
        }
    }
    for (KeyFrame* pKFi : vpOptimizableCovKFs)
    {
        gtsam::Pose3 Twb = sophusToGTSAMPose(pKFi->GetImuPose());
        initial.insert(imuPoseKey(static_cast<uint32_t>(pKFi->mnId)), Twb);
        if (pKFi->bImu)
        {
            initial.insert(velKey(static_cast<uint32_t>(pKFi->mnId)),
                           Eigen::Vector3d(pKFi->GetVelocity().cast<double>()));
            initial.insert(biasKey(static_cast<uint32_t>(pKFi->mnId)), toGTSAMBias(pKFi->GetImuBias()));
        }
    }
    for (KeyFrame* pKFi : lFixedKeyFrames)
    {
        addKFPrior(pKFi);
    }
    for (int i = 0; i < N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];
        if (!pKFi->mPrevKF || !pKFi->bImu || !pKFi->mPrevKF->bImu || !pKFi->mpImuPreintegrated)
        {
            continue;
        }
        pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
        gtsam::Key p1 = imuPoseKey(static_cast<uint32_t>(pKFi->mPrevKF->mnId));
        gtsam::Key v1 = velKey(static_cast<uint32_t>(pKFi->mPrevKF->mnId));
        gtsam::Key b1 = biasKey(static_cast<uint32_t>(pKFi->mPrevKF->mnId));
        gtsam::Key p2 = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
        gtsam::Key v2 = velKey(static_cast<uint32_t>(pKFi->mnId));
        gtsam::Key b2 = biasKey(static_cast<uint32_t>(pKFi->mnId));
        graph.add(boost::make_shared<InertialFactor>(p1, v1, b1, p2, v2, pKFi->mpImuPreintegrated));
        Eigen::Matrix<double, 6, 6> Info6 = Eigen::Matrix<double, 6, 6>::Zero();
        Info6.block<3, 3>(0, 0) = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        Info6.block<3, 3>(3, 3) = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            b1, b2, gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Gaussian::Information(Info6)));
    }

    struct VisRec
    {
        KeyFrame* pKF;
        MapPoint* pMP;
        boost::shared_ptr<gtsam::NonlinearFactor> f;
    };
    std::vector<VisRec> monoRecs, stereoRecs;
    for (MapPoint* pMP : lLocalMapPoints)
    {
        if (!pMP)
        {
            continue;
        }
        gtsam::Point3 Xw(pMP->GetWorldPos().cast<double>());
        gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + iniMPid + 1));
        initial.insert(pk, Xw);
        landmarkKeys.push_back(pk);
        for (const auto& mit : pMP->GetObservations())
        {
            KeyFrame* pKFi = mit.first;
            if (!pKFi || ((pKFi->mnBALocalForKF != pCurrKF->mnId) && (pKFi->mnBAFixedForKF != pCurrKF->mnId)) ||
                pKFi->mnId > maxKFid || pKFi->isBad())
            {
                continue;
            }
            int idx = std::get<0>(mit.second);
            const cv::KeyPoint& kpUn = pKFi->mvKeysUn[idx];
            double invSigma2 = static_cast<double>(pKFi->mvInvLevelSigma2[kpUn.octave]);
            gtsam::Key poseK = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
            if (pKFi->mvuRight[idx] < 0)
            {
                Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                gtsam::SharedNoiseModel noise = makeHuberNoise(2, 5.991, invSigma2);
                boost::shared_ptr<gtsam::NonlinearFactor> f;
                f = boost::make_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                    obs, noise, poseK, pk, boost::make_shared<gtsam::Cal3_S2>(toGTSAMCal(pKFi->mpCamera)), Tbc);
                graph.add(f);
                monoRecs.push_back({pKFi, pMP, f});
            }
            else
            {
                gtsam::StereoPoint2 obs(kpUn.pt.x, kpUn.pt.y, pKFi->mvuRight[idx]);
                gtsam::SharedNoiseModel noise = makeHuberNoise(3, 7.815, invSigma2);
                boost::shared_ptr<gtsam::NonlinearFactor> f;
                f = boost::make_shared<gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
                    obs, noise, poseK, pk,
                    boost::make_shared<gtsam::Cal3_S2Stereo>(toGTSAMStereoCal(pKFi->mpCamera, pKFi->mbf)), Tbc);
                graph.add(f);
                stereoRecs.push_back({pKFi, pMP, f});
            }
        }
    }

    if (pbStopFlag && *pbStopFlag)
    {
        return;
    }
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(8);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    gtsam::Ordering ordering = gtsam::Ordering::ColamdConstrainedFirst(graph, landmarkKeys);
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, ordering, params);
    gtsam::Values result = opt.optimize();

    std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
    for (const VisRec& r : monoRecs)
    {
        if (r.pMP->isBad())
        {
            continue;
        }
        if (2.0 * r.f->error(result) > 5.991)
        {
            vToErase.push_back({r.pKF, r.pMP});
        }
    }
    for (const VisRec& r : stereoRecs)
    {
        if (r.pMP->isBad())
        {
            continue;
        }
        if (2.0 * r.f->error(result) > 7.815)
        {
            vToErase.push_back({r.pKF, r.pMP});
        }
    }
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    for (const auto& p : vToErase)
    {
        p.first->EraseMapPointMatch(p.second);
        p.second->EraseObservation(p.first);
    }

    auto setKFFromResult = [&](KeyFrame* pKFi)
    {
        gtsam::Key pk = imuPoseKey(static_cast<uint32_t>(pKFi->mnId));
        if (!result.exists(pk))
        {
            return;
        }
        gtsam::Pose3 Twb = result.at<gtsam::Pose3>(pk);
        gtsam::Pose3 Tcw = (Twb * Tbc).inverse();
        pKFi->SetPose(gtsamToSophusPose(Tcw));
        corrPoses[pKFi] = toGTSAMSim3(Sophus::Sim3f(Sophus::RxSO3f(1.0f, Tcw.rotation().matrix().cast<float>()),
                                                    Tcw.translation().cast<float>()));
        if (pKFi->bImu)
        {
            gtsam::Key vk = velKey(static_cast<uint32_t>(pKFi->mnId));
            gtsam::Key bk = biasKey(static_cast<uint32_t>(pKFi->mnId));
            if (result.exists(vk))
            {
                pKFi->SetVelocity(result.at<gtsam::Vector3>(vk).cast<float>());
            }
            if (result.exists(bk))
            {
                pKFi->SetNewBias(fromGTSAMBias(result.at<gtsam::imuBias::ConstantBias>(bk)));
            }
        }
    };
    for (int i = 0; i < N; i++)
    {
        setKFFromResult(vpOptimizableKFs[i]);
    }
    for (int i = 0; i < Ncov; i++)
    {
        setKFFromResult(vpOptimizableCovKFs[i]);
    }
    for (MapPoint* pMP : lLocalMapPoints)
    {
        gtsam::Key pk = pointKey(static_cast<uint32_t>(pMP->mnId + iniMPid + 1));
        if (result.exists(pk))
        {
            pMP->SetWorldPos(result.at<gtsam::Point3>(pk).cast<float>());
            pMP->UpdateNormalAndDepth();
        }
    }
    pMap->IncreaseChangeIndex();
}

int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit)
{
    const gtsam::Key poseK0 = imuPoseKey(0);
    const gtsam::Key velK1 = velKey(1);
    const gtsam::Key biasK2 = biasKey(2);
    const gtsam::Key poseK4 = imuPoseKey(4);
    const gtsam::Key velK5 = velKey(5);
    const gtsam::Key biasK6 = biasKey(6);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    gtsam::Pose3 Twb_frame = sophusToGTSAMPose(pFrame->GetImuPose());
    initial.insert(poseK0, Twb_frame);
    initial.insert(velK1, Eigen::Vector3d(pFrame->GetVelocity().cast<double>()));
    initial.insert(biasK2, toGTSAMBias(pFrame->mImuBias));

    KeyFrame* pKF = pFrame->mpLastKeyFrame;
    gtsam::Pose3 Twb_kf = sophusToGTSAMPose(pKF->GetImuPose());
    initial.insert(poseK4, Twb_kf);
    initial.insert(velK5, Eigen::Vector3d(pKF->GetVelocity().cast<double>()));
    initial.insert(biasK6, toGTSAMBias(pKF->GetImuBias()));
    auto priorP4 = boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(poseK4, Twb_kf,
                                                                        gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
    auto priorV5 = boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(velK5, pKF->GetVelocity().cast<double>(),
                                                                          gtsam::noiseModel::Isotropic::Sigma(3, 1e-6));
    auto priorB6 = boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        biasK6, toGTSAMBias(pKF->GetImuBias()), gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
    graph.add(priorP4);
    graph.add(priorV5);
    graph.add(priorB6);

    pFrame->mpImuPreintegrated->SetNewBias(pKF->GetImuBias());
    auto inertialFactor =
        boost::make_shared<InertialFactor>(poseK4, velK5, biasK6, poseK0, velK1, pFrame->mpImuPreintegrated);
    graph.add(inertialFactor);
    gtsam::Matrix6 Info6 = gtsam::Matrix6::Zero();
    Info6.block<3, 3>(0, 0) = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
    Info6.block<3, 3>(3, 3) = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
    auto betweenBias = boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        biasK6, biasK2, gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Gaussian::Information(Info6));
    graph.add(betweenBias);

    const int N = pFrame->N;
    const int Nleft = pFrame->Nleft;
    const bool bRight = (Nleft != -1);
    std::vector<boost::shared_ptr<MonoOnlyPoseFactor>> vpFactorsMono;
    std::vector<boost::shared_ptr<StereoOnlyPoseFactor>> vpFactorsStereo;
    std::vector<size_t> vnIndexMono, vnIndexStereo;
    int nInitialCorrespondences = 0;
    gtsam::Pose3 Tbc = sophusToGTSAMPose(pFrame->mImuCalib.mTbc);

    {
        std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);
        for (int i = 0; i < N; i++)
        {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if (!pMP)
            {
                continue;
            }
            Eigen::Vector3d Xw = pMP->GetWorldPos().cast<double>();
            if ((!bRight && pFrame->mvuRight[i] < 0) || (bRight && i < Nleft))
            {
                cv::KeyPoint kpUn = (bRight && i < Nleft) ? pFrame->mvKeys[i] : pFrame->mvKeysUn[i];
                Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                double invSigma2 =
                    static_cast<double>(pFrame->mvInvLevelSigma2[kpUn.octave] / pFrame->mpCamera->uncertainty2(obs));
                pFrame->mvbOutlier[i] = false;
                vpFactorsMono.push_back(boost::make_shared<MonoOnlyPoseFactor>(
                    poseK0, Xw, obs, makeHuberNoise(2, 5.991, invSigma2), pFrame->mpCamera, Tbc));
                graph.add(vpFactorsMono.back());
                vnIndexMono.push_back(static_cast<size_t>(i));
                nInitialCorrespondences++;
            }
            else if (!bRight)
            {
                const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
                Eigen::Vector3d obs(kpUn.pt.x, kpUn.pt.y, pFrame->mvuRight[i]);
                double invSigma2 = static_cast<double>(pFrame->mvInvLevelSigma2[kpUn.octave] /
                                                       pFrame->mpCamera->uncertainty2(Eigen::Vector2d(obs(0), obs(1))));
                pFrame->mvbOutlier[i] = false;
                vpFactorsStereo.push_back(boost::make_shared<StereoOnlyPoseFactor>(
                    poseK0, Xw, obs, pFrame->mbf, makeHuberNoise(3, 7.815, invSigma2), pFrame->mpCamera, Tbc));
                graph.add(vpFactorsStereo.back());
                vnIndexStereo.push_back(static_cast<size_t>(i));
                nInitialCorrespondences++;
            }
            if (bRight && i >= Nleft)
            {
                const cv::KeyPoint& kpUn = pFrame->mvKeysRight[i - Nleft];
                Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                double invSigma2 =
                    static_cast<double>(pFrame->mvInvLevelSigma2[kpUn.octave] / pFrame->mpCamera->uncertainty2(obs));
                pFrame->mvbOutlier[i] = false;
                vpFactorsMono.push_back(boost::make_shared<MonoOnlyPoseFactor>(
                    poseK0, Xw, obs, makeHuberNoise(2, 5.991, invSigma2), pFrame->mpCamera, Tbc));
                graph.add(vpFactorsMono.back());
                vnIndexMono.push_back(static_cast<size_t>(i));
                nInitialCorrespondences++;
            }
        }
    }

    const double chi2Mono[4] = {12, 7.5, 5.991, 5.991};
    const double chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};
    const int its[4] = {10, 10, 10, 10};
    int nBad = 0;

    for (int it = 0; it < 4; it++)
    {
        gtsam::NonlinearFactorGraph activeGraph;
        for (size_t i = 0; i < vpFactorsMono.size(); i++)
        {
            if (!pFrame->mvbOutlier[vnIndexMono[i]])
            {
                activeGraph.add(vpFactorsMono[i]);
            }
        }
        for (size_t i = 0; i < vpFactorsStereo.size(); i++)
        {
            if (!pFrame->mvbOutlier[vnIndexStereo[i]])
            {
                activeGraph.add(vpFactorsStereo[i]);
            }
        }
        activeGraph.add(priorP4);
        activeGraph.add(priorV5);
        activeGraph.add(priorB6);
        activeGraph.add(inertialFactor);
        activeGraph.add(betweenBias);
        if (activeGraph.size() < 10)
        {
            break;
        }
        gtsam::LevenbergMarquardtParams params;
        params.setMaxIterations(its[it]);
        params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
        gtsam::LevenbergMarquardtOptimizer opt(activeGraph, initial, params);
        gtsam::Values result = opt.optimize();
        initial = result;
        nBad = 0;
        double chi2close = 1.5 * chi2Mono[it];
        for (size_t i = 0; i < vpFactorsMono.size(); i++)
        {
            double err = 2.0 * vpFactorsMono[i]->error(initial);
            bool bClose = pFrame->mvpMapPoints[vnIndexMono[i]]->mTrackDepth < 10.f;
            pFrame->mvbOutlier[vnIndexMono[i]] = (err > chi2Mono[it] && !bClose) || (bClose && err > chi2close);
            if (pFrame->mvbOutlier[vnIndexMono[i]])
            {
                nBad++;
            }
        }
        for (size_t i = 0; i < vpFactorsStereo.size(); i++)
        {
            double err = 2.0 * vpFactorsStereo[i]->error(initial);
            pFrame->mvbOutlier[vnIndexStereo[i]] = (err > chi2Stereo[it]);
            if (pFrame->mvbOutlier[vnIndexStereo[i]])
            {
                nBad++;
            }
        }
    }

    if ((nInitialCorrespondences - nBad < 30) && !bRecInit)
    {
        nBad = 0;
        for (size_t i = 0; i < vpFactorsMono.size(); i++)
        {
            pFrame->mvbOutlier[vnIndexMono[i]] = (2.0 * vpFactorsMono[i]->error(initial) >= 18.0);
        }
        for (size_t i = 0; i < vpFactorsStereo.size(); i++)
        {
            pFrame->mvbOutlier[vnIndexStereo[i]] = (2.0 * vpFactorsStereo[i]->error(initial) >= 24.0);
        }
        for (size_t i = 0; i < vnIndexMono.size(); i++)
        {
            if (pFrame->mvbOutlier[vnIndexMono[i]])
            {
                nBad++;
            }
        }
        for (size_t i = 0; i < vnIndexStereo.size(); i++)
        {
            if (pFrame->mvbOutlier[vnIndexStereo[i]])
            {
                nBad++;
            }
        }
    }

    gtsam::Pose3 Twb = initial.at<gtsam::Pose3>(poseK0);
    gtsam::Vector3 v = initial.at<gtsam::Vector3>(velK1);
    gtsam::imuBias::ConstantBias bias = initial.at<gtsam::imuBias::ConstantBias>(biasK2);
    pFrame->SetImuPoseVelocity(gtsamToSophusPose(Twb).rotationMatrix().cast<float>(),
                               gtsamToSophusPose(Twb).translation().cast<float>(), v.cast<float>());
    pFrame->mImuBias = fromGTSAMBias(bias);

    Eigen::Matrix<double, 15, 15> H = Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::Vector3d twb(Twb.translation().x(), Twb.translation().y(), Twb.translation().z());
    pFrame->mpcpi = new ConstraintPoseImu(Twb.rotation().matrix(), twb, v, bias.gyroscope(), bias.accelerometer(), H);
    return nInitialCorrespondences - nBad;
}

int Optimizer::PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit)
{
    const gtsam::Key poseK0 = imuPoseKey(0);
    const gtsam::Key velK1 = velKey(1);
    const gtsam::Key biasK2 = biasKey(2);
    const gtsam::Key poseK4 = imuPoseKey(4);
    const gtsam::Key velK5 = velKey(5);
    const gtsam::Key biasK6 = biasKey(6);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    gtsam::Pose3 Twb_frame = sophusToGTSAMPose(pFrame->GetImuPose());
    initial.insert(poseK0, Twb_frame);
    initial.insert(velK1, Eigen::Vector3d(pFrame->GetVelocity().cast<double>()));
    initial.insert(biasK2, toGTSAMBias(pFrame->mImuBias));

    Frame* pFp = pFrame->mpPrevFrame;
    gtsam::Pose3 Twb_prev = sophusToGTSAMPose(pFp->GetImuPose());
    initial.insert(poseK4, Twb_prev);
    initial.insert(velK5, Eigen::Vector3d(pFp->GetVelocity().cast<double>()));
    initial.insert(biasK6, toGTSAMBias(pFp->mImuBias));

    if (pFp->mpcpi)
    {
        graph.add(boost::make_shared<PriorNavFactor>(poseK4, velK5, biasK6, *pFp->mpcpi));
    }

    pFrame->mpImuPreintegratedFrame->SetNewBias(pFp->mImuBias);
    graph.add(
        boost::make_shared<InertialFactor>(poseK4, velK5, biasK6, poseK0, velK1, pFrame->mpImuPreintegratedFrame));
    gtsam::Matrix6 Info6Bias = gtsam::Matrix6::Zero();
    Info6Bias.block<3, 3>(0, 0) = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
    Info6Bias.block<3, 3>(3, 3) = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
    graph.add(boost::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        biasK6, biasK2, gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Gaussian::Information(Info6Bias)));

    const int N = pFrame->N;
    const int Nleft = pFrame->Nleft;
    const bool bRight = (Nleft != -1);
    std::vector<boost::shared_ptr<MonoOnlyPoseFactor>> vpFactorsMono;
    std::vector<boost::shared_ptr<StereoOnlyPoseFactor>> vpFactorsStereo;
    std::vector<size_t> vnIndexMono, vnIndexStereo;
    int nInitialCorrespondences = 0;
    gtsam::Pose3 Tbc = sophusToGTSAMPose(pFrame->mImuCalib.mTbc);

    {
        std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);
        for (int i = 0; i < N; i++)
        {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if (!pMP)
            {
                continue;
            }
            Eigen::Vector3d Xw = pMP->GetWorldPos().cast<double>();
            if ((!bRight && pFrame->mvuRight[i] < 0) || (bRight && i < Nleft))
            {
                cv::KeyPoint kpUn = (bRight && i < Nleft) ? pFrame->mvKeys[i] : pFrame->mvKeysUn[i];
                Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                double invSigma2 =
                    static_cast<double>(pFrame->mvInvLevelSigma2[kpUn.octave] / pFrame->mpCamera->uncertainty2(obs));
                pFrame->mvbOutlier[i] = false;
                vpFactorsMono.push_back(boost::make_shared<MonoOnlyPoseFactor>(
                    poseK0, Xw, obs, makeHuberNoise(2, 5.991, invSigma2), pFrame->mpCamera, Tbc));
                graph.add(vpFactorsMono.back());
                vnIndexMono.push_back(static_cast<size_t>(i));
                nInitialCorrespondences++;
            }
            else if (!bRight)
            {
                const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
                Eigen::Vector3d obs(kpUn.pt.x, kpUn.pt.y, pFrame->mvuRight[i]);
                double invSigma2 = static_cast<double>(pFrame->mvInvLevelSigma2[kpUn.octave] /
                                                       pFrame->mpCamera->uncertainty2(Eigen::Vector2d(obs(0), obs(1))));
                pFrame->mvbOutlier[i] = false;
                vpFactorsStereo.push_back(boost::make_shared<StereoOnlyPoseFactor>(
                    poseK0, Xw, obs, pFrame->mbf, makeHuberNoise(3, 7.815, invSigma2), pFrame->mpCamera, Tbc));
                graph.add(vpFactorsStereo.back());
                vnIndexStereo.push_back(static_cast<size_t>(i));
                nInitialCorrespondences++;
            }
            if (bRight && i >= Nleft)
            {
                const cv::KeyPoint& kpUn = pFrame->mvKeysRight[i - Nleft];
                Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                double invSigma2 =
                    static_cast<double>(pFrame->mvInvLevelSigma2[kpUn.octave] / pFrame->mpCamera->uncertainty2(obs));
                pFrame->mvbOutlier[i] = false;
                vpFactorsMono.push_back(boost::make_shared<MonoOnlyPoseFactor>(
                    poseK0, Xw, obs, makeHuberNoise(2, 5.991, invSigma2), pFrame->mpCamera, Tbc));
                graph.add(vpFactorsMono.back());
                vnIndexMono.push_back(static_cast<size_t>(i));
                nInitialCorrespondences++;
            }
        }
    }

    const double chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const double chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};
    const int its[4] = {10, 10, 10, 10};
    int nBad = 0;

    const bool hasPrior = (pFp->mpcpi != nullptr);
    for (int it = 0; it < 4; it++)
    {
        gtsam::NonlinearFactorGraph activeGraph;
        if (hasPrior)
        {
            activeGraph.add(graph.at(0));
        }
        activeGraph.add(graph.at(hasPrior ? 1 : 0));
        activeGraph.add(graph.at(hasPrior ? 2 : 1));
        for (size_t i = 0; i < vpFactorsMono.size(); i++)
        {
            if (!pFrame->mvbOutlier[vnIndexMono[i]])
            {
                activeGraph.add(vpFactorsMono[i]);
            }
        }
        for (size_t i = 0; i < vpFactorsStereo.size(); i++)
        {
            if (!pFrame->mvbOutlier[vnIndexStereo[i]])
            {
                activeGraph.add(vpFactorsStereo[i]);
            }
        }
        if (activeGraph.size() < 10)
        {
            break;
        }
        gtsam::LevenbergMarquardtParams params;
        params.setMaxIterations(its[it]);
        params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
        gtsam::LevenbergMarquardtOptimizer opt(activeGraph, initial, params);
        gtsam::Values result = opt.optimize();
        initial = result;
        nBad = 0;
        double chi2close = 1.5 * chi2Mono[it];
        for (size_t i = 0; i < vpFactorsMono.size(); i++)
        {
            double err = 2.0 * vpFactorsMono[i]->error(initial);
            bool bClose = pFrame->mvpMapPoints[vnIndexMono[i]]->mTrackDepth < 10.f;
            pFrame->mvbOutlier[vnIndexMono[i]] = (err > chi2Mono[it] && !bClose) || (bClose && err > chi2close);
            if (pFrame->mvbOutlier[vnIndexMono[i]])
            {
                nBad++;
            }
        }
        for (size_t i = 0; i < vpFactorsStereo.size(); i++)
        {
            double err = 2.0 * vpFactorsStereo[i]->error(initial);
            pFrame->mvbOutlier[vnIndexStereo[i]] = (err > chi2Stereo[it]);
            if (pFrame->mvbOutlier[vnIndexStereo[i]])
            {
                nBad++;
            }
        }
    }

    if ((nInitialCorrespondences - nBad < 30) && !bRecInit)
    {
        nBad = 0;
        for (size_t i = 0; i < vpFactorsMono.size(); i++)
        {
            pFrame->mvbOutlier[vnIndexMono[i]] = (2.0 * vpFactorsMono[i]->error(initial) >= 18.0);
        }
        for (size_t i = 0; i < vpFactorsStereo.size(); i++)
        {
            pFrame->mvbOutlier[vnIndexStereo[i]] = (2.0 * vpFactorsStereo[i]->error(initial) >= 24.0);
        }
        for (size_t i = 0; i < vnIndexMono.size(); i++)
        {
            if (pFrame->mvbOutlier[vnIndexMono[i]])
            {
                nBad++;
            }
        }
        for (size_t i = 0; i < vnIndexStereo.size(); i++)
        {
            if (pFrame->mvbOutlier[vnIndexStereo[i]])
            {
                nBad++;
            }
        }
    }

    gtsam::Pose3 Twb = initial.at<gtsam::Pose3>(poseK0);
    gtsam::Vector3 v = initial.at<gtsam::Vector3>(velK1);
    gtsam::imuBias::ConstantBias bias = initial.at<gtsam::imuBias::ConstantBias>(biasK2);
    pFrame->SetImuPoseVelocity(gtsamToSophusPose(Twb).rotationMatrix().cast<float>(),
                               gtsamToSophusPose(Twb).translation().cast<float>(), v.cast<float>());
    pFrame->mImuBias = fromGTSAMBias(bias);

    Eigen::Matrix<double, 15, 15> H = Eigen::Matrix<double, 15, 15>::Identity();
    Eigen::Vector3d twb(Twb.translation().x(), Twb.translation().y(), Twb.translation().z());
    pFrame->mpcpi = new ConstraintPoseImu(Twb.rotation().matrix(), twb, v, bias.gyroscope(), bias.accelerometer(), H);
    delete pFp->mpcpi;
    pFp->mpcpi = nullptr;

    return nInitialCorrespondences - nBad;
}

void Optimizer::OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                           const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose& CorrectedSim3,
                                           const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections)
{
    const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();
    const unsigned int nMaxKFid = pMap->GetMaxKFid();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    std::vector<gtsam::Similarity3> vScw(nMaxKFid + 1);
    std::vector<gtsam::Similarity3> vCorrectedSwc(nMaxKFid + 1);
    const int minFeat = 100;
    gtsam::SharedNoiseModel dof6Noise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1.0, 1.0, 1.0, 1.0).finished());

    for (KeyFrame* pKF : vpKFs)
    {
        if (pKF->isBad())
        {
            continue;
        }
        const int nIDi = pKF->mnId;
        gtsam::Pose3 Tcw;
        auto it = CorrectedSim3.find(pKF);
        if (it != CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            gtsam::Similarity3 Swc = it->second.inverse();
            Tcw = gtsam::Pose3(Swc.rotation(), Swc.translation()).inverse();
        }
        else
        {
            Sophus::SE3d Tcw_s = pKF->GetPose().cast<double>();
            vScw[nIDi] =
                gtsam::Similarity3(gtsam::Rot3(Tcw_s.rotationMatrix()), gtsam::Point3(Tcw_s.translation()), 1.0);
            Tcw = gtsam::Pose3(gtsam::Rot3(Tcw_s.rotationMatrix()), gtsam::Point3(Tcw_s.translation()));
        }
        initial.insert(poseKey(nIDi), Tcw);
        if (pKF == pLoopKF)
        {
            graph.add(
                gtsam::PriorFactor<gtsam::Pose3>(poseKey(nIDi), Tcw, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6)));
        }
    }

    std::set<std::pair<long unsigned int, long unsigned int>> sInsertedEdges;

    auto add4DoFEdge = [&](int nIDi, int nIDj, const gtsam::Similarity3& Sij)
    {
        Eigen::Matrix3d dRij = Sij.rotation().matrix();
        Eigen::Vector3d dtij(Sij.translation().x(), Sij.translation().y(), Sij.translation().z());
        graph.add(boost::make_shared<FourDOFBetweenFactor>(poseKey(nIDi), poseKey(nIDj), dRij, dtij, dof6Noise));
    };

    for (const auto& mit : LoopConnections)
    {
        KeyFrame* pKF = mit.first;
        const long unsigned int nIDi = pKF->mnId;
        const gtsam::Similarity3 Siw = vScw[nIDi];
        for (KeyFrame* pKFn : mit.second)
        {
            const long unsigned int nIDj = pKFn->mnId;
            if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(pKFn) < minFeat)
            {
                continue;
            }
            gtsam::Similarity3 Sij = Siw.compose(vScw[nIDj].inverse());
            add4DoFEdge(nIDi, nIDj, Sij);
            sInsertedEdges.insert(std::make_pair(std::min(nIDi, nIDj), std::max(nIDi, nIDj)));
        }
    }

    for (KeyFrame* pKF : vpKFs)
    {
        if (pKF->isBad())
        {
            continue;
        }
        const int nIDi = pKF->mnId;
        gtsam::Similarity3 Siw =
            (NonCorrectedSim3.find(pKF) != NonCorrectedSim3.end()) ? NonCorrectedSim3.find(pKF)->second : vScw[nIDi];
        KeyFrame* pParentKF = pKF->GetParent();
        if (pParentKF)
        {
            int nIDj = pParentKF->mnId;
            gtsam::Similarity3 Swj = (NonCorrectedSim3.find(pParentKF) != NonCorrectedSim3.end())
                                         ? NonCorrectedSim3.find(pParentKF)->second.inverse()
                                         : vScw[nIDj].inverse();
            add4DoFEdge(nIDi, nIDj, Siw.compose(Swj));
        }
        KeyFrame* prevKF = pKF->mPrevKF;
        if (prevKF)
        {
            int nIDj = prevKF->mnId;
            gtsam::Similarity3 Swj = (NonCorrectedSim3.find(prevKF) != NonCorrectedSim3.end())
                                         ? NonCorrectedSim3.find(prevKF)->second.inverse()
                                         : vScw[nIDj].inverse();
            add4DoFEdge(nIDi, nIDj, Siw.compose(Swj));
        }
        const std::set<KeyFrame*>& sLoopEdges = pKF->GetLoopEdges();
        for (KeyFrame* pLKF : sLoopEdges)
        {
            if (pLKF->mnId >= pKF->mnId)
            {
                continue;
            }
            gtsam::Similarity3 Swl = (NonCorrectedSim3.find(pLKF) != NonCorrectedSim3.end())
                                         ? NonCorrectedSim3.find(pLKF)->second.inverse()
                                         : vScw[pLKF->mnId].inverse();
            add4DoFEdge(nIDi, pLKF->mnId, Siw.compose(Swl));
        }
        for (KeyFrame* pKFn : pKF->GetCovisiblesByWeight(minFeat))
        {
            if (!pKFn || pKFn == pParentKF || pKFn == prevKF || pKFn == pKF->mNextKF || pKF->hasChild(pKFn) ||
                sLoopEdges.count(pKFn))
            {
                continue;
            }
            if (pKFn->isBad() || pKFn->mnId >= pKF->mnId)
            {
                continue;
            }
            if (sInsertedEdges.count(std::make_pair(std::min(pKF->mnId, pKFn->mnId), std::max(pKF->mnId, pKFn->mnId))))
            {
                continue;
            }
            gtsam::Similarity3 Swn = (NonCorrectedSim3.find(pKFn) != NonCorrectedSim3.end())
                                         ? NonCorrectedSim3.find(pKFn)->second.inverse()
                                         : vScw[pKFn->mnId].inverse();
            add4DoFEdge(nIDi, pKFn->mnId, Siw.compose(Swn));
        }
    }

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(20);
    params.setVerbosity("SILENT");  // avoid convergence prints (relativeDecrease, newError, etc.)
    gtsam::LevenbergMarquardtOptimizer opt(graph, initial, params);
    gtsam::Values result = opt.optimize();

    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    for (KeyFrame* pKFi : vpKFs)
    {
        if (pKFi->isBad())
        {
            continue;
        }
        const int nIDi = pKFi->mnId;
        gtsam::Pose3 Tcw = result.at<gtsam::Pose3>(poseKey(nIDi));
        gtsam::Similarity3 CorrectedSiw(Tcw.rotation(), Tcw.translation(), 1.0);
        vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
        Eigen::Vector3d t = Eigen::Vector3d(Tcw.translation().x(), Tcw.translation().y(), Tcw.translation().z());
        Sophus::SE3d Tcw_s(Tcw.rotation().matrix(), t);
        pKFi->SetPose(Tcw_s.cast<float>());
    }
    for (MapPoint* pMP : vpMPs)
    {
        if (pMP->isBad())
        {
            continue;
        }
        int nIDr = pMP->GetReferenceKeyFrame()->mnId;
        const gtsam::Similarity3& Srw = vScw[nIDr];
        const gtsam::Similarity3& correctedSwr = vCorrectedSwc[nIDr];
        gtsam::Point3 eigP3Dw(pMP->GetWorldPos().cast<double>());
        gtsam::Point3 inCam = Srw.transformFrom(eigP3Dw);
        gtsam::Point3 eigCorrectedP3Dw = correctedSwr.transformFrom(inCam);
        pMP->SetWorldPos(Eigen::Vector3f(static_cast<float>(eigCorrectedP3Dw.x()),
                                         static_cast<float>(eigCorrectedP3Dw.y()),
                                         static_cast<float>(eigCorrectedP3Dw.z())));
        pMP->UpdateNormalAndDepth();
    }
    pMap->IncreaseChangeIndex();
}

}  // namespace ORB_SLAM3
