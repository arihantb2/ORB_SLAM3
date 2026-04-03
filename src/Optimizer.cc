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
#include <unordered_set>

namespace ORB_SLAM3
{

enum class InertialOptMode
{
    Full,             // optimize pose, vel, bias, gravity, scale
    BiasOnly,         // fix poses; optimize vel, bias
    GravityScaleOnly  // fix pose, vel, bias; optimize gravity, scale
};

bool sortByVal(const std::pair<MapPoint*, int>& a, const std::pair<MapPoint*, int>& b)
{
    return (a.second < b.second);
}

// Returns observations sorted by KeyFrame mnId so that graph edge construction order
// is deterministic regardless of pointer address (ASLR).
static std::vector<std::pair<KeyFrame*, std::tuple<int, int>>>
sortedObservations(const std::map<KeyFrame*, std::tuple<int, int>>& obs)
{
    std::vector<std::pair<KeyFrame*, std::tuple<int, int>>> v(obs.begin(), obs.end());
    std::sort(v.begin(), v.end(),
              [](const std::pair<KeyFrame*, std::tuple<int, int>>& a,
                 const std::pair<KeyFrame*, std::tuple<int, int>>& b) {
                  return a.first->mnId < b.first->mnId;
              });
    return v;
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

        for (const auto& [pKF, tup] : sortedObservations(observations))
        {
            if (pKF->isBad() || pKF->mnId > maxKFid)
            {
                continue;
            }
            const int leftIndex = std::get<0>(tup);
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
                // GTSAM StereoPoint2 is (uL, uR, v). We store measurements as (uL, v, uR), so pass uR=kp_ur as the 2nd argument.
                gtsam::StereoPoint2 obs(pKF->mvKeysUn[leftIndex].pt.x, kp_ur, pKF->mvKeysUn[leftIndex].pt.y);
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

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << pFrame->mnId << "] POSE_OPTIMIZATION: nInitialCorrespondences=" << nInitialCorrespondences
        << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << pFrame->mnId << "] POSE_OPTIMIZATION: nMatches=" << nInitialCorrespondences - nBad << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << "[" << pFrame->mnId << "] POSE_OPTIMIZATION: nOutliers=" << nBad << std::endl;

    gtsam::Pose3 Tcw_final = initial.at<gtsam::Pose3>(poseK);
    pFrame->SetPose(gtsamToSophusPose(Tcw_final));
    return nInitialCorrespondences - nBad;
}

namespace
{
// Local bundle adjustment prior toggles, configurable via Settings / YAML.
// Defaults preserve existing behavior.
bool use_pose_priors_lba = false;
bool use_scale_priors_lba = true;
bool use_odometry_priors_lba = false;
}  // namespace

void Optimizer::ConfigureLocalBundleAdjustmentPriors(bool use_pose_priors, bool use_scale_priors,
                                                     bool use_odometry_priors)
{
    use_pose_priors_lba = use_pose_priors;
    use_scale_priors_lba = use_scale_priors;
    use_odometry_priors_lba = use_odometry_priors;
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF,
                                      int& num_MPs, int& num_edges, std::vector<unsigned long>& fixed_kf_ids,
                                      std::vector<unsigned long>& optimised_kf_ids,
                                      std::vector<LBAMapPoint>& lba_map_points,
                                      std::vector<unsigned long>& outlier_mp_ids,
                                      std::vector<CovisibilityEdge>& covisibility_edges,
                                      std::vector<SpanningTreeEdge>& spanning_tree_edges)
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
        const std::map<KeyFrame*, std::tuple<int, int>> observations = (*lit)->GetObservations();
        for (const auto& [pKFi, tup] : sortedObservations(observations))
        {
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

    for (KeyFrame* pKFi : lLocalKeyFrames)
    {
        optimised_kf_ids.push_back(pKFi->mnId);
    }
    for (KeyFrame* pKFi : lFixedCameras)
    {
        fixed_kf_ids.push_back(pKFi->mnId);
    }
    // When the map-origin KF is inside lLocalKeyFrames it receives a tight
    // prior (sigma = 1e-9) that effectively anchors it.  Include its ID in
    // fixed_kf_ids so that num_fixed_kfs == fixed_kf_ids.size() always holds.
    // The ID also appears in optimised_kf_ids (it is a GTSAM variable).
    const unsigned long initKFid = pCurrentMap->GetInitKFid();
    for (KeyFrame* pKFi : lLocalKeyFrames)
    {
        if (pKFi->mnId == initKFid)
        {
            fixed_kf_ids.push_back(pKFi->mnId);
        }
    }
    // Covisibility edges between all KF pairs in the LBA window.
    // Optimised × optimised: iterate over unique pairs only (j > i).
    {
        const std::vector<KeyFrame*> vLocal(lLocalKeyFrames.begin(), lLocalKeyFrames.end());
        for (size_t i = 0; i < vLocal.size(); ++i)
        {
            for (size_t j = i + 1; j < vLocal.size(); ++j)
            {
                const int w = vLocal[i]->GetWeight(vLocal[j]);
                if (w > 0)
                {
                    covisibility_edges.push_back({vLocal[i]->mnId, vLocal[j]->mnId, w});
                }
            }
        }
    }
    // Optimised × fixed.
    for (KeyFrame* pKFlocal : lLocalKeyFrames)
    {
        for (KeyFrame* pKFfixed : lFixedCameras)
        {
            const int w = pKFlocal->GetWeight(pKFfixed);
            if (w > 0)
            {
                const unsigned long id_a = std::min(pKFlocal->mnId, pKFfixed->mnId);
                const unsigned long id_b = std::max(pKFlocal->mnId, pKFfixed->mnId);
                covisibility_edges.push_back({id_a, id_b, w});
            }
        }
    }

    // Spanning-tree edges: one entry per KF in the LBA window.
    // Build a set of window IDs for the parent_in_lba_window flag.
    {
        std::unordered_set<unsigned long> windowIds;
        for (unsigned long id : optimised_kf_ids)
        {
            windowIds.insert(id);
        }
        for (unsigned long id : fixed_kf_ids)
        {
            windowIds.insert(id);
        }
        auto collectSpanningEdges = [&](const std::list<KeyFrame*>& kfs)
        {
            for (KeyFrame* pKFi : kfs)
            {
                KeyFrame* pParent = pKFi->GetParent();
                const unsigned long parent_id = pParent ? pParent->mnId : 0;
                spanning_tree_edges.push_back(
                    {pKFi->mnId, parent_id, pParent != nullptr && windowIds.count(parent_id) > 0});
            }
        };
        collectSpanningEdges(lLocalKeyFrames);
        collectSpanningEdges(lFixedCameras);
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::KeyVector landmarkKeys;
    unsigned long maxKFid = 0;

    pCurrentMap->msOptKFs.clear();
    pCurrentMap->msFixedKFs.clear();

    // copy lLocalKeyFrames to a vector and sort it by mnId
    std::vector<KeyFrame*> vLocalKeyFrames(lLocalKeyFrames.begin(), lLocalKeyFrames.end());
    std::sort(vLocalKeyFrames.begin(), vLocalKeyFrames.end(),
              [](KeyFrame* a, KeyFrame* b) { return a->mnId > b->mnId; });

    for (KeyFrame* pKFi : lLocalKeyFrames)
    {
        gtsam::Pose3 Tcw = sophusToGTSAMPose(pKFi->GetPose());
        initial.insert(poseKey(static_cast<uint32_t>(pKFi->mnId)), Tcw);
        if (pKFi->mnId == pMap->GetInitKFid())
        {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(poseKey(static_cast<uint32_t>(pKFi->mnId)), Tcw,
                                                       gtsam::noiseModel::Isotropic::Sigma(6, 1e-9)));
        }
        if (pKFi->mnId > maxKFid)
        {
            maxKFid = pKFi->mnId;
        }
        pCurrentMap->msOptKFs.insert(pKFi->mnId);
    }

    num_OptKF = static_cast<int>(lLocalKeyFrames.size());

    // Between factors (odometry priors)
    if (use_odometry_priors_lba)
    {
        int num_between_factors = 0;
        // create a between factor between consecutive local keyframes
        for (auto it = vLocalKeyFrames.begin(); it != std::prev(vLocalKeyFrames.end()); ++it)
        {
            KeyFrame* pKFi = *it;
            KeyFrame* pKFiNext = *(std::next(it));

            if (!pKFi->hasPosePrior() || !pKFiNext->hasPosePrior())
            {
                continue;
            }

            // skip between factors between keyframes that are not sequential
            if (std::abs(static_cast<int>(pKFiNext->mnId - pKFi->mnId)) > 1)
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << pKF->mnId
                    << "] LOCAL_BUNDLE_ADJUSTMENT: skipping between factor between non-sequential keyframes "
                    << pKFi->mnId << "->" << pKFiNext->mnId << std::endl;
                continue;
            }

            const auto ext_T_ci = sophusToGTSAMPose(pKFi->mPosePrior.value());
            const auto ext_T_ciNext = sophusToGTSAMPose(pKFiNext->mPosePrior.value());
            const auto T_ci_ciNext = ext_T_ci.inverse() * ext_T_ciNext;

            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "[" << pKF->mnId << "] LOCAL_BUNDLE_ADJUSTMENT: adding between factor between keyframes "
                << pKFi->mnId << "->" << pKFiNext->mnId << " T_ci_ciNext=" << T_ci_ciNext.translation().transpose()
                << std::endl;

            const auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
            // add robust loss huber
            const auto robust_noise =
                gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.0), noise);
            const auto factor = boost::make_shared<BetweenFactorTcw>(poseKey(static_cast<uint32_t>(pKFi->mnId)),
                                                                     poseKey(static_cast<uint32_t>(pKFiNext->mnId)),
                                                                     T_ci_ciNext, robust_noise);
            graph.add(factor);
            num_between_factors++;
        }
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << pKF->mnId << "] LOCAL_BUNDLE_ADJUSTMENT: num_between_factors=" << num_between_factors
            << std::endl;
    }

    // Scale factors
    if (use_scale_priors_lba)
    {
        int num_scale_factors = 0;
        for (auto it = vLocalKeyFrames.begin(); it != std::prev(vLocalKeyFrames.end()); ++it)
        {
            KeyFrame* pKFi = *it;
            KeyFrame* pKFiNext = *(std::next(it));

            if (!pKFi->hasPosePrior() || !pKFiNext->hasPosePrior())
            {
                continue;
            }

            // skip scale factors between keyframes that are not sequential
            if (std::abs(static_cast<int>(pKFiNext->mnId - pKFi->mnId)) > 1)
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << pKF->mnId
                    << "] LOCAL_BUNDLE_ADJUSTMENT: skipping scale factor between non-sequential keyframes "
                    << pKFi->mnId << "->" << pKFiNext->mnId << std::endl;
                continue;
            }

            const auto ext_T_ci = sophusToGTSAMPose(pKFi->mPosePrior.value());
            const auto ext_T_ciNext = sophusToGTSAMPose(pKFiNext->mPosePrior.value());
            const auto T_ci_ciNext = ext_T_ci.inverse() * ext_T_ciNext;
            const auto translation_norm = T_ci_ciNext.translation().norm();

            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "[" << pKF->mnId << "] LOCAL_BUNDLE_ADJUSTMENT: adding scale factor between keyframes " << pKFi->mnId
                << "->" << pKFiNext->mnId << " T_ci_ciNext=" << T_ci_ciNext.translation().transpose()
                << " translation norm=" << translation_norm << std::endl;

            const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1e-3);
            // add robust loss huber
            const auto robust_noise =
                gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.0), noise);
            const auto factor = boost::make_shared<ScaleFactorTcw>(poseKey(static_cast<uint32_t>(pKFi->mnId)),
                                                                   poseKey(static_cast<uint32_t>(pKFiNext->mnId)),
                                                                   translation_norm, robust_noise);
            graph.add(factor);
            num_scale_factors++;
        }
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << pKF->mnId << "] LOCAL_BUNDLE_ADJUSTMENT: num_scale_factors=" << num_scale_factors << std::endl;
    }

    // Prior factors (pose priors)
    if (use_pose_priors_lba)
    {
        // Add prior factors for the local keyframes using the pose priors
        auto initKF = pCurrentMap->GetOriginKF();
        int num_pose_prior_factors = 0;

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << pKF->mnId << "] LOCAL_BUNDLE_ADJUSTMENT: initKF=" << initKF->mnId << std::endl;

        if (initKF && initKF->hasPosePrior())
        {
            const auto ext_T_w = sophusToGTSAMPose(initKF->mPosePrior.value());
            const auto c0_T_w = sophusToGTSAMPose(initKF->GetPose());

            for (KeyFrame* pKFi : vLocalKeyFrames)
            {
                if (!pKFi->hasPosePrior())
                {
                    continue;
                }

                if (pKFi->mnId == initKF->mnId)
                {
                    continue;
                }

                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << pKF->mnId << "] LOCAL_BUNDLE_ADJUSTMENT: pKF" << pKFi->mnId
                    << "->mPosePrior=" << pKFi->mPosePrior.value().matrix().block<3, 1>(0, 3).transpose() << std::endl;

                // Compute the pose prior, note that we have the keyframe priors in external frame, we need them in the visual map coordinate frame
                const auto& ciPrior_T_ext = sophusToGTSAMPose(pKFi->mPosePrior.value().inverse());
                const auto ciPrior_T_w = ciPrior_T_ext * ext_T_w;  // measurement
                const auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
                // add robust loss huber
                const auto robust_noise =
                    gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.0), noise);
                const auto factor = boost::make_shared<PriorFactorTcw>(poseKey(static_cast<uint32_t>(pKFi->mnId)),
                                                                       ciPrior_T_w.inverse(), robust_noise);

                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << pKF->mnId << "] w_T_c" << pKFi->mnId
                    << "Prior: " << ciPrior_T_w.inverse().translation().transpose() << std::endl;

                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << pKF->mnId << "] w_T_c" << pKFi->mnId << ": "
                    << pKFi->GetPose().inverse().translation().transpose() << std::endl;

                const auto error_vector = factor->evaluateError(sophusToGTSAMPose(pKFi->GetPose()));
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << pKF->mnId << "] " << pKFi->mnId << "Error: " << error_vector.transpose() << std::endl;

                graph.add(factor);
                num_pose_prior_factors++;
            }
        }

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << pKF->mnId << "] LOCAL_BUNDLE_ADJUSTMENT: num_pose_prior_factors=" << num_pose_prior_factors
            << std::endl;
    }

    for (KeyFrame* pKFi : lFixedCameras)
    {
        gtsam::Pose3 Tcw = sophusToGTSAMPose(pKFi->GetPose());
        initial.insert(poseKey(static_cast<uint32_t>(pKFi->mnId)), Tcw);
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(poseKey(static_cast<uint32_t>(pKFi->mnId)), Tcw,
                                                   gtsam::noiseModel::Isotropic::Sigma(6, 1e-9)));
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

        for (const auto& [pKFi, tup] : sortedObservations(pMP->GetObservations()))
        {
            if (pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
            {
                continue;
            }
            const int leftIndex = std::get<0>(tup);
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
                graph.add(boost::make_shared<PinholeMonoTcwFactor>(poseK, pk, obs, noise, pKFi->mpCamera));

                nEdges++;
                monoEdges.push_back(std::make_tuple(pKFi, pMP, leftIndex));
            }
            else
            {
                const float kp_ur = pKFi->mvuRight[leftIndex];
                // GTSAM StereoPoint2 is (uL, uR, v). We store measurements as (uL, v, uR), so pass uR=kp_ur as the 2nd argument.
                gtsam::StereoPoint2 obs(pKFi->mvKeysUn[leftIndex].pt.x, kp_ur, pKFi->mvKeysUn[leftIndex].pt.y);
                gtsam::SharedNoiseModel noise = makeHuberNoise(3, 7.815, invSigma2);
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
    {
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
    }  // map lock released before calling EraseObservation / SetBadFlag

    lba_map_points.clear();
    lba_map_points.reserve(lLocalMapPoints.size());
    for (MapPoint* pMP : lLocalMapPoints)
    {
        if (!pMP || pMP->isBad())
        {
            continue;
        }
        lba_map_points.push_back({pMP->mnId, pMP->GetWorldPos()});
    }

    // Post-optimisation outlier rejection: check reprojection error using the
    // updated poses and 3-D positions, then erase high-error observations.
    // Done outside the map lock because EraseObservation / SetBadFlag acquire
    // their own per-object locks and may modify the map internally.
    for (const auto& [pKFi, pMP, leftIndex] : monoEdges)
    {
        if (pMP->isBad())
        {
            continue;
        }
        const Eigen::Vector3f x3Dc = pKFi->GetPose() * pMP->GetWorldPos();
        if (x3Dc(2) <= 0.0f)
        {
            const unsigned long mp_id = pMP->mnId;
            pMP->EraseObservation(pKFi);
            if (pMP->isBad())
            {
                outlier_mp_ids.push_back(mp_id);
            }
            continue;
        }
        const cv::KeyPoint& kp = pKFi->mvKeysUn[leftIndex];
        const float sigma2 = pKFi->mvLevelSigma2[kp.octave];
        const cv::Point2f uv = pKFi->mpCamera->project(cv::Point3f(x3Dc(0), x3Dc(1), x3Dc(2)));
        const float errX = uv.x - kp.pt.x;
        const float errY = uv.y - kp.pt.y;
        if ((errX * errX + errY * errY) > 5.991f * sigma2)
        {
            const unsigned long mp_id = pMP->mnId;
            pMP->EraseObservation(pKFi);
            if (pMP->isBad())
            {
                outlier_mp_ids.push_back(mp_id);
            }
        }
    }

    for (const auto& [pKFi, pMP, leftIndex] : stereoEdges)
    {
        if (pMP->isBad())
        {
            continue;
        }
        const Eigen::Vector3f x3Dc = pKFi->GetPose() * pMP->GetWorldPos();
        if (x3Dc(2) <= 0.0f)
        {
            const unsigned long mp_id = pMP->mnId;
            pMP->EraseObservation(pKFi);
            if (pMP->isBad())
            {
                outlier_mp_ids.push_back(mp_id);
            }
            continue;
        }
        const float invz = 1.0f / x3Dc(2);
        const float u = pKFi->fx * x3Dc(0) * invz + pKFi->cx;
        const float v = pKFi->fy * x3Dc(1) * invz + pKFi->cy;
        const float u_r = u - pKFi->mbf * invz;
        const cv::KeyPoint& kp = pKFi->mvKeysUn[leftIndex];
        const float kp_ur = pKFi->mvuRight[leftIndex];
        const float sigma2 = pKFi->mvLevelSigma2[kp.octave];
        const float errX = u - kp.pt.x;
        const float errY = v - kp.pt.y;
        const float errXr = u_r - kp_ur;
        if ((errX * errX + errY * errY + errXr * errXr) > 7.815f * sigma2)
        {
            const unsigned long mp_id = pMP->mnId;
            pMP->EraseObservation(pKFi);
            if (pMP->isBad())
            {
                outlier_mp_ids.push_back(mp_id);
            }
        }
    }
}

}  // namespace ORB_SLAM3
