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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"

#include <gtsam/geometry/Similarity3.h>

#include <math.h>
#include <set>
#include <vector>

namespace ORB_SLAM3
{

class LoopClosing;

class Optimizer
{
public:
    // Configuration for LocalBundleAdjustment priors (set from Settings / config file).
    // Pose priors use external pose priors on keyframes.
    // Scale priors constrain inter-keyframe scale using odometry priors.
    // Odometry priors add between-pose factors between consecutive keyframes.
    static void ConfigureLocalBundleAdjustmentPriors(bool use_pose_priors, bool use_scale_priors,
                                                     bool use_odometry_priors);

    void static BundleAdjustment(const std::vector<KeyFrame*>& vpKF, const std::vector<MapPoint*>& vpMP,
                                 int nIterations = 5, bool* pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations = 5, bool* pbStopFlag = NULL,
                                       const unsigned long nLoopKF = 0, const bool bRobust = true);
    void static FullInertialBA(Map* pMap, int its, const bool bFixLocal = false, const unsigned long nLoopKF = 0,
                               bool* pbStopFlag = NULL, bool bInit = false, float priorG = 1e2, float priorA = 1e6,
                               Eigen::VectorXd* vSingVal = NULL, bool* bHess = NULL);

    void static LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF,
                                      int& num_MPs, int& num_edges);

    int static PoseOptimization(Frame* pFrame);
    int static PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
    int static PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit = false);

    // if bFixScale is true, 6DoF optimization (stereo), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose& CorrectedSim3,
                                       const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections,
                                       const bool& bFixScale);
    void static OptimizeEssentialGraph(KeyFrame* pCurKF, std::vector<KeyFrame*>& vpFixedKFs,
                                       std::vector<KeyFrame*>& vpFixedCorrectedKFs,
                                       std::vector<KeyFrame*>& vpNonFixedKFs,
                                       std::vector<MapPoint*>& vpNonCorrectedMPs);

    // For inertial loopclosing
    void static OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                           const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose& CorrectedSim3,
                                           const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections);

    // if bFixScale is true, optimize SE3 (stereo), Sim3 otherwise (mono) (NEW)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches1,
                            gtsam::Similarity3& g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double, 7, 7>& mAcumHessian, const bool bAllPoints = false);

    // For inertial systems

    void static LocalInertialBA(KeyFrame* pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF,
                                int& num_MPs, int& num_edges, bool bLarge = false, bool bRecInit = false);
    void static MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool* pbStopFlag, Map* pMap,
                                LoopClosing::KeyFrameAndPose& corrPoses);

    // Local BA in welding area when two maps are merged
    void static LocalBundleAdjustment(KeyFrame* pMainKF, std::vector<KeyFrame*> vpAdjustKF,
                                      std::vector<KeyFrame*> vpFixedKF, bool* pbStopFlag);

    // Marginalize block element (start:end,start:end). Perform Schur complement.
    // Marginalized elements are filled with zeros.
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd& H, const int& start, const int& end);

    // Inertial pose-graph
    void static InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale, Eigen::Vector3d& bg,
                                     Eigen::Vector3d& ba, bool bMono, Eigen::MatrixXd& covInertial,
                                     bool bFixedVel = false, bool bGauss = false, float priorG = 1e2,
                                     float priorA = 1e6);
    void static InertialOptimization(Map* pMap, Eigen::Vector3d& bg, Eigen::Vector3d& ba, float priorG = 1e2,
                                     float priorA = 1e6);
    void static InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  //namespace ORB_SLAM3

#endif  // OPTIMIZER_H
