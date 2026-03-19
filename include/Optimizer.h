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
#include "Map.h"
#include "MapPoint.h"

#include <math.h>
#include <set>
#include <vector>

namespace ORB_SLAM3
{

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

    void static LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF,
                                      int& num_MPs, int& num_edges);

    int static PoseOptimization(Frame* pFrame);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  //namespace ORB_SLAM3

#endif  // OPTIMIZER_H
