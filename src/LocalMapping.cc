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

#include "LocalMapping.h"

#include "Atlas.h"
#include "FeatureMatcher.h"
#include "GeometricTools.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Optimizer.h"
#include "Settings.h"
#include "Tracking.h"

#include <chrono>
#include <list>
#include <mutex>
#include <set>
#include <string>
#include <tuple>

namespace ORB_SLAM3
{

LocalMapping::LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, Settings* settings)
    : mpSystem(pSys),
      mbMonocular(bMonocular),
      mbResetRequested(false),
      mbResetRequestedActiveMap(false),
      mbFinishRequested(false),
      mbFinished(true),
      mpAtlas(pAtlas),
      bInitializing(false),
      mbAbortBA(false),
      mbStopped(false),
      mbStopRequested(false),
      mbNotStop(false),
      mbAcceptKeyFrames(true)
{
    if (!settings)
    {
        throw std::runtime_error("LocalMapping requires non-null Settings (File.version \"1.0\" config).");
    }
    loadFromSettings(settings);

    mnMatchesInliers = 0;
}

void LocalMapping::loadFromSettings(Settings* settings)
{
    mThFarPoints = settings->thFarPoints();
    mbFarPoints = (mThFarPoints != 0);
    if (mbFarPoints)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "Discard points further than " << mThFarPoints << " m from current camera" << std::endl;
    }

    mOptimizeEveryTSeconds = settings->localMappingOptimizeEveryTSeconds();
    mMinKeyframesForLBA = settings->localMappingMinKeyframesForLBA();
    mMPCullingMinObsMono = settings->localMappingMPCullingMinObsMono();
    mMPCullingMinObsStereo = settings->localMappingMPCullingMinObsStereo();
    mMPCullingMinKFAgeForObsCheck = settings->localMappingMPCullingMinKFAgeForObsCheck();
    mMPCullingMaxKFAgeInRecent = settings->localMappingMPCullingMaxKFAgeInRecent();
    mMPCullingMinFoundRatio = settings->localMappingMPCullingMinFoundRatio();
    mCreateNewMapPointsCovisibilityMono = settings->localMappingCreateNewMapPointsCovisibilityMono();
    mCreateNewMapPointsCovisibilityStereo = settings->localMappingCreateNewMapPointsCovisibilityStereo();
    mCreateNewMapPointsMatchRatio = settings->localMappingCreateNewMapPointsMatchRatio();
    mCreateNewMapPointsMinBaselineDepthRatio = settings->localMappingCreateNewMapPointsMinBaselineDepthRatio();
    mCreateNewMapPointsMaxCosParallax = settings->localMappingCreateNewMapPointsMaxCosParallax();
    mCreateNewMapPointsScaleConsistencyFactor = settings->localMappingCreateNewMapPointsScaleConsistencyFactor();
    mSearchInNeighborsNumNeighborKFs = settings->localMappingSearchInNeighborsNumNeighborKFs();
    mSearchInNeighborsNumSecondNeighbors = settings->localMappingSearchInNeighborsNumSecondNeighbors();
    mSearchInNeighborsMaxTemporalNeighbors = settings->localMappingSearchInNeighborsMaxTemporalNeighbors();
    mKeyFrameCullingRedundantRatio = settings->localMappingKeyFrameCullingRedundantRatio();
    mKeyFrameCullingMinObsInOthers = settings->localMappingKeyFrameCullingMinObsInOthers();
    mKeyFrameCullingMaxKeyframesToCheck = settings->localMappingKeyFrameCullingMaxKeyframesToCheck();
    mKeyFrameCullingEarlyExitAfterAbort = settings->localMappingKeyFrameCullingEarlyExitAfterAbort();
}

void LocalMapping::SetTracker(Tracking* pTracker)
{
    mpTracker = pTracker;
}

void LocalMapping::Run()
{
    mbFinished = false;

    while (!RunLoop())
    {
        usleep(3000);
    }

    SetFinish();
}

bool LocalMapping::RunLoop()
{
    // Tracking will see that Local Mapping is busy
    SetAcceptKeyFrames(false);

    // Check if there are keyframes in the queue
    if (CheckNewKeyFrames())
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "----------------------------------------------------------------------------------------------------";
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "[-:-] LOCAL_MAPPING_LOOP";
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "----------------------------------------------------------------------------------------------------";
        const auto start_time = std::chrono::steady_clock::now();

        // BoW conversion and insertion in Map
        ProcessNewKeyFrame();

        // Check recent MapPoints
        MapPointCulling();

        // Triangulate new MapPoints
        CreateNewMapPoints();

        mbAbortBA = false;

        if (!CheckNewKeyFrames())
        {
            // Find more matches in neighbor keyframes and fuse point duplications
            SearchInNeighbors();
        }

        constexpr double LBA_TIME_EPSILON = 0.1;  // 100ms
        bool b_doLBA = true;
        if (prevOptimizedKFTimestamp > 0.0)
        {
            const auto time_since_last_optimize = mpCurrentKeyFrame->mTimeStamp - prevOptimizedKFTimestamp;
            if (time_since_last_optimize < mOptimizeEveryTSeconds - LBA_TIME_EPSILON)
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "[" << mpCurrentKeyFrame->mnFrameId << ":" << mpCurrentKeyFrame->mnId
                    << "] Skipping LBA because it's too soon (time_since_last_optimize=" << time_since_last_optimize
                    << " s < OptimizeEveryTSeconds=" << mOptimizeEveryTSeconds << " s)." << std::endl;
                b_doLBA = false;
            }
        }

        bool b_doneLBA = false;
        int num_FixedKF_BA = 0;
        int num_OptKF_BA = 0;
        int num_MPs_BA = 0;
        int num_edges_BA = 0;

        if (!CheckNewKeyFrames() && !stopRequested() && b_doLBA)
        {
            if (mpAtlas->KeyFramesInMap() > mMinKeyframesForLBA)
            {

                {
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),
                                                     num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
                    Verbose::Print(Verbose::VERBOSITY_DEBUG)
                        << "[" << mpCurrentKeyFrame->mnFrameId << ":" << mpCurrentKeyFrame->mnId
                        << "] LBA performed with " << num_FixedKF_BA << " fixed KFs, " << num_OptKF_BA
                        << " optimized KFs, " << num_MPs_BA << " MapPoints, and " << num_edges_BA << " edges."
                        << std::endl;
                    b_doneLBA = true;
                }
                prevOptimizedKFTimestamp = mpCurrentKeyFrame->mTimeStamp;
            }

            // Check redundant local Keyframes
            KeyFrameCulling();
        }

        const auto end_time = std::chrono::steady_clock::now();
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "[" << mpCurrentKeyFrame->mnFrameId << ":" << mpCurrentKeyFrame->mnId
            << "] LOCAL_MAPPING_LOOP: duration=" << duration << " ms" << std::endl;

        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "----------------------------------------------------------------------------------------------------";
    }
    else if (Stop())
    {
        // Safe area to stop
        while (isStopped() && !CheckFinish())
        {
            usleep(3000);
        }

        return CheckFinish();
    }

    ResetIfRequested();

    // Tracking will see that Local Mapping is busy
    SetAcceptKeyFrames(true);

    return CheckFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame* pKF)
{
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA = true;
}

bool LocalMapping::CheckNewKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    return (!mlNewKeyFrames.empty());
}

void LocalMapping::SetNewKeyFrame()
{
    int pending_KFs_count = 0;
    {
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
        pending_KFs_count = mlNewKeyFrames.size();
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "[" << mpCurrentKeyFrame->mnFrameId << ":" << mpCurrentKeyFrame->mnId
                                             << "] SET_NEW_KEYFRAME: pending KFs=" << pending_KFs_count << std::endl;
}

void LocalMapping::ProcessNewKeyFrame()
{
    SetNewKeyFrame();

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const std::vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for (size_t i = 0; i < vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if (pMP)
        {
            if (!pMP->isBad())
            {
                if (!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else  // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::EmptyQueue()
{
    while (CheckNewKeyFrames())
    {
        ProcessNewKeyFrame();
    }
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    std::list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    const int cnThObs = mbMonocular ? mMPCullingMinObsMono : mMPCullingMinObsStereo;

    int borrar = mlpRecentAddedMapPoints.size();

    while (lit != mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;

        if (pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
            continue;
        }

        const bool tooFewObservations = ((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= mMPCullingMinKFAgeForObsCheck &&
                                        pMP->Observations() <= cnThObs;
        const bool tooOld = ((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= mMPCullingMaxKFAgeInRecent;
        const bool lowFoundRatio = pMP->GetFoundRatio() < mMPCullingMinFoundRatio;
        const bool shouldErase = lowFoundRatio || tooFewObservations || tooOld;
        const bool shouldSetBad = lowFoundRatio || tooFewObservations;

        if (shouldErase)
        {
            if (shouldSetBad)
            {
                pMP->SetBadFlag();
            }
            lit = mlpRecentAddedMapPoints.erase(lit);
            continue;
        }

        lit++;
        borrar--;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    const int nn = mbMonocular ? mCreateNewMapPointsCovisibilityMono : mCreateNewMapPointsCovisibilityStereo;
    std::vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    const DescriptorType descriptorType = (mpCurrentKeyFrame && mpCurrentKeyFrame->mDescriptors.type() == CV_32FC1)
                                              ? DescriptorType::FLOAT32
                                              : DescriptorType::BINARY;
    FeatureMatcher matcher(mCreateNewMapPointsMatchRatio, false, descriptorType);

    Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
    Eigen::Matrix<float, 3, 4> eigTcw1 = sophTcw1.matrix3x4();
    Eigen::Matrix<float, 3, 3> Rcw1 = eigTcw1.block<3, 3>(0, 0);
    Eigen::Matrix<float, 3, 3> Rwc1 = Rcw1.transpose();
    Eigen::Vector3f tcw1 = sophTcw1.translation();
    Eigen::Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float& fx1 = mpCurrentKeyFrame->fx;
    const float& fy1 = mpCurrentKeyFrame->fy;
    const float& cx1 = mpCurrentKeyFrame->cx;
    const float& cy1 = mpCurrentKeyFrame->cy;
    const float& invfx1 = mpCurrentKeyFrame->invfx;
    const float& invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = mCreateNewMapPointsScaleConsistencyFactor * mpCurrentKeyFrame->mfScaleFactor;
    int countStereo = 0;
    int countStereoGoodProj = 0;
    int countStereoAttempt = 0;
    int totalStereoPts = 0;
    // Search matches with epipolar restriction and triangulate
    for (size_t i = 0; i < vpNeighKFs.size(); i++)
    {
        if (i > 0 && CheckNewKeyFrames())
        {
            return;
        }
        KeyFrame* pKF2 = vpNeighKFs[i];

        GeometricCamera *pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

        // Check first that baseline is not too short
        Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
        Eigen::Vector3f vBaseline = Ow2 - Ow1;
        const float baseline = vBaseline.norm();

        if (!mbMonocular)
        {
            if (baseline < pKF2->mb)
            {
                continue;
            }
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline / medianDepthKF2;

            if (ratioBaselineDepth < mCreateNewMapPointsMinBaselineDepthRatio)
            {
                continue;
            }
        }

        // Search matches that fullfil epipolar constraint
        std::vector<std::pair<size_t, size_t>> vMatchedIndices;
        bool bCoarse = false;

        matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, vMatchedIndices, false, bCoarse);

        Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
        Eigen::Matrix<float, 3, 4> eigTcw2 = sophTcw2.matrix3x4();
        Eigen::Matrix<float, 3, 3> Rcw2 = eigTcw2.block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 3> Rwc2 = Rcw2.transpose();
        Eigen::Vector3f tcw2 = sophTcw2.translation();

        const float& fx2 = pKF2->fx;
        const float& fy2 = pKF2->fy;
        const float& cx2 = pKF2->cx;
        const float& cy2 = pKF2->cy;
        const float& invfx2 = pKF2->invfx;
        const float& invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for (int ikp = 0; ikp < nmatches; ikp++)
        {
            const int& idx1 = vMatchedIndices[ikp].first;
            const int& idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint& kp1 = (mpCurrentKeyFrame->NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]
                                      : (idx1 < mpCurrentKeyFrame->NLeft)
                                          ? mpCurrentKeyFrame->mvKeys[idx1]
                                          : mpCurrentKeyFrame->mvKeysRight[idx1 - mpCurrentKeyFrame->NLeft];
            const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = (kp1_ur >= 0);

            const cv::KeyPoint& kp2 = (pKF2->NLeft == -1)    ? pKF2->mvKeysUn[idx2]
                                      : (idx2 < pKF2->NLeft) ? pKF2->mvKeys[idx2]
                                                             : pKF2->mvKeysRight[idx2 - pKF2->NLeft];

            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = (kp2_ur >= 0);

            // Check parallax between rays
            Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
            Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

            Eigen::Vector3f ray1 = Rwc1 * xn1;
            Eigen::Vector3f ray2 = Rwc2 * xn2;
            const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

            float cosParallaxStereo = cosParallaxRays + 1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if (bStereo1)
            {
                cosParallaxStereo1 = cos(2 * atan2(mpCurrentKeyFrame->mb / 2, mpCurrentKeyFrame->mvDepth[idx1]));
            }
            else if (bStereo2)
            {
                cosParallaxStereo2 = cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));
            }
            if (bStereo1 || bStereo2)
            {
                totalStereoPts++;
            }
            cosParallaxStereo = std::min(cosParallaxStereo1, cosParallaxStereo2);

            Eigen::Vector3f x3D;

            bool goodProj = false;
            bool bPointStereo = false;
            if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
                (bStereo1 || bStereo2 || cosParallaxRays < mCreateNewMapPointsMaxCosParallax))
            {
                goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
                if (!goodProj)
                {
                    continue;
                }
            }
            else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)
            {
                countStereoAttempt++;
                bPointStereo = true;
                goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
            }
            else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
            {
                countStereoAttempt++;
                bPointStereo = true;
                goodProj = pKF2->UnprojectStereo(idx2, x3D);
            }
            else
            {
                continue;  //No stereo and very low parallax
            }

            if (goodProj && bPointStereo)
            {
                countStereoGoodProj++;
            }
            if (!goodProj)
            {
                continue;
            }
            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
            if (z1 <= 0)
            {
                continue;
            }
            float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
            if (z2 <= 0)
            {
                continue;
            }
            //Check reprojection error in first keyframe
            const float& sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3D) + tcw1(0);
            const float y1 = Rcw1.row(1).dot(x3D) + tcw1(1);
            const float invz1 = 1.0 / z1;

            if (!bStereo1)
            {
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1, y1, z1));
                float errX1 = uv1.x - kp1.pt.x;
                float errY1 = uv1.y - kp1.pt.y;

                if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                {
                    continue;
                }
            }
            else
            {
                float u1 = fx1 * x1 * invz1 + cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf * invz1;
                float v1 = fy1 * y1 * invz1 + cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8 * sigmaSquare1)
                {
                    continue;
                }
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3D) + tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3D) + tcw2(1);
            const float invz2 = 1.0 / z2;
            if (!bStereo2)
            {
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2, y2, z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                {
                    continue;
                }
            }
            else
            {
                float u2 = fx2 * x2 * invz2 + cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf * invz2;
                float v2 = fy2 * y2 * invz2 + cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8 * sigmaSquare2)
                {
                    continue;
                }
            }

            //Check scale consistency
            Eigen::Vector3f normal1 = x3D - Ow1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = x3D - Ow2;
            float dist2 = normal2.norm();

            if (dist1 == 0 || dist2 == 0)
            {
                continue;
            }
            if (mbFarPoints && (dist1 >= mThFarPoints || dist2 >= mThFarPoints))  // MODIFICATION
            {
                continue;
            }
            const float ratioDist = dist2 / dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];

            if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
            {
                continue;
            }
            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
            if (bPointStereo)
            {
                countStereo++;
            }
            pMP->AddObservation(mpCurrentKeyFrame, idx1);
            pMP->AddObservation(pKF2, idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
            pKF2->AddMapPoint(pMP, idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }

    Verbose::Print(Verbose::VERBOSITY_QUIET) << "[" << mpCurrentKeyFrame->mnFrameId << "] Added "
                                             << mlpRecentAddedMapPoints.size() << " map points" << std::endl;
}

void LocalMapping::SearchInNeighbors()
{
    const std::vector<KeyFrame*> vpNeighKFs =
        mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(mSearchInNeighborsNumNeighborKFs);
    std::vector<KeyFrame*> vpTargetKFs;
    for (std::vector<KeyFrame*>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
        {
            continue;
        }
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for (int i = 0, imax = vpTargetKFs.size(); i < imax; i++)
    {
        const std::vector<KeyFrame*> vpSecondNeighKFs =
            vpTargetKFs[i]->GetBestCovisibilityKeyFrames(mSearchInNeighborsNumSecondNeighbors);
        for (std::vector<KeyFrame*>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end();
             vit2 != vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if (pKFi2->isBad() || pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId ||
                pKFi2->mnId == mpCurrentKeyFrame->mnId)
            {
                continue;
            }
            vpTargetKFs.push_back(pKFi2);
            pKFi2->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
        }
        if (mbAbortBA)
        {
            break;
        }
    }

    // Search matches by projection from current KF in target KFs
    const DescriptorType descriptorType = (mpCurrentKeyFrame && mpCurrentKeyFrame->mDescriptors.type() == CV_32FC1)
                                              ? DescriptorType::FLOAT32
                                              : DescriptorType::BINARY;
    FeatureMatcher matcher(0.6f, true, descriptorType);
    std::vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for (std::vector<KeyFrame*>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi, vpMapPointMatches);
        if (pKFi->NLeft != -1)
        {
            matcher.Fuse(pKFi, vpMapPointMatches, true);
        }
    }

    if (mbAbortBA)
    {
        return;
    }
    // Search matches by projection from target KFs in current KF
    std::vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

    for (std::vector<KeyFrame*>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end(); vitKF != vendKF;
         vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        std::vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for (std::vector<MapPoint*>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end();
             vitMP != vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if (!pMP)
            {
                continue;
            }
            if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
            {
                continue;
            }
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates);
    if (mpCurrentKeyFrame->NLeft != -1)
    {
        matcher.Fuse(mpCurrentKeyFrame, vpFuseCandidates, true);
    }
    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if (pMP)
        {
            if (!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

void LocalMapping::RequestStop()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    mbStopRequested = true;
    std::unique_lock<std::mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    if (mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    std::unique_lock<std::mutex> lock2(mMutexFinish);
    if (mbFinished)
    {
        return;
    }
    mbStopped = false;
    mbStopRequested = false;
    for (std::list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
    {
        delete *lit;
    }
    mlNewKeyFrames.clear();
}

bool LocalMapping::AcceptKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    std::unique_lock<std::mutex> lock(mMutexAccept);
    mbAcceptKeyFrames = flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    std::unique_lock<std::mutex> lock(mMutexStop);

    if (flag && mbStopped)
    {
        return false;
    }
    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    const int Nd = 21;
    mpCurrentKeyFrame->UpdateBestCovisibles();
    std::vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    const float redundant_th = mKeyFrameCullingRedundantRatio;
    int count = 0;

    for (std::vector<KeyFrame*>::iterator vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end(); vit != vend;
         vit++)
    {
        count++;
        KeyFrame* pKF = *vit;

        if ((pKF->mnId == pKF->GetMap()->GetInitKFid()) || pKF->isBad())
        {
            continue;
        }
        const std::vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        const int thObs = mKeyFrameCullingMinObsInOthers;
        int nRedundantObservations = 0;
        int nMPs = 0;
        for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    if (!mbMonocular)
                    {
                        if (pKF->mvDepth[i] > pKF->mThDepth || pKF->mvDepth[i] < 0)
                        {
                            continue;
                        }
                    }

                    nMPs++;
                    if (pMP->Observations() > thObs)
                    {
                        const int& scaleLevel = (pKF->NLeft == -1) ? pKF->mvKeysUn[i].octave
                                                : (i < pKF->NLeft) ? pKF->mvKeys[i].octave
                                                                   : pKF->mvKeysRight[i].octave;
                        const std::map<KeyFrame*, std::tuple<int, int>> observations = pMP->GetObservations();
                        int nObs = 0;
                        for (std::map<KeyFrame*, std::tuple<int, int>>::const_iterator mit = observations.begin(),
                                                                                       mend = observations.end();
                             mit != mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if (pKFi == pKF)
                            {
                                continue;
                            }
                            std::tuple<int, int> indexes = mit->second;
                            int leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);
                            int scaleLeveli = -1;
                            if (pKFi->NLeft == -1)
                            {
                                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                            }
                            else
                            {
                                if (leftIndex != -1)
                                {
                                    scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                                }
                                if (rightIndex != -1)
                                {
                                    int rightLevel = pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                                    scaleLeveli =
                                        (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel : scaleLeveli;
                                }
                            }

                            if (scaleLeveli <= scaleLevel + 1)
                            {
                                nObs++;
                                if (nObs > thObs)
                                {
                                    break;
                                }
                            }
                        }
                        if (nObs > thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if (nRedundantObservations > redundant_th * nMPs)
        {
            pKF->SetBadFlag();
        }
        if ((count > mKeyFrameCullingEarlyExitAfterAbort && mbAbortBA) || count > mKeyFrameCullingMaxKeyframesToCheck)
        {
            break;
        }
    }
}

void LocalMapping::RequestReset()
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while (1)
    {
        {
            std::unique_lock<std::mutex> lock2(mMutexReset);
            if (!mbResetRequested)
            {
                break;
            }
        }
        usleep(3000);
    }
}

void LocalMapping::RequestResetActiveMap(Map* pMap)
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbResetRequestedActiveMap = true;
        mpMapToReset = pMap;
    }

    while (1)
    {
        {
            std::unique_lock<std::mutex> lock2(mMutexReset);
            if (!mbResetRequestedActiveMap)
            {
                break;
            }
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    bool executed_reset = false;
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbResetRequested)
        {
            executed_reset = true;

            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested = false;
            mbResetRequestedActiveMap = false;
        }

        if (mbResetRequestedActiveMap)
        {
            executed_reset = true;
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();

            mbResetRequested = false;
            mbResetRequestedActiveMap = false;
        }
    }
}

void LocalMapping::RequestFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
    std::unique_lock<std::mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

bool LocalMapping::IsInitializing()
{
    return bInitializing;
}

double LocalMapping::GetCurrKFTime()
{

    if (mpCurrentKeyFrame)
    {
        return mpCurrentKeyFrame->mTimeStamp;
    }
    else
    {
        return 0.0;
    }
}

KeyFrame* LocalMapping::GetCurrKF()
{
    return mpCurrentKeyFrame;
}

}  // namespace ORB_SLAM3
