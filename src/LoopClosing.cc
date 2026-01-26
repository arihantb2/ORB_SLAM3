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

#include "LoopClosing.h"

#include "Atlas.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "Map.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Sim3Solver.h"
#include "System.h"
#include "Tracking.h"

#include <mutex>
#include <thread>

namespace ORB_SLAM3
{

LoopClosing::LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc, const bool bFixScale,
                         const bool bActiveLC)
    : mbResetRequested(false),
      mbResetActiveMapRequested(false),
      mbFinishRequested(false),
      mbFinished(true),
      mpAtlas(pAtlas),
      mpKeyFrameDB(pDB),
      mpORBVocabulary(pVoc),
      mpMatchedKF(NULL),
      mLastLoopKFid(0),
      mbRunningGBA(false),
      mbFinishedGBA(true),
      mbStopGBA(false),
      mpThreadGBA(NULL),
      mbFixScale(bFixScale),
      mnFullBAIdx(0),
      mnLoopNumCoincidences(0),
      mnMergeNumCoincidences(0),
      mbLoopDetected(false),
      mbMergeDetected(false),
      mnLoopNumNotFound(0),
      mnMergeNumNotFound(0),
      mbActiveLC(bActiveLC)
{
    mnCovisibilityConsistencyTh = 3;
    mpLastCurrentKF = static_cast<KeyFrame*>(NULL);

    mstrFolderSubTraj = "SubTrajectories/";
    mnNumCorrection = 0;
    mnCorrectionGBA = 0;
}

void LoopClosing::SetTracker(Tracking* pTracker)
{
    mpTracker = pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping* pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void LoopClosing::Run()
{
    mbFinished = false;

    while (1)
    {

        //NEW LOOP AND MERGE DETECTION ALGORITHM
        //----------------------------

        if (CheckNewKeyFrames())
        {
            if (mpLastCurrentKF)
            {
                mpLastCurrentKF->mvpLoopCandKFs.clear();
                mpLastCurrentKF->mvpMergeCandKFs.clear();
            }

            bool bFindedRegion = NewDetectCommonRegions();

            if (bFindedRegion)
            {
                if (mbMergeDetected)
                {
                    if ((mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO ||
                         mpTracker->mSensor == System::IMU_RGBD) &&
                        (!mpCurrentKF->GetMap()->isImuInitialized()))
                    {
                        Verbose::Print(Verbose::VERBOSITY_NORMAL) << "IMU is not initilized, merge is aborted" << endl;
                    }
                    else
                    {
                        Sophus::SE3d mTmw = mpMergeMatchedKF->GetPose().cast<double>();
                        g2o::Sim3 gSmw2(mTmw.unit_quaternion(), mTmw.translation(), 1.0);
                        Sophus::SE3d mTcw = mpCurrentKF->GetPose().cast<double>();
                        g2o::Sim3 gScw1(mTcw.unit_quaternion(), mTcw.translation(), 1.0);
                        g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
                        g2o::Sim3 gSw1m = mg2oMergeSlw;

                        mSold_new = (gSw2c * gScw1);

                        if (mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
                        {
                            Verbose::Print(Verbose::VERBOSITY_NORMAL) << "Merge check transformation with IMU" << endl;
                            if (mSold_new.scale() < 0.90 || mSold_new.scale() > 1.1)
                            {
                                mpMergeLastCurrentKF->SetErase();
                                mpMergeMatchedKF->SetErase();
                                mnMergeNumCoincidences = 0;
                                mvpMergeMatchedMPs.clear();
                                mvpMergeMPs.clear();
                                mnMergeNumNotFound = 0;
                                mbMergeDetected = false;
                                Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                                continue;
                            }
                            // If inertial, force only yaw
                            if ((mpTracker->mSensor == System::IMU_MONOCULAR ||
                                 mpTracker->mSensor == System::IMU_STEREO || mpTracker->mSensor == System::IMU_RGBD) &&
                                mpCurrentKF->GetMap()->GetIniertialBA1())
                            {
                                Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                                phi(0) = 0;
                                phi(1) = 0;
                                mSold_new = g2o::Sim3(ExpSO3(phi), mSold_new.translation(), 1.0);
                            }
                        }

                        mg2oMergeSmw = gSmw2 * gSw2c * gScw1;

                        mg2oMergeScw = mg2oMergeSlw;

                        //mpTracker->SetStepByStep(true);

                        Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);

                        // TODO UNCOMMENT
                        if (mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO ||
                            mpTracker->mSensor == System::IMU_RGBD)
                            MergeLocal2();
                        else
                            MergeLocal();

                        Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
                    }

                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(1);

                    // Reset all variables
                    mpMergeLastCurrentKF->SetErase();
                    mpMergeMatchedKF->SetErase();
                    mnMergeNumCoincidences = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeMPs.clear();
                    mnMergeNumNotFound = 0;
                    mbMergeDetected = false;

                    if (mbLoopDetected)
                    {
                        // Reset Loop variables
                        mpLoopLastCurrentKF->SetErase();
                        mpLoopMatchedKF->SetErase();
                        mnLoopNumCoincidences = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopMPs.clear();
                        mnLoopNumNotFound = 0;
                        mbLoopDetected = false;
                    }
                }

                if (mbLoopDetected)
                {
                    bool bGoodLoop = true;
                    vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
                    vnPR_TypeRecogn.push_back(0);

                    Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);

                    mg2oLoopScw = mg2oLoopSlw;  //*mvg2oSim3LoopTcw[nCurrentIndex];
                    if (mpCurrentKF->GetMap()->IsInertial())
                    {
                        Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
                        g2o::Sim3 g2oTwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
                        g2o::Sim3 g2oSww_new = g2oTwc * mg2oLoopScw;

                        Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
                        Verbose::Print(Verbose::VERBOSITY_NORMAL) << "phi = " << phi.transpose() << endl;
                        if (fabs(phi(0)) < 0.008f && fabs(phi(1)) < 0.008f && fabs(phi(2)) < 0.349f)
                        {
                            if (mpCurrentKF->GetMap()->IsInertial())
                            {
                                // If inertial, force only yaw
                                if ((mpTracker->mSensor == System::IMU_MONOCULAR ||
                                     mpTracker->mSensor == System::IMU_STEREO ||
                                     mpTracker->mSensor == System::IMU_RGBD) &&
                                    mpCurrentKF->GetMap()->GetIniertialBA2())
                                {
                                    phi(0) = 0;
                                    phi(1) = 0;
                                    g2oSww_new = g2o::Sim3(ExpSO3(phi), g2oSww_new.translation(), 1.0);
                                    mg2oLoopScw = g2oTwc.inverse() * g2oSww_new;
                                }
                            }
                        }
                        else
                        {
                            Verbose::Print(Verbose::VERBOSITY_NORMAL) << "BAD LOOP!!!" << endl;
                            bGoodLoop = false;
                        }
                    }

                    if (bGoodLoop)
                    {

                        mvpLoopMapPoints = mvpLoopMPs;

                        CorrectLoop();

                        mnNumCorrection += 1;
                    }

                    // Reset all variables
                    mpLoopLastCurrentKF->SetErase();
                    mpLoopMatchedKF->SetErase();
                    mnLoopNumCoincidences = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopMPs.clear();
                    mnLoopNumNotFound = 0;
                    mbLoopDetected = false;
                }

                mpLastCurrentKF = mpCurrentKF;
            }
        }

        ResetIfRequested();

        if (CheckFinish())
        {
            break;
        }

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if (pKF->mnId != 0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return (!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::NewDetectCommonRegions()
{
    // To deactivate placerecognition. No loopclosing nor merging will be performed
    if (!mbActiveLC)
        return false;

    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
        mpCurrentKF->mbCurrentPlaceRecognition = true;

        mpLastMap = mpCurrentKF->GetMap();
    }

    if (mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA2())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    if (mpTracker->mSensor == System::STEREO && mpLastMap->GetAllKeyFrames().size() < 5)  //12
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    if (mpLastMap->GetAllKeyFrames().size() < 12)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    //Check the last candidates with geometric validation
    // Loop candidates
    bool bLoopDetectedInKF = false;
    bool bCheckSpatial = false;

    if (mnLoopNumCoincidences > 0)
    {
        bCheckSpatial = true;
        // Find from the last KF candidates
        Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse()).cast<double>();
        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw = gScl * mg2oLoopSlw;
        int numProjMatches = 0;
        vector<MapPoint*> vpMatchedMPs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches,
                                                            mvpLoopMPs, vpMatchedMPs);
        if (bCommonRegion)
        {

            bLoopDetectedInKF = true;

            mnLoopNumCoincidences++;
            mpLoopLastCurrentKF->SetErase();
            mpLoopLastCurrentKF = mpCurrentKF;
            mg2oLoopSlw = gScw;
            mvpLoopMatchedMPs = vpMatchedMPs;

            mbLoopDetected = mnLoopNumCoincidences >= 3;
            mnLoopNumNotFound = 0;

            if (!mbLoopDetected)
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL) << "PR: Loop detected with Reffine Sim3" << endl;
            }
        }
        else
        {
            bLoopDetectedInKF = false;

            mnLoopNumNotFound++;
            if (mnLoopNumNotFound >= 2)
            {
                mpLoopLastCurrentKF->SetErase();
                mpLoopMatchedKF->SetErase();
                mnLoopNumCoincidences = 0;
                mvpLoopMatchedMPs.clear();
                mvpLoopMPs.clear();
                mnLoopNumNotFound = 0;
            }
        }
    }

    //Merge candidates
    bool bMergeDetectedInKF = false;
    if (mnMergeNumCoincidences > 0)
    {
        // Find from the last KF candidates
        Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();

        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw = gScl * mg2oMergeSlw;
        int numProjMatches = 0;
        vector<MapPoint*> vpMatchedMPs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches,
                                                            mvpMergeMPs, vpMatchedMPs);
        if (bCommonRegion)
        {
            bMergeDetectedInKF = true;

            mnMergeNumCoincidences++;
            mpMergeLastCurrentKF->SetErase();
            mpMergeLastCurrentKF = mpCurrentKF;
            mg2oMergeSlw = gScw;
            mvpMergeMatchedMPs = vpMatchedMPs;

            mbMergeDetected = mnMergeNumCoincidences >= 3;
        }
        else
        {
            mbMergeDetected = false;
            bMergeDetectedInKF = false;

            mnMergeNumNotFound++;
            if (mnMergeNumNotFound >= 2)
            {
                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mnMergeNumNotFound = 0;
            }
        }
    }

    if (mbMergeDetected || mbLoopDetected)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        return true;
    }

    //TODO: This is only necessary if we use a minimun score for pick the best candidates
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

    // Extract candidates from the bag of words
    vector<KeyFrame*> vpMergeBowCand, vpLoopBowCand;
    if (!bMergeDetectedInKF || !bLoopDetectedInKF)
    {
        // Search in BoW
        mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand, 3);
    }

    // Check the BoW candidates if the geometric candidate list is empty
    //Loop candidates
    if (!bLoopDetectedInKF && !vpLoopBowCand.empty())
    {
        mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw,
                                                    mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
    }
    // Merge candidates
    if (!bMergeDetectedInKF && !vpMergeBowCand.empty())
    {
        mbMergeDetected =
            DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw,
                                       mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
    }

    mpKeyFrameDB->add(mpCurrentKF);

    if (mbMergeDetected || mbLoopDetected)
    {
        return true;
    }

    mpCurrentKF->SetErase();
    mpCurrentKF->mbCurrentPlaceRecognition = false;

    return false;
}

bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3& gScw,
                                                 int& nNumProjMatches, std::vector<MapPoint*>& vpMPs,
                                                 std::vector<MapPoint*>& vpMatchedMPs)
{
    set<MapPoint*> spAlreadyMatchedMPs;
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

    int nProjMatches = 30;
    int nProjOptMatches = 50;
    int nProjMatchesRep = 100;

    if (nNumProjMatches >= nProjMatches)
    {
        Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
        g2o::Sim3 gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
        g2o::Sim3 gScm = gScw * gSwm;
        Eigen::Matrix<double, 7, 7> mHessian7x7;

        bool bFixedScale = mbFixScale;  // TODO CHECK; Solo para el monocular inertial
        if (mpTracker->mSensor == System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2())
            bFixedScale = false;
        int numOptMatches =
            Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);

        if (numOptMatches > nProjOptMatches)
        {
            g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(), 1.0);

            vector<MapPoint*> vpMatchedMP;
            vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));

            nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs,
                                                      vpMPs, vpMatchedMPs);
            if (nNumProjMatches >= nProjMatchesRep)
            {
                gScw = gScw_estimation;
                return true;
            }
        }
    }
    return false;
}

bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<KeyFrame*>& vpBowCand, KeyFrame*& pMatchedKF2,
                                             KeyFrame*& pLastCurrentKF, g2o::Sim3& g2oScw, int& nNumCoincidences,
                                             std::vector<MapPoint*>& vpMPs, std::vector<MapPoint*>& vpMatchedMPs)
{
    int nBoWMatches = 20;
    int nBoWInliers = 15;
    int nSim3Inliers = 20;
    int nProjMatches = 50;
    int nProjOptMatches = 80;

    set<KeyFrame*> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

    int nNumCovisibles = 10;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);

    // Varibles to select the best numbe
    KeyFrame* pBestMatchedKF;
    int nBestMatchesReproj = 0;
    int nBestNumCoindicendes = 0;
    g2o::Sim3 g2oBestScw;
    std::vector<MapPoint*> vpBestMapPoints;
    std::vector<MapPoint*> vpBestMatchedMapPoints;

    int numCandidates = vpBowCand.size();
    vector<int> vnStage(numCandidates, 0);
    vector<int> vnMatchesStage(numCandidates, 0);

    int index = 0;
    for (KeyFrame* pKFi : vpBowCand)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        // Current KF against KF with covisibles version
        std::vector<KeyFrame*> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
        if (vpCovKFi.empty())
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL) << "Covisible list empty" << std::endl;
            vpCovKFi.push_back(pKFi);
        }
        else
        {
            vpCovKFi.push_back(vpCovKFi[0]);
            vpCovKFi[0] = pKFi;
        }

        bool bAbortByNearKF = false;
        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            if (spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
            {
                bAbortByNearKF = true;
                break;
            }
        }
        if (bAbortByNearKF)
        {
            continue;
        }

        std::vector<std::vector<MapPoint*>> vvpMatchedMPs;
        vvpMatchedMPs.resize(vpCovKFi.size());
        std::set<MapPoint*> spMatchedMPi;
        int numBoWMatches = 0;

        KeyFrame* pMostBoWMatchesKF = pKFi;
        int nMostBoWNumMatches = 0;

        std::vector<MapPoint*> vpMatchedPoints =
            std::vector<MapPoint*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
        std::vector<KeyFrame*> vpKeyFrameMatchedMP =
            std::vector<KeyFrame*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));

        int nIndexMostBoWMatchesKF = 0;
        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            if (!vpCovKFi[j] || vpCovKFi[j]->isBad())
                continue;

            int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
            if (num > nMostBoWNumMatches)
            {
                nMostBoWNumMatches = num;
                nIndexMostBoWMatchesKF = j;
            }
        }

        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            for (int k = 0; k < vvpMatchedMPs[j].size(); ++k)
            {
                MapPoint* pMPi_j = vvpMatchedMPs[j][k];
                if (!pMPi_j || pMPi_j->isBad())
                    continue;

                if (spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                {
                    spMatchedMPi.insert(pMPi_j);
                    numBoWMatches++;

                    vpMatchedPoints[k] = pMPi_j;
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                }
            }
        }

        //pMostBoWMatchesKF = vpCovKFi[pMostBoWMatchesKF];

        if (numBoWMatches >= nBoWMatches)  // TODO pick a good threshold
        {
            // Geometric validation
            bool bFixedScale = mbFixScale;
            if (mpTracker->mSensor == System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                bFixedScale = false;

            Sim3Solver solver =
                Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
            solver.SetRansacParameters(0.99, nBoWInliers, 300);  // at least 15 inliers

            bool bNoMore = false;
            vector<bool> vbInliers;
            int nInliers;
            bool bConverge = false;
            Eigen::Matrix4f mTcm;
            while (!bConverge && !bNoMore)
            {
                mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
            }

            if (bConverge)
            {
                // Match by reprojection
                vpCovKFi.clear();
                vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                vpCovKFi.push_back(pMostBoWMatchesKF);
                set<KeyFrame*> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                set<MapPoint*> spMapPoints;
                vector<MapPoint*> vpMapPoints;
                vector<KeyFrame*> vpKeyFrames;
                for (KeyFrame* pCovKFi : vpCovKFi)
                {
                    for (MapPoint* pCovMPij : pCovKFi->GetMapPointMatches())
                    {
                        if (!pCovMPij || pCovMPij->isBad())
                            continue;

                        if (spMapPoints.find(pCovMPij) == spMapPoints.end())
                        {
                            spMapPoints.insert(pCovMPij);
                            vpMapPoints.push_back(pCovMPij);
                            vpKeyFrames.push_back(pCovKFi);
                        }
                    }
                }

                g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(),
                               solver.GetEstimatedTranslation().cast<double>(), (double)solver.GetEstimatedScale());
                g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),
                               pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
                g2o::Sim3 gScw = gScm * gSmw;  // Similarity matrix of current from the world position
                Sophus::Sim3f mScw = Converter::toSophus(gScw);

                vector<MapPoint*> vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                vector<KeyFrame*> vpMatchedKF;
                vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));
                int numProjMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames,
                                                                vpMatchedMP, vpMatchedKF, 8, 1.5);
                if (numProjMatches >= nProjMatches)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    bool bFixedScale = mbFixScale;
                    if (mpTracker->mSensor == System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                        bFixedScale = false;

                    int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale,
                                                                mHessian7x7, true);

                    if (numOptMatches >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),
                                       pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
                        g2o::Sim3 gScw = gScm * gSmw;  // Similarity matrix of current from the world position
                        Sophus::Sim3f mScw = Converter::toSophus(gScw);

                        vector<MapPoint*> vpMatchedMP;
                        vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                        int numProjOptMatches =
                            matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

                        if (numProjOptMatches >= nProjOptMatches)
                        {
                            int max_x = -1, min_x = 1000000;
                            int max_y = -1, min_y = 1000000;
                            for (MapPoint* pMPi : vpMatchedMP)
                            {
                                if (!pMPi || pMPi->isBad())
                                {
                                    continue;
                                }

                                tuple<size_t, size_t> indexes = pMPi->GetIndexInKeyFrame(pKFi);
                                int index = get<0>(indexes);
                                if (index >= 0)
                                {
                                    int coord_x = pKFi->mvKeysUn[index].pt.x;
                                    if (coord_x < min_x)
                                    {
                                        min_x = coord_x;
                                    }
                                    if (coord_x > max_x)
                                    {
                                        max_x = coord_x;
                                    }
                                    int coord_y = pKFi->mvKeysUn[index].pt.y;
                                    if (coord_y < min_y)
                                    {
                                        min_y = coord_y;
                                    }
                                    if (coord_y > max_y)
                                    {
                                        max_y = coord_y;
                                    }
                                }
                            }

                            int nNumKFs = 0;
                            //vpMatchedMPs = vpMatchedMP;
                            //vpMPs = vpMapPoints;
                            // Check the Sim3 transformation with the current KeyFrame covisibles
                            vector<KeyFrame*> vpCurrentCovKFs =
                                mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

                            int j = 0;
                            while (nNumKFs < 3 && j < vpCurrentCovKFs.size())
                            {
                                KeyFrame* pKFj = vpCurrentCovKFs[j];
                                Sophus::SE3d mTjc = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
                                g2o::Sim3 gSjc(mTjc.unit_quaternion(), mTjc.translation(), 1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int numProjMatches_j = 0;
                                vector<MapPoint*> vpMatchedMPs_j;
                                bool bValid = DetectCommonRegionsFromLastKF(
                                    pKFj, pMostBoWMatchesKF, gSjw, numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

                                if (bValid)
                                {
                                    Sophus::SE3f Tc_w = mpCurrentKF->GetPose();
                                    Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
                                    Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                                    Eigen::Vector3f vector_dist = Tc_cj.translation();
                                    nNumKFs++;
                                }
                                j++;
                            }

                            if (nNumKFs < 3)
                            {
                                vnStage[index] = 8;
                                vnMatchesStage[index] = nNumKFs;
                            }

                            if (nBestMatchesReproj < numProjOptMatches)
                            {
                                nBestMatchesReproj = numProjOptMatches;
                                nBestNumCoindicendes = nNumKFs;
                                pBestMatchedKF = pMostBoWMatchesKF;
                                g2oBestScw = gScw;
                                vpBestMapPoints = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMP;
                            }
                        }
                    }
                }
            }
            /*else
            {
                Verbose::PrintMess("BoW candidate: it don't match with the current one", Verbose::VERBOSITY_DEBUG);
            }*/
        }
        index++;
    }

    if (nBestMatchesReproj > 0)
    {
        pLastCurrentKF = mpCurrentKF;
        nNumCoincidences = nBestNumCoindicendes;
        pMatchedKF2 = pBestMatchedKF;
        pMatchedKF2->SetNotErase();
        g2oScw = g2oBestScw;
        vpMPs = vpBestMapPoints;
        vpMatchedMPs = vpBestMatchedMapPoints;

        return nNumCoincidences >= 3;
    }
    else
    {
        int maxStage = -1;
        int maxMatched;
        for (int i = 0; i < vnStage.size(); ++i)
        {
            if (vnStage[i] > maxStage)
            {
                maxStage = vnStage[i];
                maxMatched = vnMatchesStage[i];
            }
        }
    }
    return false;
}

bool LoopClosing::DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3& gScw,
                                                int& nNumProjMatches, std::vector<MapPoint*>& vpMPs,
                                                std::vector<MapPoint*>& vpMatchedMPs)
{
    set<MapPoint*> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

    int nProjMatches = 30;
    if (nNumProjMatches >= nProjMatches)
    {
        return true;
    }

    return false;
}

int LoopClosing::FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3& g2oScw,
                                         set<MapPoint*>& spMatchedMPinOrigin, vector<MapPoint*>& vpMapPoints,
                                         vector<MapPoint*>& vpMatchedMapPoints)
{
    int nNumCovisibles = 10;
    vector<KeyFrame*> vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<KeyFrame*> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<KeyFrame*> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
    if (nInitialCov < nNumCovisibles)
    {
        for (int i = 0; i < nInitialCov; ++i)
        {
            vector<KeyFrame*> vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
            int nInserted = 0;
            int j = 0;
            while (j < vpKFs.size() && nInserted < nNumCovisibles)
            {
                if (spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() &&
                    spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end())
                {
                    spCheckKFs.insert(vpKFs[j]);
                    ++nInserted;
                }
                ++j;
            }
            vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
        }
    }
    set<MapPoint*> spMapPoints;
    vpMapPoints.clear();
    vpMatchedMapPoints.clear();
    for (KeyFrame* pKFi : vpCovKFm)
    {
        for (MapPoint* pMPij : pKFi->GetMapPointMatches())
        {
            if (!pMPij || pMPij->isBad())
                continue;

            if (spMapPoints.find(pMPij) == spMapPoints.end())
            {
                spMapPoints.insert(pMPij);
                vpMapPoints.push_back(pMPij);
            }
        }
    }

    Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
    ORBmatcher matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
    int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

    return num_matches;
}

void LoopClosing::CorrectLoop()
{
    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();
    mpLocalMapper->EmptyQueue();  // Proccess keyframes in the queue

    // If a Global Bundle Adjustment is running, abort it
    if (isRunningGBA())
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL) << "Stoping Global Bundle Adjustment...";
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if (mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        Verbose::Print(Verbose::VERBOSITY_NORMAL) << "  Done!!" << endl;
    }

    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF] = mg2oLoopScw;
    Sophus::SE3f Twc = mpCurrentKF->GetPoseInverse();
    Sophus::SE3f Tcw = mpCurrentKF->GetPose();
    g2o::Sim3 g2oScw(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>(), 1.0);
    NonCorrectedSim3[mpCurrentKF] = g2oScw;

    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
    Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(), mg2oLoopScw.translation() / mg2oLoopScw.scale());
    mpCurrentKF->SetPose(correctedTcw.cast<float>());

    Map* pLoopMap = mpCurrentKF->GetMap();

    {
        // Get Map Mutex
        unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

        const bool bImuInit = pLoopMap->isImuInitialized();

        for (vector<KeyFrame*>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end();
             vit != vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            if (pKFi != mpCurrentKF)
            {
                Sophus::SE3f Tiw = pKFi->GetPose();
                Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
                g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oLoopScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi] = g2oCorrectedSiw;

                // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),
                                          g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
                pKFi->SetPose(correctedTiw.cast<float>());

                //Pose without correction
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(), Tiw.translation().cast<double>(), 1.0);
                NonCorrectedSim3[pKFi] = g2oSiw;
            }
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end(); mit != mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            /*Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
            pKFi->SetPose(correctedTiw.cast<float>());*/

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if (!pMPi)
                    continue;
                if (pMPi->isBad())
                    continue;
                if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Correct velocity according to orientation correction
            if (bImuInit)
            {
                Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation()).cast<float>();
                pKFi->SetVelocity(Rcor * pKFi->GetVelocity());
            }

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }
        // TODO Check this index increasement
        mpAtlas->GetCurrentMap()->IncreaseChangeIndex();

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for (size_t i = 0; i < mvpLoopMatchedMPs.size(); i++)
        {
            if (mvpLoopMatchedMPs[i])
            {
                MapPoint* pLoopMP = mvpLoopMatchedMPs[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if (pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP, i);
                    pLoopMP->AddObservation(mpCurrentKF, i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);

    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*>> LoopConnections;

    for (vector<KeyFrame*>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end();
         vit != vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
        for (vector<KeyFrame*>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end();
             vit_prev != vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for (vector<KeyFrame*>::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end();
             vit2 != vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    bool bFixedScale = mbFixScale;
    // TODO CHECK; Solo para el monocular inertial
    if (mpTracker->mSensor == System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
        bFixedScale = false;

    if (pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
    {
        Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3,
                                              LoopConnections);
    }
    else
    {
        Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3,
                                          LoopConnections, bFixedScale);
    }

    mpAtlas->InformNewBigChange();

    // Add loop edge
    mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
    if (!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1))
    {
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;
        mnCorrectionGBA = mnNumCorrection;

        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
    }

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    mLastLoopKFid = mpCurrentKF->mnId;  //TODO old varible, it is not use in the new algorithm
}

void LoopClosing::MergeLocal()
{
    int numTemporalKFs = 25;  //Temporal KFs in the local window if the map is inertial.

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    KeyFrame* pNewChild;
    KeyFrame* pNewParent;

    vector<KeyFrame*> vpLocalCurrentWindowKFs;
    vector<KeyFrame*> vpMergeConnectedKFs;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    bool bRelaunchBA = false;

    // If a Global Bundle Adjustment is running, abort it
    if (isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if (mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        bRelaunchBA = true;
    }

    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    mpLocalMapper->EmptyQueue();

    // Merge map will become in the new active map with the local window of KFs and MPs from the current map.
    // Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking
    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMergeMap = mpMergeMatchedKF->GetMap();

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    //Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
    set<KeyFrame*> spLocalWindowKFs;
    //Get MPs in the welding area from the current map
    set<MapPoint*> spLocalWindowMPs;
    if (pCurrentMap->IsInertial() && pMergeMap->IsInertial())  //TODO Check the correct initialization
    {
        KeyFrame* pKFi = mpCurrentKF;
        int nInserted = 0;
        while (pKFi && nInserted < numTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;

            set<MapPoint*> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
        }

        pKFi = mpCurrentKF->mNextKF;
        while (pKFi)
        {
            spLocalWindowKFs.insert(pKFi);

            set<MapPoint*> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());

            pKFi = mpCurrentKF->mNextKF;
        }
    }
    else
    {
        spLocalWindowKFs.insert(mpCurrentKF);
    }

    vector<KeyFrame*> vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    spLocalWindowKFs.insert(mpCurrentKF);
    const int nMaxTries = 5;
    int nNumTries = 0;
    while (spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<KeyFrame*> vpNewCovKFs;
        vpNewCovKFs.empty();
        for (KeyFrame* pKFi : spLocalWindowKFs)
        {
            vector<KeyFrame*> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);
            for (KeyFrame* pKFcov : vpKFiCov)
            {
                if (pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }
            }
        }

        spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        nNumTries++;
    }

    for (KeyFrame* pKFi : spLocalWindowKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        set<MapPoint*> spMPs = pKFi->GetMapPoints();
        spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
    }

    set<KeyFrame*> spMergeConnectedKFs;
    if (pCurrentMap->IsInertial() && pMergeMap->IsInertial())  //TODO Check the correct initialization
    {
        KeyFrame* pKFi = mpMergeMatchedKF;
        int nInserted = 0;
        while (pKFi && nInserted < numTemporalKFs / 2)
        {
            spMergeConnectedKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;
        }

        pKFi = mpMergeMatchedKF->mNextKF;
        while (pKFi && nInserted < numTemporalKFs)
        {
            spMergeConnectedKFs.insert(pKFi);
            pKFi = mpCurrentKF->mNextKF;
        }
    }
    else
    {
        spMergeConnectedKFs.insert(mpMergeMatchedKF);
    }
    vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
    spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
    spMergeConnectedKFs.insert(mpMergeMatchedKF);
    nNumTries = 0;
    while (spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
    {
        vector<KeyFrame*> vpNewCovKFs;
        for (KeyFrame* pKFi : spMergeConnectedKFs)
        {
            vector<KeyFrame*> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);
            for (KeyFrame* pKFcov : vpKFiCov)
            {
                if (pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end())
                {
                    vpNewCovKFs.push_back(pKFcov);
                }
            }
        }

        spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
        nNumTries++;
    }

    set<MapPoint*> spMapPointMerge;
    for (KeyFrame* pKFi : spMergeConnectedKFs)
    {
        set<MapPoint*> vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
    }

    vector<MapPoint*> vpCheckFuseMapPoint;
    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
    g2o::Sim3 g2oNonCorrectedSwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
    g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
    g2o::Sim3 g2oCorrectedScw = mg2oMergeScw;  //TODO Check the transformation

    KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
    vCorrectedSim3[mpCurrentKF] = g2oCorrectedScw;
    vNonCorrectedSim3[mpCurrentKF] = g2oNonCorrectedScw;

    for (KeyFrame* pKFi : spLocalWindowKFs)
    {
        if (!pKFi || pKFi->isBad())
        {
            Verbose::PrintMess("Bad KF in correction", Verbose::VERBOSITY_DEBUG);
            continue;
        }

        if (pKFi->GetMap() != pCurrentMap)
            Verbose::PrintMess("Other map KF, this should't happen", Verbose::VERBOSITY_DEBUG);

        g2o::Sim3 g2oCorrectedSiw;

        if (pKFi != mpCurrentKF)
        {
            Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
            g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            //Pose without correction
            vNonCorrectedSim3[pKFi] = g2oSiw;

            Sophus::SE3d Tic = Tiw * Twc;
            g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
            g2oCorrectedSiw = g2oSic * mg2oMergeScw;
            vCorrectedSim3[pKFi] = g2oCorrectedSiw;
        }
        else
        {
            g2oCorrectedSiw = g2oCorrectedScw;
        }
        pKFi->mTcwMerge = pKFi->GetPose();

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        double s = g2oCorrectedSiw.scale();
        pKFi->mfScale = s;
        Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

        pKFi->mTcwMerge = correctedTiw.cast<float>();

        if (pCurrentMap->isImuInitialized())
        {
            Eigen::Quaternionf Rcor =
                (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
            pKFi->mVwbMerge = Rcor * pKFi->GetVelocity();
        }

        //TODO DEBUG to know which are the KFs that had been moved to the other map
    }

    int numPointsWithCorrection = 0;

    set<MapPoint*>::iterator itMP = spLocalWindowMPs.begin();
    while (itMP != spLocalWindowMPs.end())
    {
        MapPoint* pMPi = *itMP;
        if (!pMPi || pMPi->isBad())
        {
            itMP = spLocalWindowMPs.erase(itMP);
            continue;
        }

        KeyFrame* pKFref = pMPi->GetReferenceKeyFrame();
        if (vCorrectedSim3.find(pKFref) == vCorrectedSim3.end())
        {
            itMP = spLocalWindowMPs.erase(itMP);
            numPointsWithCorrection++;
            continue;
        }
        g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

        // Project with non-corrected pose and project back with corrected pose
        Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
        Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
        Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

        pMPi->mPosMerge = eigCorrectedP3Dw.cast<float>();
        pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

        itMP++;
    }

    {
        unique_lock<mutex> currentLock(
            pCurrentMap->mMutexMapUpdate);  // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(
            pMergeMap->mMutexMapUpdate);  // We remove the Kfs and MPs in the merged area from the old map

        for (KeyFrame* pKFi : spLocalWindowKFs)
        {
            if (!pKFi || pKFi->isBad())
            {
                continue;
            }

            pKFi->mTcwBefMerge = pKFi->GetPose();
            pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
            pKFi->SetPose(pKFi->mTcwMerge);

            // Make sure connections are updated
            pKFi->UpdateMap(pMergeMap);
            pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;
            pMergeMap->AddKeyFrame(pKFi);
            pCurrentMap->EraseKeyFrame(pKFi);

            if (pCurrentMap->isImuInitialized())
            {
                pKFi->SetVelocity(pKFi->mVwbMerge);
            }
        }

        for (MapPoint* pMPi : spLocalWindowMPs)
        {
            if (!pMPi || pMPi->isBad())
                continue;

            pMPi->SetWorldPos(pMPi->mPosMerge);
            pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
            pMPi->UpdateMap(pMergeMap);
            pMergeMap->AddMapPoint(pMPi);
            pCurrentMap->EraseMapPoint(pMPi);
        }

        mpAtlas->ChangeMap(pMergeMap);
        mpAtlas->SetMapBad(pCurrentMap);
        pMergeMap->IncreaseChangeIndex();
        //TODO for debug
        pMergeMap->ChangeId(pCurrentMap->GetId());
    }

    //Rebuild the essential graph in the local window
    pCurrentMap->GetOriginKF()->SetFirstConnection(false);
    pNewChild = mpCurrentKF->GetParent();  // Old parent, it will be the new child of this KF
    pNewParent =
        mpCurrentKF;  // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpCurrentKF->ChangeParent(mpMergeMatchedKF);
    while (pNewChild)
    {
        pNewChild->EraseChild(pNewParent);  // We remove the relation between the old parent and the new for avoid loop
        KeyFrame* pOldParent = pNewChild->GetParent();

        pNewChild->ChangeParent(pNewParent);

        pNewParent = pNewChild;
        pNewChild = pOldParent;
    }

    //Update the connections between the local window
    mpMergeMatchedKF->UpdateConnections();

    vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    vpMergeConnectedKFs.push_back(mpMergeMatchedKF);

    // Project MapPoints observed in the neighborhood of the merge keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);

    // Update connectivity
    for (KeyFrame* pKFi : spLocalWindowKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for (KeyFrame* pKFi : spMergeConnectedKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    bool bStop = false;
    vpLocalCurrentWindowKFs.clear();
    vpMergeConnectedKFs.clear();
    std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
    std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
    if (mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO ||
        mpTracker->mSensor == System::IMU_RGBD)
    {
        Optimizer::MergeInertialBA(mpCurrentKF, mpMergeMatchedKF, &bStop, pCurrentMap, vCorrectedSim3);
    }
    else
    {
        Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs, vpMergeConnectedKFs, &bStop);
    }

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();

    //Update the non critical area from the current map to the merged map
    vector<KeyFrame*> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
    vector<MapPoint*> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

    if (vpCurrentMapKFs.size() == 0)
    {
    }
    else
    {
        if (mpTracker->mSensor == System::MONOCULAR)
        {
            unique_lock<mutex> currentLock(
                pCurrentMap->mMutexMapUpdate);  // We update the current map with the Merge information

            for (KeyFrame* pKFi : vpCurrentMapKFs)
            {
                if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                {
                    continue;
                }

                g2o::Sim3 g2oCorrectedSiw;

                Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
                //Pose without correction
                vNonCorrectedSim3[pKFi] = g2oSiw;

                Sophus::SE3d Tic = Tiw * Twc;
                g2o::Sim3 g2oSim(Tic.unit_quaternion(), Tic.translation(), 1.0);
                g2oCorrectedSiw = g2oSim * mg2oMergeScw;
                vCorrectedSim3[pKFi] = g2oCorrectedSiw;

                // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                double s = g2oCorrectedSiw.scale();

                pKFi->mfScale = s;

                Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

                pKFi->mTcwBefMerge = pKFi->GetPose();
                pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

                pKFi->SetPose(correctedTiw.cast<float>());

                if (pCurrentMap->isImuInitialized())
                {
                    Eigen::Quaternionf Rcor =
                        (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
                    pKFi->SetVelocity(Rcor * pKFi->GetVelocity());  // TODO: should add here scale s
                }
            }
            for (MapPoint* pMPi : vpCurrentMapMPs)
            {
                if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pCurrentMap)
                    continue;

                KeyFrame* pKFref = pMPi->GetReferenceKeyFrame();
                g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
                g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

                // Project with non-corrected pose and project back with corrected pose
                Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
                pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());

                pMPi->UpdateNormalAndDepth();
            }
        }

        mpLocalMapper->RequestStop();
        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->isStopped())
        {
            usleep(1000);
        }

        // Optimize graph (and update the loop position for each element form the begining to the end)
        if (mpTracker->mSensor != System::MONOCULAR)
        {
            Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs,
                                              vpCurrentMapKFs, vpCurrentMapMPs);
        }

        {
            // Get Merge Map Mutex
            unique_lock<mutex> currentLock(
                pCurrentMap->mMutexMapUpdate);  // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(
                pMergeMap->mMutexMapUpdate);  // We remove the Kfs and MPs in the merged area from the old map

            for (KeyFrame* pKFi : vpCurrentMapKFs)
            {
                if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
                {
                    continue;
                }

                // Make sure connections are updated
                pKFi->UpdateMap(pMergeMap);
                pMergeMap->AddKeyFrame(pKFi);
                pCurrentMap->EraseKeyFrame(pKFi);
            }

            for (MapPoint* pMPi : vpCurrentMapMPs)
            {
                if (!pMPi || pMPi->isBad())
                    continue;

                pMPi->UpdateMap(pMergeMap);
                pMergeMap->AddMapPoint(pMPi);
                pCurrentMap->EraseMapPoint(pMPi);
            }
        }
    }

    mpLocalMapper->Release();

    if (bRelaunchBA &&
        (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1)))
    {
        // Launch a new thread to perform Global Bundle Adjustment
        mbRunningGBA = true;
        mbFinishedGBA = false;
        mbStopGBA = false;
        mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pMergeMap, mpCurrentKF->mnId);
    }

    mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
    mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

    pCurrentMap->IncreaseChangeIndex();
    pMergeMap->IncreaseChangeIndex();

    mpAtlas->RemoveBadMaps();
}

void LoopClosing::MergeLocal2()
{

    int numTemporalKFs = 11;  //TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

    //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
    KeyFrame* pNewChild;
    KeyFrame* pNewParent;

    vector<KeyFrame*> vpLocalCurrentWindowKFs;
    vector<KeyFrame*> vpMergeConnectedKFs;

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;

    // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
    bool bRelaunchBA = false;

    // If a Global Bundle Adjustment is running, abort it
    if (isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if (mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
        bRelaunchBA = true;
    }

    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    Map* pCurrentMap = mpCurrentKF->GetMap();
    Map* pMergeMap = mpMergeMatchedKF->GetMap();

    {
        float s_on = mSold_new.scale();
        Sophus::SE3f T_on(mSold_new.rotation().cast<float>(), mSold_new.translation().cast<float>());

        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

        mpLocalMapper->EmptyQueue();

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        bool bScaleVel = false;
        if (s_on != 1)
            bScaleVel = true;
        mpAtlas->GetCurrentMap()->ApplyScaledRotation(T_on, s_on, bScaleVel);
        mpTracker->UpdateFrameIMU(s_on, mpCurrentKF->GetImuBias(), mpTracker->GetLastKeyFrame());

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    }

    const int numKFnew = pCurrentMap->KeyFramesInMap();

    if ((mpTracker->mSensor == System::IMU_MONOCULAR || mpTracker->mSensor == System::IMU_STEREO ||
         mpTracker->mSensor == System::IMU_RGBD) &&
        !pCurrentMap->GetIniertialBA2())
    {
        // Map is not completly initialized
        Eigen::Vector3d bg, ba;
        bg << 0., 0., 0.;
        ba << 0., 0., 0.;
        Optimizer::InertialOptimization(pCurrentMap, bg, ba);
        IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        mpTracker->UpdateFrameIMU(1.0f, b, mpTracker->GetLastKeyFrame());

        // Set map initialized
        pCurrentMap->SetIniertialBA2();
        pCurrentMap->SetIniertialBA1();
        pCurrentMap->SetImuInitialized();
    }

    // Load KFs and MPs from merge map
    {
        // Get Merge Map Mutex (This section stops tracking!!)
        unique_lock<mutex> currentLock(
            pCurrentMap->mMutexMapUpdate);  // We update the current map with the Merge information
        unique_lock<mutex> mergeLock(
            pMergeMap->mMutexMapUpdate);  // We remove the Kfs and MPs in the merged area from the old map

        vector<KeyFrame*> vpMergeMapKFs = pMergeMap->GetAllKeyFrames();
        vector<MapPoint*> vpMergeMapMPs = pMergeMap->GetAllMapPoints();

        for (KeyFrame* pKFi : vpMergeMapKFs)
        {
            if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap)
            {
                continue;
            }

            // Make sure connections are updated
            pKFi->UpdateMap(pCurrentMap);
            pCurrentMap->AddKeyFrame(pKFi);
            pMergeMap->EraseKeyFrame(pKFi);
        }

        for (MapPoint* pMPi : vpMergeMapMPs)
        {
            if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                continue;

            pMPi->UpdateMap(pCurrentMap);
            pCurrentMap->AddMapPoint(pMPi);
            pMergeMap->EraseMapPoint(pMPi);
        }

        // Save non corrected poses (already merged maps)
        vector<KeyFrame*> vpKFs = pCurrentMap->GetAllKeyFrames();
        for (KeyFrame* pKFi : vpKFs)
        {
            Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
            g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
            NonCorrectedSim3[pKFi] = g2oSiw;
        }
    }

    pMergeMap->GetOriginKF()->SetFirstConnection(false);
    pNewChild = mpMergeMatchedKF->GetParent();  // Old parent, it will be the new child of this KF
    pNewParent =
        mpMergeMatchedKF;  // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
    mpMergeMatchedKF->ChangeParent(mpCurrentKF);
    while (pNewChild)
    {
        pNewChild->EraseChild(pNewParent);  // We remove the relation between the old parent and the new for avoid loop
        KeyFrame* pOldParent = pNewChild->GetParent();
        pNewChild->ChangeParent(pNewParent);
        pNewParent = pNewChild;
        pNewChild = pOldParent;
    }

    vector<MapPoint*>
        vpCheckFuseMapPoint;  // MapPoint vector from current map to allow to fuse duplicated points with the old map (merge)
    vector<KeyFrame*> vpCurrentConnectedKFs;

    mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
    vector<KeyFrame*> aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(), aux.begin(), aux.end());
    if (mvpMergeConnectedKFs.size() > 6)
        mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin() + 6, mvpMergeConnectedKFs.end());

    mpCurrentKF->UpdateConnections();
    vpCurrentConnectedKFs.push_back(mpCurrentKF);
    aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
    if (vpCurrentConnectedKFs.size() > 6)
        vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin() + 6, vpCurrentConnectedKFs.end());

    set<MapPoint*> spMapPointMerge;
    for (KeyFrame* pKFi : mvpMergeConnectedKFs)
    {
        set<MapPoint*> vpMPs = pKFi->GetMapPoints();
        spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
        if (spMapPointMerge.size() > 1000)
            break;
    }

    vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
    std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

    SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

    for (KeyFrame* pKFi : vpCurrentConnectedKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }
    for (KeyFrame* pKFi : mvpMergeConnectedKFs)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateConnections();
    }

    // TODO Check: If new map is too small, we suppose that not informaiton can be propagated from new to old map
    if (numKFnew < 10)
    {
        mpLocalMapper->Release();
        return;
    }

    // Perform BA
    bool bStopFlag = false;
    KeyFrame* pCurrKF = mpTracker->GetLastKeyFrame();
    Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap, CorrectedSim3);

    // Release Local Mapping.
    mpLocalMapper->Release();

    return;
}

void LoopClosing::CheckObservations(set<KeyFrame*>& spKFsMap1, set<KeyFrame*>& spKFsMap2)
{
    Verbose::Print(Verbose::VERBOSITY_NORMAL) << "----------------------" << endl;
    for (KeyFrame* pKFi1 : spKFsMap1)
    {
        map<KeyFrame*, int> mMatchedMP;
        set<MapPoint*> spMPs = pKFi1->GetMapPoints();

        for (MapPoint* pMPij : spMPs)
        {
            if (!pMPij || pMPij->isBad())
            {
                continue;
            }

            map<KeyFrame*, tuple<int, int>> mMPijObs = pMPij->GetObservations();
            for (KeyFrame* pKFi2 : spKFsMap2)
            {
                if (mMPijObs.find(pKFi2) != mMPijObs.end())
                {
                    if (mMatchedMP.find(pKFi2) != mMatchedMP.end())
                    {
                        mMatchedMP[pKFi2] = mMatchedMP[pKFi2] + 1;
                    }
                    else
                    {
                        mMatchedMP[pKFi2] = 1;
                    }
                }
            }
        }

        if (mMatchedMP.size() == 0)
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "CHECK-OBS: KF " << pKFi1->mnId << " has not any matched MP with the other map" << endl;
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL) << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with "
                                                      << mMatchedMP.size() << " KF from the other map" << endl;
            for (pair<KeyFrame*, int> matchedKF : mMatchedMP)
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "   -KF: " << matchedKF.first->mnId << ", Number of matches: " << matchedKF.second << endl;
            }
        }
    }
    Verbose::Print(Verbose::VERBOSITY_NORMAL) << "----------------------" << endl;
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose& CorrectedPosesMap, vector<MapPoint*>& vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end(); mit != mend;
         mit++)
    {
        int num_replaces = 0;
        KeyFrame* pKFi = mit->first;
        Map* pMap = pKFi->GetMap();

        g2o::Sim3 g2oScw = mit->second;
        Sophus::Sim3f Scw = Converter::toSophus(g2oScw);

        vector<MapPoint*> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint*>(NULL));
        int numFused = matcher.Fuse(pKFi, Scw, vpMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for (int i = 0; i < nLP; i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if (pRep)
            {

                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }

        total_replaces += num_replaces;
    }
}

void LoopClosing::SearchAndFuse(const vector<KeyFrame*>& vConectedKFs, vector<MapPoint*>& vpMapPoints)
{
    ORBmatcher matcher(0.8);

    int total_replaces = 0;

    for (auto mit = vConectedKFs.begin(), mend = vConectedKFs.end(); mit != mend; mit++)
    {
        int num_replaces = 0;
        KeyFrame* pKF = (*mit);
        Map* pMap = pKF->GetMap();
        Sophus::SE3f Tcw = pKF->GetPose();
        Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
        Scw.setScale(1.f);
        vector<MapPoint*> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF, Scw, vpMapPoints, 4, vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        const int nLP = vpMapPoints.size();
        for (int i = 0; i < nLP; i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if (pRep)
            {
                num_replaces += 1;
                pRep->Replace(vpMapPoints[i]);
            }
        }
    }
}

void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while (1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetRequested)
                break;
        }
        usleep(5000);
    }
}

void LoopClosing::RequestResetActiveMap(Map* pMap)
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMapRequested = true;
        mpMapToReset = pMap;
    }

    while (1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if (!mbResetActiveMapRequested)
                break;
        }
        usleep(3000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if (mbResetRequested)
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL) << "Loop closer reset requested..." << endl;
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid = 0;  //TODO old variable, it is not use in the new algorithm
        mbResetRequested = false;
        mbResetActiveMapRequested = false;
    }
    else if (mbResetActiveMapRequested)
    {

        for (list<KeyFrame*>::const_iterator it = mlpLoopKeyFrameQueue.begin(); it != mlpLoopKeyFrameQueue.end();)
        {
            KeyFrame* pKFi = *it;
            if (pKFi->GetMap() == mpMapToReset)
            {
                it = mlpLoopKeyFrameQueue.erase(it);
            }
            else
                ++it;
        }

        mLastLoopKFid = mpAtlas->GetLastInitKFid();  //TODO old variable, it is not use in the new algorithm
        mbResetActiveMapRequested = false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF)
{
    Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

    const bool bImuInit = pActiveMap->isImuInitialized();

    if (!bImuInit)
        Optimizer::GlobalBundleAdjustemnt(pActiveMap, 10, &mbStopGBA, nLoopKF, false);
    else
        Optimizer::FullInertialBA(pActiveMap, 7, false, nLoopKF, &mbStopGBA);

    int idx = mnFullBAIdx;

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if (idx != mnFullBAIdx)
            return;

        if (!bImuInit && pActiveMap->isImuInitialized())
            return;

        if (!mbStopGBA)
        {
            Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
            Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(), pActiveMap->mvpKeyFrameOrigins.end());

            while (!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                Sophus::SE3f Twc = pKF->GetPoseInverse();
                for (set<KeyFrame*>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++)
                {
                    KeyFrame* pChild = *sit;
                    if (!pChild || pChild->isBad())
                        continue;

                    if (pChild->mnBAGlobalForKF != nLoopKF)
                    {
                        Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                        pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;  //*Tcorc*pKF->mTcwGBA;

                        Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                        if (pChild->isVelocitySet())
                        {
                            pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                        }
                        else
                            Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);

                        pChild->mBiasGBA = pChild->GetImuBias();

                        pChild->mnBAGlobalForKF = nLoopKF;
                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);

                if (pKF->bImu)
                {
                    pKF->mVwbBefGBA = pKF->GetVelocity();
                    pKF->SetVelocity(pKF->mVwbGBA);
                    pKF->SetNewBias(pKF->mBiasGBA);
                }

                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = pActiveMap->GetAllMapPoints();

            for (size_t i = 0; i < vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if (pMP->isBad())
                    continue;

                if (pMP->mnBAGlobalForKF == nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if (pRefKF->mnBAGlobalForKF != nLoopKF)
                        continue;

                    Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                    // Backproject using corrected camera
                    pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
                }
            }

            pActiveMap->InformNewBigChange();
            pActiveMap->IncreaseChangeIndex();

            mpLocalMapper->Release();

            Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

}  // namespace ORB_SLAM3
