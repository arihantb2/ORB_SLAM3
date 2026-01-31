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

#include "System.h"
#include "Verbose.h"

#include <openssl/md5.h>
#include <pangolin/pangolin.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <chrono>
#include <thread>
#include "Atlas.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "Settings.h"
#include "Tracking.h"
#include "Viewer.h"

namespace ORB_SLAM3
{

std::atomic<Verbose::eLevel> Verbose::th{Verbose::VERBOSITY_NORMAL};
std::mutex Verbose::cout_mutex;

System::System(const string& strVocFile, const string& strSettingsFile, const eSensor sensor, const bool bUseViewer,
               const bool bTurnOffLC)
    : mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbResetActiveMap(false), mbShutDown(false)
{
    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

    // Output welcome message
    Verbose::Print(Verbose::VERBOSITY_QUIET)
        << endl
        << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and "
           "Juan D. Tardós, University of Zaragoza."
        << endl
        << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of "
           "Zaragoza."
        << endl
        << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
        << "This is free software, and you are welcome to redistribute it" << endl
        << "under certain conditions. See LICENSE.txt." << endl
        << endl;

    Verbose::Print(Verbose::VERBOSITY_QUIET) << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET) << "Monocular" << endl;
    }
    else if (mSensor == STEREO)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET) << "Stereo" << endl;
    }
    else if (mSensor == IMU_MONOCULAR)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET) << "Monocular-Inertial" << endl;
    }
    else if (mSensor == IMU_STEREO)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET) << "Stereo-Inertial" << endl;
    }

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if (!node.empty() && node.isString() && node.string() == "1.0")
    {
        settings_ = new Settings(strSettingsFile, mSensor);

        Verbose::Print(Verbose::VERBOSITY_QUIET) << (*settings_) << endl;
    }
    else
    {
        throw std::runtime_error("Settings file version is not supported");
    }

    Verbose::Print(Verbose::VERBOSITY_QUIET) << "Loop Closing status: " << (!bTurnOffLC ? "ON" : "OFF") << endl;

    node = fsSettings["newMaps"];
    bool newMaps = true;
    if (!node.empty())
    {
        newMaps = (node.operator int()) == 1;
    }
    Verbose::Print(Verbose::VERBOSITY_QUIET) << "Atlas new maps status: " << (newMaps ? "ON" : "OFF") << endl;

    mStrVocabularyFilePath = strVocFile;

    //Load ORB Vocabulary
    Verbose::Print(Verbose::VERBOSITY_QUIET) << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    Verbose::Print(Verbose::VERBOSITY_QUIET) << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Atlas
    Verbose::Print(Verbose::VERBOSITY_QUIET) << "Initialization of Atlas from scratch " << endl;
    mpAtlas = new Atlas(0);

    const bool monocular = mSensor == MONOCULAR || mSensor == IMU_MONOCULAR;
    const bool inertial = mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO;

    if (inertial)
    {
        mpAtlas->SetInertialSensor();
    }

    //Create Drawers. These are used by the Viewer
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    //Initialize the Tracking thread
    mpTracker = new Tracking(this, mpVocabulary, mpMapDrawer, mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor,
                             settings_, newMaps);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, monocular, inertial);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
    if (settings_)
    {
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    }
    else
    {
        mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    }
    if (mpLocalMapper->mThFarPoints != 0)
    {
        Verbose::Print(Verbose::VERBOSITY_QUIET)
            << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
    {
        mpLocalMapper->mbFarPoints = false;
    }

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR,
                                   !bTurnOffLC);  // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    //Initialize the Viewer thread and launch
    if (bUseViewer)
    {
        mpViewer = new Viewer(this, mpMapDrawer, mpTracker, strSettingsFile, settings_);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
    }
}

Sophus::SE3f System::TrackStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp,
                                 const vector<IMU::Point>& vImuMeas)
{
    if (mSensor != STEREO && mSensor != IMU_STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if (settings_ && settings_->needToRectify())
    {
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if (settings_ && settings_->needToResize())
    {
        cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
        cv::resize(imRight, imRightToFeed, settings_->newImSize());
    }
    else
    {
        imLeftToFeed = imLeft.clone();
        imRightToFeed = imRight.clone();
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if (mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
    {
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
        {
            mpTracker->GrabImuData(vImuMeas[i_imu]);
        }
    }

    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    mDetectedKeyPoints = mpTracker->mCurrentFrame.mvKeys;
    mTrackedOutliers = mpTracker->mCurrentFrame.mvbOutlier;

    return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat& im, const double& timestamp, const vector<IMU::Point>& vImuMeas)
{

    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbShutDown)
        {
            return Sophus::SE3f();
        }
    }

    if (mSensor != MONOCULAR && mSensor != IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial."
             << endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    if (settings_ && settings_->needToResize())
    {
        cv::Mat resizedIm;
        cv::resize(im, resizedIm, settings_->newImSize());
        imToFeed = resizedIm;
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if (mbResetActiveMap)
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL) << "SYSTEM-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        for (size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
        {
            mpTracker->GrabImuData(vImuMeas[i_imu]);
        }
    }

    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    mDetectedKeyPoints = mpTracker->mCurrentFrame.mvKeys;
    mTrackedOutliers = mpTracker->mCurrentFrame.mvbOutlier;

    return Tcw;
}

bool System::MapChanged()
{
    static int n = 0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if (n < curn)
    {
        n = curn;
        return true;
    }
    else
    {
        return false;
    }
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbShutDown = true;
    }

    Verbose::Print(Verbose::VERBOSITY_NORMAL) << "Shutdown" << endl;

    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if (mpViewer)
    {
        mpViewer->RequestFinish();
    }

    const auto current_id = std::this_thread::get_id();
    const bool local_thread = mptLocalMapping && mptLocalMapping->get_id() == current_id;
    const bool loop_thread = mptLoopClosing && mptLoopClosing->get_id() == current_id;
    const bool viewer_thread = mptViewer && mptViewer->get_id() == current_id;

    while (mpLocalMapper && !local_thread && !mpLocalMapper->isFinished())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    while (mpLoopCloser && !loop_thread && !mpLoopCloser->isFinished())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    while (mpViewer && !viewer_thread && !mpViewer->isFinished())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (mptLocalMapping && mptLocalMapping->joinable() && mptLocalMapping->get_id() != current_id)
    {
        mptLocalMapping->join();
    }

    if (mptLoopClosing && mptLoopClosing->joinable() && mptLoopClosing->get_id() != current_id)
    {
        mptLoopClosing->join();
    }

    if (mptViewer && mptViewer->joinable() && mptViewer->get_id() != current_id)
    {
        mptViewer->join();
    }
}

bool System::isShutDown()
{
    unique_lock<mutex> lock(mMutexReset);
    return mbShutDown;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

vector<cv::KeyPoint> System::GetDetectedKeyPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mDetectedKeyPoints;
}

vector<cv::KeyPoint> System::GetDetectedKeyPointsRight()
{
    unique_lock<mutex> lock(mMutexState);
    return {};
}

vector<cv::KeyPoint> System::GetInlierKeyPoints()
{
    unique_lock<mutex> lock(mMutexState);
    vector<cv::KeyPoint> out;
    const size_t n = mDetectedKeyPoints.size();
    if (mTrackedMapPoints.size() != n || mTrackedOutliers.size() != n)
    {
        return out;
    }
    out.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        if (mTrackedMapPoints[i] && !mTrackedOutliers[i])
        {
            out.push_back(mDetectedKeyPoints[i]);
        }
    }
    return out;
}

vector<cv::KeyPoint> System::GetInlierKeyPointsRight()
{
    unique_lock<mutex> lock(mMutexState);
    return {};
}

vector<cv::KeyPoint> System::GetOutlierKeyPoints()
{
    unique_lock<mutex> lock(mMutexState);
    vector<cv::KeyPoint> out;
    const size_t n = mDetectedKeyPoints.size();
    if (mTrackedMapPoints.size() != n || mTrackedOutliers.size() != n)
    {
        return out;
    }
    out.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        if (mTrackedMapPoints[i] && mTrackedOutliers[i])
        {
            out.push_back(mDetectedKeyPoints[i]);
        }
    }
    return out;
}

vector<cv::KeyPoint> System::GetOutlierKeyPointsRight()
{
    unique_lock<mutex> lock(mMutexState);
    return {};
}

vector<Sophus::SE3f> System::GetKeyframeTrajectory()
{
    unique_lock<mutex> lock(mMutexState);
    vector<Sophus::SE3f> trajectory;
    auto keyframes = GetKeyFrames();
    trajectory.reserve(keyframes.size());
    for (KeyFrame* pKF : keyframes)
    {
        if (!pKF || pKF->isBad())
        {
            continue;
        }
        trajectory.push_back(pKF->GetPoseInverse());
    }
    return trajectory;
}

Tracking* System::GetTracker() const
{
    return mpTracker;
}

int System::GetLastBigChangeIdx()
{
    return mpAtlas->GetLastBigChangeIdx();
}

vector<KeyFrame*> System::GetKeyFrames()
{
    unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    return mpAtlas->GetAllKeyFrames();
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    if ((aux > 0.) && mpAtlas->isImuInitialized())
    {
        return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
    }
    else
    {
        return 0.f;
    }
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
    {
        return false;
    }
    else
    {
        if (mpTracker->mState == Tracking::LOST)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool System::isFinished()
{
    return (GetTimeFromIMUInit() > 0.1);
}

void System::ChangeDataset()
{
    if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }
}

float System::GetImageScale()
{
    return mpTracker->GetImageScale();
}

}  // namespace ORB_SLAM3
