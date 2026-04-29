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
#include <chrono>
#include <thread>
#include "Atlas.h"
#include "LocalMapping.h"
#include "Settings.h"
#include "Tracking.h"
#include "bow/BowVocabularyFactory.h"

namespace ORB_SLAM3
{

std::atomic<Verbose::eLevel> Verbose::th{Verbose::VERBOSITY_NORMAL};
std::mutex Verbose::cout_mutex;
std::unique_ptr<std::ofstream> Verbose::log_file_;
std::atomic<bool> Verbose::console_enabled{false};

System::System(const std::string& strVocFile, const std::string& strConfigFile, const eSensor sensor,
               const CameraCalibrationInput& calib, const std::string& strLogFile, const bool bVerboseConsole,
               const bool bSynchronousLocalMapping)
    : mSensor(sensor), mbReset(false), mbResetActiveMap(false), mbShutDown(false)
{
    Verbose::SetLogFile(strLogFile);
    Verbose::SetConsole(bVerboseConsole);
    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_DEBUG);

    // Output welcome message
    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << std::endl
        << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and "
           "Juan D. Tardós, University of Zaragoza."
        << std::endl
        << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of "
           "Zaragoza."
        << std::endl
        << "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl
        << "This is free software, and you are welcome to redistribute it" << std::endl
        << "under certain conditions. See LICENSE.txt." << std::endl
        << std::endl;

    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Input sensor was set to: ";

    if (mSensor == MONOCULAR)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Monocular" << std::endl;
    }
    else if (mSensor == STEREO)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Stereo" << std::endl;
    }
    // Check algorithm config file
    cv::FileStorage fsSettings(strConfigFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        throw std::runtime_error("Failed to open algorithm config at: " + strConfigFile);
    }

    cv::FileNode node = fsSettings["File.version"];
    if (!node.empty() && node.isString() && node.string() == "1.0")
    {
        settings_ = new Settings(strConfigFile, mSensor, calib);
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << (*settings_) << std::endl;
    }
    else
    {
        throw std::runtime_error("Algorithm config file version is not supported");
    }

    node = fsSettings["newMaps"];
    bool newMaps = true;
    if (!node.empty())
    {
        newMaps = (node.operator int()) == 1;
    }
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Atlas new maps status: " << (newMaps ? "ON" : "OFF") << std::endl;

    mStrVocabularyFilePath = strVocFile;
    cv::FileNode vocabPathNode = fsSettings["Vocabulary.path"];
    if (!vocabPathNode.empty() && vocabPathNode.isString())
    {
        mStrVocabularyFilePath = vocabPathNode.string();
    }

    Verbose::Print(Verbose::VERBOSITY_DEBUG)
        << std::endl
        << "Loading vocabulary from: " << mStrVocabularyFilePath
        << ". This could take a while..." << std::endl;

    mpVocabulary = CreateBowVocabulary();
    bool bVocLoad = mpVocabulary->load(mStrVocabularyFilePath);
    if (!bVocLoad)
    {
        throw std::runtime_error("Could not load vocabulary from file: " + mStrVocabularyFilePath);
    }
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Vocabulary loaded!" << std::endl << std::endl;

    //Create the Atlas
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Initialization of Atlas from scratch " << std::endl;
    mpAtlas = new Atlas(0);

    const bool monocular = mSensor == MONOCULAR;

    //Initialize the Tracking thread
    mpTracker = new Tracking(this, mpVocabulary.get(), mpAtlas, strConfigFile, mSensor, settings_, newMaps);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, monocular, settings_);
    if (bSynchronousLocalMapping)
    {
        mpLocalMapper->SetSynchronousMode(true);
    }
    mptLocalMapping = new std::thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);

    mpLocalMapper->SetTracker(mpTracker);
}

TrackingResult System::TrackStereo(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp,
                                   const std::optional<Sophus::SE3f>& posePrior)
{
    if (mSensor != STEREO)
    {
        throw std::runtime_error("You called TrackStereo but input sensor was not set to Stereo.");
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
        std::unique_lock<std::mutex> lock(mMutexReset);
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

    TrackingResult tracking_result = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, posePrior);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;

    return tracking_result;
}

TrackingResult System::TrackMonocular(const cv::Mat& im, const double& timestamp,
                                      const std::optional<Sophus::SE3f>& posePrior)
{
    if (mSensor != MONOCULAR)
    {
        throw std::runtime_error("You called TrackMonocular but input sensor was not set to Monocular.");
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
        std::unique_lock<std::mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if (mbResetActiveMap)
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL) << "SYSTEM-> Reseting active map in monocular case" << std::endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    TrackingResult tracking_result = mpTracker->GrabImageMonocular(imToFeed, timestamp, posePrior);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;

    return tracking_result;
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
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

void System::Shutdown()
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbShutDown = true;
    }

    Verbose::Print(Verbose::VERBOSITY_NORMAL) << "Shutdown" << std::endl;

    mpLocalMapper->RequestFinish();

    const auto current_id = std::this_thread::get_id();
    const bool local_thread = mptLocalMapping && mptLocalMapping->get_id() == current_id;

    while (mpLocalMapper && !local_thread && !mpLocalMapper->isFinished())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (mptLocalMapping && mptLocalMapping->joinable() && mptLocalMapping->get_id() != current_id)
    {
        mptLocalMapping->join();
    }
}

bool System::isShutDown()
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    return mbShutDown;
}

int System::GetTrackingState()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackingState;
}

std::vector<Sophus::SE3f> System::GetKeyframeTrajectory()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    std::vector<Sophus::SE3f> trajectory;
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

std::vector<KeyFrame*> System::GetKeyFrames()
{
    std::unique_lock<std::mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    return mpAtlas->GetAllKeyFrames();
}

bool System::isLost()
{
    return mpTracker->mState == Tracking::LOST;
}

bool System::isFinished()
{
    return true;
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

void System::SetLocalMappingCallback(LocalMappingCallback cb)
{
    mpLocalMapper->SetCallback(std::move(cb));
}

}  // namespace ORB_SLAM3
