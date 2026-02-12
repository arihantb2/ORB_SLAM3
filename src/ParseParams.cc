#include "Tracking.h"

#include "Atlas.h"
#include "System.h"

#include "CameraModels/KannalaBrandt8.h"
#include "CameraModels/Metashape.h"
#include "CameraModels/Pinhole.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "ORBextractor.h"

namespace ORB_SLAM3
{

void Tracking::newParameterLoader(Settings* settings)
{
    mpCamera = settings->camera1();
    mpCamera = mpAtlas->AddCamera(mpCamera);

    if (settings->needToUndistort())
    {
        mDistCoef = settings->camera1DistortionCoef();
    }
    else
    {
        mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    }

    //TODO: missing image scaling and rectification
    mImageScale = 1.0f;

    mK = mpCamera->toK();
    mK_ = mpCamera->toK_();

    if ((mSensor == System::STEREO || mSensor == System::IMU_STEREO) &&
        settings->cameraType() == Settings::KannalaBrandt)
    {
        mpCamera2 = settings->camera2();
        mpCamera2 = mpAtlas->AddCamera(mpCamera2);

        mTlr = settings->Tlr();
    }

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        mbf = settings->bf();
        mThDepth = settings->b() * settings->thDepth();
    }

    mMinFrames = 0;
    mMaxFrames = settings->fps();
    mbRGB = settings->rgb();

    //ORB parameters
    int nFeatures = settings->nFeatures();
    int nInitFeatures = settings->nInitFeatures();
    int nLevels = settings->nLevels();
    int fIniThFAST = settings->initThFAST();
    int fMinThFAST = settings->minThFAST();
    float fScaleFactor = settings->scaleFactor();

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }
    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        mpIniORBextractor = new ORBextractor(nInitFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }
    mMonocularInitSearchWindowSize = settings->monocularInitSearchWindowSize();
    mMonocularInitMinKeypoints = settings->monocularInitMinKeypoints();
    mMonocularInitNNRatio = settings->monocularInitNNRatio();
    mMonocularInitMinMatches = settings->monocularInitMinMatches();
    mStereoInitMinKeypoints = settings->stereoInitMinKeypoints();
    mReferenceKeyframeNNRatio = settings->referenceKeyframeNNRatio();
    mReferenceKeyframeMinBoWMatches = settings->referenceKeyframeMinBoWMatches();
    mReferenceKeyframeMinOptimizedMapMatches = settings->referenceKeyframeMinOptimizedMapMatches();
    mMotionModelNNRatio = settings->motionModelNNRatio();
    mMotionModelProjectionSearchThStereo = settings->motionModelProjectionSearchThStereo();
    mMotionModelProjectionSearchThMono = settings->motionModelProjectionSearchThMono();
    mMotionModelMinInitialMatches = settings->motionModelMinInitialMatches();
    mMotionModelRetryProjectionSearchThStereo = settings->motionModelRetryProjectionSearchThStereo();
    mMotionModelRetryProjectionSearchThMono = settings->motionModelRetryProjectionSearchThMono();
    mMotionModelMinRetryMatches = settings->motionModelMinRetryMatches();
    mMotionModelMinOptimizedMapMatches = settings->motionModelMinOptimizedMapMatches();
    mLocalMapGenericMinInliers = settings->localMapGenericMinInliers();
    mLocalMapVisualMinInliers = settings->localMapVisualMinInliers();
    //IMU parameters
    Sophus::SE3f Tbc = settings->Tbc();
    mInsertKFsLost = settings->insertKFsWhenLost();
    mImuFreq = settings->imuFrequency();
    mImuPer = 0.001;  //1.0 / (double) mImuFreq;     //TODO: ESTO ESTA BIEN?
    float Ng = settings->noiseGyro();
    float Na = settings->noiseAcc();
    float Ngw = settings->gyroWalk();
    float Naw = settings->accWalk();

    const float sf = sqrt(mImuFreq);
    mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
}

void Tracking::oldParameterLoader(const std::string& strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if (!b_parse_cam)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG)
            << "*Error with the camera parameters in the config file*" << std::endl;
    }

    // Load ORB parameters
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if (!b_parse_orb)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "*Error with the ORB parameters in the config file*" << std::endl;
    }
    bool b_parse_imu = true;
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        b_parse_imu = ParseIMUParamFile(fSettings);
        if (!b_parse_imu)
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG)
                << "*Error with the IMU parameters in the config file*" << std::endl;
        }

        mnFramesToResetIMU = mMaxFrames;
    }

    cv::FileNode node = fSettings["MonocularInit.SearchWindowSize"];
    if (!node.empty() && node.isInt())
    {
        mMonocularInitSearchWindowSize = node.operator int();
    }
    else
    {
        mMonocularInitSearchWindowSize = 100;
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "[WARNING] MonocularInit.SearchWindowSize not found. Defaulting to 100." << std::endl;
    }

    node = fSettings["MonocularInit.MinKeypoints"];
    if (!node.empty() && node.isInt())
    {
        mMonocularInitMinKeypoints = node.operator int();
    }
    else
    {
        mMonocularInitMinKeypoints = 100;
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "[WARNING] MonocularInit.MinKeypoints not found. Defaulting to 100." << std::endl;
    }

    node = fSettings["MonocularInit.NNRatio"];
    if (!node.empty() && node.isReal())
    {
        mMonocularInitNNRatio = node.real();
    }
    else
    {
        mMonocularInitNNRatio = 0.9f;
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "[WARNING] MonocularInit.NNRatio not found. Defaulting to 0.9." << std::endl;
    }

    node = fSettings["MonocularInit.MinMatches"];
    if (!node.empty() && node.isInt())
    {
        mMonocularInitMinMatches = node.operator int();
    }
    else
    {
        mMonocularInitMinMatches = 100;
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "[WARNING] MonocularInit.MinMatches not found. Defaulting to 100." << std::endl;
    }

    node = fSettings["Tracking.StereoInit.MinKeypoints"];
    if (!node.empty() && node.isInt())
    {
        mStereoInitMinKeypoints = node.operator int();
    }
    node = fSettings["Tracking.ReferenceKeyframe.NNRatio"];
    if (!node.empty() && node.isReal())
    {
        mReferenceKeyframeNNRatio = node.real();
    }
    node = fSettings["Tracking.ReferenceKeyframe.MinBoWMatches"];
    if (!node.empty() && node.isInt())
    {
        mReferenceKeyframeMinBoWMatches = node.operator int();
    }
    node = fSettings["Tracking.ReferenceKeyframe.MinOptimizedMapMatches"];
    if (!node.empty() && node.isInt())
    {
        mReferenceKeyframeMinOptimizedMapMatches = node.operator int();
    }
    node = fSettings["Tracking.MotionModel.NNRatio"];
    if (!node.empty() && node.isReal())
    {
        mMotionModelNNRatio = node.real();
    }
    node = fSettings["Tracking.MotionModel.ProjectionSearchThStereo"];
    if (!node.empty() && node.isInt())
    {
        mMotionModelProjectionSearchThStereo = node.operator int();
    }
    node = fSettings["Tracking.MotionModel.ProjectionSearchThMono"];
    if (!node.empty() && node.isInt())
    {
        mMotionModelProjectionSearchThMono = node.operator int();
    }
    node = fSettings["Tracking.MotionModel.MinInitialMatches"];
    if (!node.empty() && node.isInt())
    {
        mMotionModelMinInitialMatches = node.operator int();
    }
    node = fSettings["Tracking.MotionModel.RetryProjectionSearchThStereo"];
    if (!node.empty() && node.isInt())
    {
        mMotionModelRetryProjectionSearchThStereo = node.operator int();
    }
    node = fSettings["Tracking.MotionModel.RetryProjectionSearchThMono"];
    if (!node.empty() && node.isInt())
    {
        mMotionModelRetryProjectionSearchThMono = node.operator int();
    }
    node = fSettings["Tracking.MotionModel.MinRetryMatches"];
    if (!node.empty() && node.isInt())
    {
        mMotionModelMinRetryMatches = node.operator int();
    }
    node = fSettings["Tracking.MotionModel.MinOptimizedMapMatches"];
    if (!node.empty() && node.isInt())
    {
        mMotionModelMinOptimizedMapMatches = node.operator int();
    }
    node = fSettings["Tracking.LocalMap.GenericMinInliers"];
    if (!node.empty() && node.isInt())
    {
        mLocalMapGenericMinInliers = node.operator int();
    }
    node = fSettings["Tracking.LocalMap.VisualMinInliers"];
    if (!node.empty() && node.isInt())
    {
        mLocalMapVisualMinInliers = node.operator int();
    }

    if (!b_parse_cam || !b_parse_orb || !b_parse_imu)
    {
        throw std::runtime_error("**ERROR in the config file, the format is not correct**");
    }
}

bool Tracking::ParseCamParamFile(cv::FileStorage& fSettings)
{
    mDistCoef = cv::Mat::zeros(4, 1, CV_32F);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << std::endl << "Camera Parameters: " << std::endl;
    bool b_miss_params = false;

    std::string sCameraName = fSettings["Camera.type"];
    if (sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if (!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if (!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if (!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if (!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if (!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if (!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if (!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if (!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if (!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        node = fSettings["Camera.imageScale"];
        if (!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if (b_miss_params)
        {
            return false;
        }

        if (mImageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx = fx * mImageScale;
            fy = fy * mImageScale;
            cx = cx * mImageScale;
            cy = cy * mImageScale;
        }

        std::vector<float> vCamCalib{fx, fy, cx, cy};

        mpCamera = new Pinhole(vCamCalib);

        mpCamera = mpAtlas->AddCamera(mpCamera);

        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Camera: Pinhole" << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Image scale: " << mImageScale << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- fx: " << fx << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- fy: " << fy << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- cx: " << cx << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- cy: " << cy << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k2: " << mDistCoef.at<float>(1) << std::endl;

        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if (mDistCoef.rows == 5)
        {
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k3: " << mDistCoef.at<float>(4) << std::endl;
        }
        mK = cv::Mat::eye(3, 3, CV_32F);
        mK.at<float>(0, 0) = fx;
        mK.at<float>(1, 1) = fy;
        mK.at<float>(0, 2) = cx;
        mK.at<float>(1, 2) = cy;

        mK_.setIdentity();
        mK_(0, 0) = fx;
        mK_(1, 1) = fy;
        mK_(0, 2) = cx;
        mK_(1, 2) = cy;
    }
    else if (sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;
        mImageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if (!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if (!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if (!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if (!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if (!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if (!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if (!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if (!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.imageScale"];
        if (!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if (!b_miss_params)
        {
            if (mImageScale != 1.f)
            {
                // K matrix parameters must be scaled.
                fx = fx * mImageScale;
                fy = fy * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
            }

            std::vector<float> vCamCalib{fx, fy, cx, cy, k1, k2, k3, k4};
            mpCamera = new KannalaBrandt8(vCamCalib);
            mpCamera = mpAtlas->AddCamera(mpCamera);
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Camera: Fisheye" << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Image scale: " << mImageScale << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- fx: " << fx << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- fy: " << fy << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- cx: " << cx << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- cy: " << cy << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k1: " << k1 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k2: " << k2 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k3: " << k3 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3, 3, CV_32F);
            mK.at<float>(0, 0) = fx;
            mK.at<float>(1, 1) = fy;
            mK.at<float>(0, 2) = cx;
            mK.at<float>(1, 2) = cy;

            mK_.setIdentity();
            mK_(0, 0) = fx;
            mK_(1, 1) = fy;
            mK_(0, 2) = cx;
            mK_(1, 2) = cy;
        }

        if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
        {
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if (!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if (!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if (!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if (!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if (!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if (!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if (!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if (!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL)
                    << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if (!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if (!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if (!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if (!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            cv::Mat cvTlr;
            if (!node.empty())
            {
                cvTlr = node.mat();
                if (cvTlr.rows != 3 || cvTlr.cols != 4)
                {
                    Verbose::Print(Verbose::VERBOSITY_NORMAL)
                        << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                Verbose::Print(Verbose::VERBOSITY_NORMAL) << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if (!b_miss_params)
            {
                if (mImageScale != 1.f)
                {
                    // K matrix parameters must be scaled.
                    fx = fx * mImageScale;
                    fy = fy * mImageScale;
                    cx = cx * mImageScale;
                    cy = cy * mImageScale;

                    leftLappingBegin = leftLappingBegin * mImageScale;
                    leftLappingEnd = leftLappingEnd * mImageScale;
                    rightLappingBegin = rightLappingBegin * mImageScale;
                    rightLappingEnd = rightLappingEnd * mImageScale;
                }

                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                std::vector<float> vCamCalib2{fx, fy, cx, cy, k1, k2, k3, k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);
                mpCamera2 = mpAtlas->AddCamera(mpCamera2);

                mTlr = Converter::toSophus(cvTlr);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                Verbose::Print(Verbose::VERBOSITY_DEBUG) << std::endl << "Camera2 Parameters:" << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Camera: Fisheye" << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Image scale: " << mImageScale << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- fx: " << fx << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- fy: " << fy << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- cx: " << cx << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- cy: " << cy << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k1: " << k1 << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k2: " << k2 << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k3: " << k3 << std::endl;
                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k4: " << k4 << std::endl;

                Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- mTlr: \n" << cvTlr << std::endl;

                Verbose::Print(Verbose::VERBOSITY_DEBUG)
                    << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if (b_miss_params)
        {
            return false;
        }
    }
    else if (sCameraName == "Metashape")
    {
        float f, cx, cy;
        float b1, b2;
        float k1, k2, k3, k4, p1, p2;
        mImageScale = 1.f;

        cv::FileNode node = fSettings["Camera.f"];
        if (!node.empty() && node.isReal())
        {
            f = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.f parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if (!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if (!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.b1"];
        if (!node.empty() && node.isReal())
        {
            b1 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.b1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.b2"];
        if (!node.empty() && node.isReal())
        {
            b2 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.b2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k1"];
        if (!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if (!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if (!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if (!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if (!node.empty() && node.isReal())
        {
            p1 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if (!node.empty() && node.isReal())
        {
            p2 = node.real();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        int width_px = 0;
        int height_px = 0;
        node = fSettings["Camera.width"];
        if (!node.empty() && node.isInt())
        {
            width_px = node.operator int();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.width parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.height"];
        if (!node.empty() && node.isInt())
        {
            height_px = node.operator int();
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.height parameter doesn't exist or is not an integer*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.imageScale"];
        if (!node.empty() && node.isReal())
        {
            mImageScale = node.real();
        }

        if (!b_miss_params)
        {
            if (mImageScale != 1.f)
            {
                f = f * mImageScale;
                cx = cx * mImageScale;
                cy = cy * mImageScale;
                b1 = b1 * mImageScale;
                b2 = b2 * mImageScale;
                width_px = static_cast<int>(width_px * mImageScale);
                height_px = static_cast<int>(height_px * mImageScale);
            }

            const float fx = f + b1;
            const float fy = f;
            const float skew = b2;

            const float cx_abs = cx + 0.5f * static_cast<float>(width_px);
            const float cy_abs = cy + 0.5f * static_cast<float>(height_px);

            std::vector<float> vCamCalib{fx, fy, cx_abs, cy_abs, k1, k2, k3, k4, p1, p2, skew};
            mpCamera = new Metashape(vCamCalib);
            mpCamera = mpAtlas->AddCamera(mpCamera);

            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Camera: Metashape" << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Image scale: " << mImageScale << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- f: " << f << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- b1: " << b1 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- b2: " << b2 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- cx: " << cx_abs << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- cy: " << cy_abs << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k1: " << k1 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k2: " << k2 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k3: " << k3 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- k4: " << k4 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- p1: " << p1 << std::endl;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- p2: " << p2 << std::endl;

            mK = cv::Mat::eye(3, 3, CV_32F);
            mK.at<float>(0, 0) = fx;
            mK.at<float>(1, 1) = fy;
            mK.at<float>(0, 1) = skew;
            mK.at<float>(0, 2) = cx_abs;
            mK.at<float>(1, 2) = cy_abs;

            mK_.setIdentity();
            mK_(0, 0) = fx;
            mK_(1, 1) = fy;
            mK_(0, 1) = skew;
            mK_(0, 2) = cx_abs;
            mK_(1, 2) = cy_abs;
        }

        if (b_miss_params)
        {
            return false;
        }
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL) << "*Not Supported Camera Sensor*" << std::endl;
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if (!node.empty() && node.isReal())
        {
            mbf = node.real();
            if (mImageScale != 1.f)
            {
                mbf *= mImageScale;
            }
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
    }

    float fps = fSettings["Camera.fps"];
    if (fps <= 1)
    {
        fps = 30;
    }
    // Max/Min Frames to insert keyframes
    mMinFrames = 0;
    mMaxFrames = fps;

    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- fps: " << fps << std::endl;

    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if (mbRGB)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- color order: RGB (ignored if grayscale)" << std::endl;
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- color order: BGR (ignored if grayscale)" << std::endl;
    }
    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if (!node.empty() && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf * mThDepth / fx;
            Verbose::Print(Verbose::VERBOSITY_DEBUG) << std::endl
                                                     << "Depth Threshold (Close/Far Points): " << mThDepth << std::endl;
        }
        else
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
    }

    if (b_miss_params)
    {
        return false;
    }

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage& fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;
    int nInitFeatures = 0;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if (!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBExtractor.nInitFeatures"];
    if (!node.empty() && node.isInt())
    {
        nInitFeatures = node.operator int();
    }
    else
    {
        nInitFeatures = static_cast<int>(2.5f * nFeatures);
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if (!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if (!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if (!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if (!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if (b_miss_params)
    {
        return false;
    }

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (mSensor == System::STEREO || mSensor == System::IMU_STEREO)
    {
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }
    if (mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        mpIniORBextractor = new ORBextractor(nInitFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << std::endl << "ORB Extractor Parameters: " << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Number of Features: " << nFeatures << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Scale Levels: " << nLevels << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Scale Factor: " << fScaleFactor << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Initial Fast Threshold: " << fIniThFAST << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "- Minimum Fast Threshold: " << fMinThFAST << std::endl;

    return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage& fSettings)
{
    bool b_miss_params = false;

    cv::Mat cvTbc;
    cv::FileNode node = fSettings["Tbc"];
    if (!node.empty())
    {
        cvTbc = node.mat();
        if (cvTbc.rows != 4 || cvTbc.cols != 4)
        {
            Verbose::Print(Verbose::VERBOSITY_NORMAL)
                << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL) << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Left camera to Imu Transform (Tbc): " << std::endl
                                             << cvTbc << std::endl;
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    Sophus::SE3f Tbc(eigTbc);

    node = fSettings["InsertKFsWhenLost"];
    mInsertKFsLost = true;
    if (!node.empty() && node.isInt())
    {
        mInsertKFsLost = (bool)node.operator int();
    }

    if (!mInsertKFsLost)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Do not insert keyframes when lost visual tracking " << std::endl;
    }
    float Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if (!node.empty() && node.isInt())
    {
        mImuFreq = node.operator int();
        mImuPer = 0.001;  //1.0 / (double) mImuFreq;
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if (!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if (!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if (!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if (!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        Verbose::Print(Verbose::VERBOSITY_NORMAL)
            << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.fastInit"];
    mFastInit = false;
    if (!node.empty())
    {
        mFastInit = static_cast<int>(fSettings["IMU.fastInit"]) != 0;
    }

    if (mFastInit)
    {
        Verbose::Print(Verbose::VERBOSITY_DEBUG) << "Fast IMU initialization. Acceleration is not checked \n";
    }
    if (b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(mImuFreq);
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "IMU frequency: " << mImuFreq << " Hz" << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << std::endl;
    Verbose::Print(Verbose::VERBOSITY_DEBUG) << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << std::endl;

    mpImuCalib = new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);

    return true;
}

}  // namespace ORB_SLAM3