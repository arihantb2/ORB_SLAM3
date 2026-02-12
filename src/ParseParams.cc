#include "Tracking.h"

#include "Atlas.h"
#include "System.h"

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

}  // namespace ORB_SLAM3