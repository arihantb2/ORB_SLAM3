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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>
#include "Frame.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"

#include "CameraModels/GeometricCamera.h"

#include <mutex>

namespace ORB_SLAM3
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class GeometricCamera;

class KeyFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();
    KeyFrame(Frame& F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const Sophus::SE3f& Tcw);
    void SetVelocity(const Eigen::Vector3f& Vw_);

    Sophus::SE3f GetPose();

    Sophus::SE3f GetPoseInverse();

    Eigen::Vector3f GetCameraCenter();

    Eigen::Vector3f GetImuPosition();
    Eigen::Matrix3f GetImuRotation();
    Sophus::SE3f GetImuPose();
    Eigen::Matrix3f GetRotation();
    Eigen::Vector3f GetTranslation();
    Eigen::Vector3f GetVelocity();
    bool isVelocitySet();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int& weight);
    void EraseConnection(KeyFrame* pKF);

    void UpdateConnections(bool upParent = true);
    void UpdateBestCovisibles();
    std::set<KeyFrame*> GetConnectedKeyFrames();
    std::vector<KeyFrame*> GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int& N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int& w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);
    void SetFirstConnection(bool bFirst);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // Merge Edges
    void AddMergeEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetMergeEdges();

    // MapPoint observation functions
    int GetNumberMPs();
    void AddMapPoint(MapPoint* pMP, const size_t& idx);
    void EraseMapPointMatch(const int& idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const int& idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int& minObs);
    MapPoint* GetMapPoint(const size_t& idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float& x, const float& y, const float& r,
                                          const bool bRight = false) const;
    bool UnprojectStereo(int i, Eigen::Vector3f& x3D);

    // Image
    bool IsInImage(const float& x, const float& y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp(int a, int b) { return a > b; }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2) { return pKF1->mnId < pKF2->mnId; }

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void SetNewBias(const IMU::Bias& b);
    Eigen::Vector3f GetGyroBias();

    Eigen::Vector3f GetAccBias();

    IMU::Bias GetImuBias();

    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f& kp, float& u, float& v);
    bool ProjectPointUnDistort(MapPoint* pMP, cv::Point2f& kp, float& u, float& v);

    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    bool bImu;

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    //Number of optimizations by BA(amount of iterations in BA)
    long unsigned int mnNumberOfOpt;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnMergeQuery;
    int mnMergeWords;
    float mMergeScore;
    long unsigned int mnPlaceRecognitionQuery;
    int mnPlaceRecognitionWords;
    float mPlaceRecognitionScore;

    bool mbCurrentPlaceRecognition;

    // Variables used by loop closing
    Sophus::SE3f mTcwGBA;
    Sophus::SE3f mTcwBefGBA;
    Eigen::Vector3f mVwbGBA;
    Eigen::Vector3f mVwbBefGBA;
    IMU::Bias mBiasGBA;
    long unsigned int mnBAGlobalForKF;

    // Variables used by merging
    Sophus::SE3f mTcwMerge;
    Sophus::SE3f mTcwBefMerge;
    Sophus::SE3f mTwcBefMerge;
    Eigen::Vector3f mVwbMerge;
    Eigen::Vector3f mVwbBefMerge;
    IMU::Bias mBiasMerge;
    long unsigned int mnMergeCorrectedForKF;
    long unsigned int mnMergeForKF;
    float mfScaleMerge;
    long unsigned int mnBALocalForMerge;

    float mfScale;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    cv::Mat mDistCoef;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight;  // negative value for monocular points
    const std::vector<float> mvDepth;   // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    Sophus::SE3f mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;

    // Preintegrated IMU measurements from previous keyframe
    KeyFrame* mPrevKF;
    KeyFrame* mNextKF;

    IMU::Preintegrated* mpImuPreintegrated;
    IMU::Calib mImuCalib;

    unsigned int mnOriginMapId;

    std::string mNameFile;

    int mnDataset;

    std::vector<KeyFrame*> mvpLoopCandKFs;
    std::vector<KeyFrame*> mvpMergeCandKFs;

    //bool mbHasHessian;
    //cv::Mat mHessianPose;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    // sophus poses
    Sophus::SE3<float> mTcw;
    Eigen::Matrix3f mRcw;
    Sophus::SE3<float> mTwc;
    Eigen::Matrix3f mRwc;

    // IMU position
    Eigen::Vector3f mOwb;
    // Velocity (Only used for inertial SLAM)
    Eigen::Vector3f mVw;
    bool mbHasVelocity;

    // Imu bias
    IMU::Bias mImuBias;

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector<std::vector<std::vector<size_t>>> mGrid;

    std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;
    std::set<KeyFrame*> mspMergeEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline;  // Only for visualization

    Map* mpMap;

    // Calibration
    Eigen::Matrix3f mK_;

    // Mutex
    std::mutex mMutexPose;  // for pose, velocity and biases
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;

public:
    GeometricCamera* mpCamera;

    //Indexes of stereo observations correspondences
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    //KeyPoints in the right image (for stereo coordinates)
    const std::vector<cv::KeyPoint> mvKeysRight;

    const int NLeft;
};

}  // namespace ORB_SLAM3

#endif  // KEYFRAME_H
