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

#include "Frame.h"
#include "bow/IBowVocabulary.h"
#include "bow/BowTypes.h"

#include "CameraModels/GeometricCamera.h"

#include <mutex>

namespace ORB_SLAM3
{

class Map;
class MapPoint;
class Frame;

class GeometricCamera;

class KeyFrame
{
public:
    // --- Public member variables (thread-safe or immutable) ---
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;
    const double mTimeStamp;

    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;
    long unsigned int mnNumberOfOpt;

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

    Sophus::SE3f mTcwGBA;
    Sophus::SE3f mTcwBefGBA;
    Eigen::Vector3f mVwbBefGBA;
    long unsigned int mnBAGlobalForKF;

    Sophus::SE3f mTcwMerge;
    Sophus::SE3f mTcwBefMerge;
    Sophus::SE3f mTwcBefMerge;
    Eigen::Vector3f mVwbBefMerge;
    long unsigned int mnMergeCorrectedForKF;
    long unsigned int mnMergeForKF;
    float mfScaleMerge;
    long unsigned int mnBALocalForMerge;

    float mfScale;

    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    cv::Mat mDistCoef;
    const int N;

    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight;
    const std::vector<float> mvDepth;
    const cv::Mat mDescriptors, mDescriptorsRight;
    const std::vector<int> vDescIndex;

    BowVector mBowVec;
    FeatureVector mFeatVec;

    Sophus::SE3f mTcp;

    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;

    unsigned int mnOriginMapId;
    std::string mNameFile;
    int mnDataset;

    GeometricCamera* mpCamera;
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;
    const std::vector<cv::KeyPoint> mvKeysRight;
    const int NLeft;

    // Pose prior
    std::optional<Sophus::SE3f> mPosePrior;

    // --- Public member functions ---
    KeyFrame();
    KeyFrame(Frame& F, Map* pMap);

    void SetPose(const Sophus::SE3f& Tcw);
    void SetVelocity(const Eigen::Vector3f& Vw_);
    Sophus::SE3f GetPose();
    Sophus::SE3f GetPoseInverse();
    Eigen::Vector3f GetCameraCenter();
    Eigen::Matrix3f GetRotation();
    Eigen::Vector3f GetTranslation();
    Eigen::Vector3f GetVelocity();
    bool isVelocitySet();

    void ComputeBoW();

    void AddConnection(KeyFrame* pKF, const int& weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections(bool upParent = true);
    void UpdateBestCovisibles();
    std::set<KeyFrame*> GetConnectedKeyFrames();
    std::vector<KeyFrame*> GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int& N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int& w);
    int GetWeight(KeyFrame* pKF);

    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);
    void SetFirstConnection(bool bFirst);

    int GetNumberMPs();
    void AddMapPoint(MapPoint* pMP, const size_t& idx);
    void EraseMapPointMatch(const int& idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const int& idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int& minObs);
    MapPoint* GetMapPoint(const size_t& idx);

    std::vector<size_t> GetFeaturesInArea(const float& x, const float& y, const float& r,
                                          const bool bRight = false) const;
    bool UnprojectStereo(int i, Eigen::Vector3f& x3D);
    bool IsInImage(const float& x, const float& y) const;

    void SetNotErase();
    void SetErase();
    void SetBadFlag();
    bool isBad();

    float ComputeSceneMedianDepth(const int q);

    static bool weightComp(int a, int b) { return a > b; }
    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2) { return pKF1->mnId < pKF2->mnId; }

    Map* GetMap();
    void UpdateMap(Map* pMap);

    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f& kp, float& u, float& v);
    bool ProjectPointUnDistort(MapPoint* pMP, cv::Point2f& kp, float& u, float& v);

    bool hasPosePrior() const { return mPosePrior.has_value(); }

protected:
    // --- Protected member variables (mutex-protected) ---
    Sophus::SE3<float> mTcw;
    Eigen::Matrix3f mRcw;
    Sophus::SE3<float> mTwc;
    Eigen::Matrix3f mRwc;

    Eigen::Vector3f mVw;
    bool mbHasVelocity;

    std::vector<MapPoint*> mvpMapPoints;

    IBowVocabulary* mpORBvocabulary;

    std::vector<std::vector<std::vector<size_t>>> mGrid;

    std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;

    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;

    float mHalfBaseline;

    Map* mpMap;

    Eigen::Matrix3f mK_;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;
};

}  // namespace ORB_SLAM3

#endif  // KEYFRAME_H
