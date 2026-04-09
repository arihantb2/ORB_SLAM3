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

#ifndef FRAME_H
#define FRAME_H

#include "bow/BowTypes.h"
#include "bow/IBowVocabulary.h"

#include "Settings.h"

#include <optional>
#include <vector>

#include <opencv2/opencv.hpp>

#include "sophus/se3.hpp"

namespace ORB_SLAM3
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class GeometricCamera;
class FeatureExtractor;

class Frame
{
public:
    // --- Public member variables ---
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IBowVocabulary* mpORBvocabulary;
    FeatureExtractor* mpFeatureExtractorLeft;
    FeatureExtractor* mpFeatureExtractorRight;

    double mTimeStamp;

    cv::Mat mK;
    Eigen::Matrix3f mK_;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    float mbf;
    float mb;
    float mThDepth;

    int N;

    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<int> vDescIndex;

    std::vector<MapPoint*> mvpMapPoints;
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    BowVector mBowVec;
    FeatureVector mFeatVec;

    cv::Mat mDescriptors;
    cv::Mat mDescriptorsRight;

    std::vector<bool> mvbOutlier;
    int mnCloseMPs;

    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    KeyFrame* mpLastKeyFrame;
    Frame* mpPrevFrame;

    static long unsigned int nNextId;
    long unsigned int mnId;

    KeyFrame* mpReferenceKF;

    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvInvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;
    static bool mbInitialComputations;

    std::map<long unsigned int, cv::Point2f> mmProjectPoints;
    std::map<long unsigned int, cv::Point2f> mmMatchedInImage;

    // Debug: 2D correspondences for visualization (current frame vs various sources).
    // (last_frame_point, current_frame_point)
    std::vector<std::pair<cv::Point2f, cv::Point2f>> mDebugFrame2FrameMatches;
    // (ref_keyframe_point, current_frame_point)
    std::vector<std::pair<cv::Point2f, cv::Point2f>> mDebugFrame2RefKfMatches;
    // (local_map_point, current_frame_point)
    std::vector<std::pair<cv::Point2f, cv::Point2f>> mDebugFrame2LocalMapMatches;

    std::string mNameFile;
    int mnDataset;

    GeometricCamera* mpCamera;
    int Nleft;
    std::vector<int> mvLeftToRightMatch;
    std::vector<int> mvRightToLeftMatch;

    // Pose prior
    std::optional<Sophus::SE3f> mPosePrior;

    // Estimated pose
    Sophus::SE3<float> mTcw;
    Eigen::Matrix<float, 3, 3> mRwc;
    Eigen::Matrix<float, 3, 1> mOw;
    Eigen::Matrix<float, 3, 3> mRcw;
    Eigen::Matrix<float, 3, 1> mtcw;
    bool mbHasPose;

    Eigen::Vector3f mVw;
    bool mbHasVelocity;

    // --- Public member functions ---
    Frame();
    Frame(const Frame& frame);
    Frame(const cv::Mat& imLeft, const cv::Mat& imRight, const double& timeStamp, FeatureExtractor* extractorLeft,
          FeatureExtractor* extractorRight, IBowVocabulary* voc, cv::Mat& K, cv::Mat& distCoef, const float& bf,
          const float& thDepth, GeometricCamera* pCamera, Frame* pPrevF = nullptr);
    Frame(const cv::Mat& imGray, const double& timeStamp, FeatureExtractor* extractor, IBowVocabulary* voc,
          GeometricCamera* pCamera, cv::Mat& distCoef, const float& bf, const float& thDepth, Frame* pPrevF = nullptr);

    ~Frame();

    void ExtractFeatures(bool left, const cv::Mat& im, const int x0, const int x1);
    void ComputeBoW();

    void SetPose(const Sophus::SE3<float>& Tcw);
    void SetVelocity(const Eigen::Vector3f& Vw);
    Eigen::Vector3f GetVelocity() const;

    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f& kp, float& u, float& v);
    Eigen::Vector3f inRefCoordinates(const Eigen::Vector3f& pCw);

    bool PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY);
    std::vector<size_t> GetFeaturesInArea(const float& x, const float& y, const float& r, const int minLevel = -1,
                                          const int maxLevel = -1, const bool bRight = false) const;

    void ComputeStereoMatches();
    bool UnprojectStereo(const int& i, Eigen::Vector3f& x3D);

    bool isSet() const;

    void setPosePrior(const Sophus::SE3f& posePrior) { mPosePrior = posePrior; }
    bool hasPosePrior() const { return mPosePrior.has_value(); }

    void UpdatePoseMatrices();
    inline Eigen::Vector3f GetCameraCenter() { return mOw; }
    inline Eigen::Matrix3f GetRotationInverse() { return mRwc; }
    inline Sophus::SE3<float> GetPose() const { return mTcw; }
    inline Eigen::Matrix3f GetRwc() const { return mRwc; }
    inline Eigen::Vector3f GetOw() const { return mOw; }
    inline bool HasPose() const { return mbHasPose; }
    inline bool HasVelocity() const { return mbHasVelocity; }

private:
    // --- Private member variables ---
    bool mbIsSet;

    // --- Private member functions ---
    void UndistortKeyPoints();
    void ComputeImageBounds(const cv::Mat& imLeft);
    void AssignFeaturesToGrid();
};

}  // namespace ORB_SLAM3

#endif  // FRAME_H
