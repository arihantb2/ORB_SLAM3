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

#ifndef TRACKING_H
#define TRACKING_H

#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "Frame.h"
#include "bow/IBowVocabulary.h"

namespace ORB_SLAM3
{

class Atlas;
class LocalMapping;
class System;
class Settings;
class KeyFrame;
class Map;
class MapPoint;
class FeatureExtractor;
class GeometricCamera;

// ---------------------------------------------------------------------------
// Helper structs for rich tracking introspection data
// ---------------------------------------------------------------------------

// A single feature match between two images, with keypoint indices for
// cross-referencing and an inlier flag set after pose optimization.
struct MatchedKeypoint
{
    int current_kp_idx = -1;  // Index into current Frame's mvKeysUn
    int source_kp_idx = -1;   // Index into source Frame's/KF's mvKeysUn; -1 if unknown
    cv::KeyPoint current_kp;  // Keypoint in current frame (undistorted)
    cv::KeyPoint source_kp;   // Keypoint in source frame/KF (undistorted)
    bool is_inlier = false;   // True if this match survives pose optimization
};

// A map point that is currently observed in the current frame, with its
// 3D position expressed in both world and camera frames.
struct MapPointObservation
{
    int keypoint_idx = -1;           // Index into current Frame's mvKeysUn
    cv::KeyPoint keypoint;           // Keypoint in current frame (undistorted)
    Eigen::Vector3f pos_world;       // MapPoint world position (GetWorldPos())
    Eigen::Vector3f pos_camera;      // pos_world transformed to camera frame: Tcw * pos_world
    unsigned long map_point_id = 0;  // MapPoint::mnId — unique across the map
    bool is_inlier = true;           // False if marked as outlier by pose optimization
};

// All ORB keypoints detected in the current frame, plus stereo matching
// results for stereo-pinhole mode.
struct FrameKeypointData
{
    // Left/mono keypoints in the undistorted image plane (Frame::mvKeysUn).
    // For stereo-pinhole the image is already rectified, so these are also
    // undistorted. For monocular the standard undistortion is applied.
    std::vector<cv::KeyPoint> left_keypoints;

    // Right-image keypoints (Frame::mvKeysRight). Empty for monocular.
    std::vector<cv::KeyPoint> right_keypoints;

    // Per-left-keypoint stereo matching results (stereo only, else empty).
    // Indexed the same as left_keypoints (i.e. index i corresponds to left_keypoints[i]).
    // stereo_right_u[i] = u-coordinate of the match on the right image (-1 = no match).
    // stereo_depth[i]   = depth in metres derived from the stereo baseline (-1 = no match).
    // Source: Frame::mvuRight and Frame::mvDepth respectively.
    std::vector<float> stereo_right_u;
    std::vector<float> stereo_depth;

    // Convenience list of matched stereo pairs for visualisation.
    // Each entry is (left_kp_index, reconstructed right-image KeyPoint).
    // The right KeyPoint has pt.x = stereo_right_u[i], pt.y = left_keypoints[i].pt.y,
    // and the same octave/response/size as the left keypoint.
    std::vector<std::pair<int, cv::KeyPoint>> stereo_match_pairs;
};

// A keypoint that has valid stereo depth but is NOT currently matched to any
// tracked map point. If this frame is promoted to a KeyFrame, LocalMapping
// will create a new MapPoint for each of these. 3D positions are computed
// inline via Frame::UnprojectStereo(). Empty for monocular.
struct NewMapPointCandidate
{
    int keypoint_idx = -1;       // Index into current Frame's mvKeysUn
    cv::KeyPoint left_kp;        // Left undistorted keypoint
    cv::KeyPoint right_kp;       // Reconstructed right keypoint (pt.x = mvuRight[i])
    float depth = -1.f;          // Stereo depth in metres (Frame::mvDepth[i])
    Eigen::Vector3f pos_world;   // 3D world position (from UnprojectStereo)
    Eigen::Vector3f pos_camera;  // 3D camera-frame position: Tcw * pos_world
};

// ---------------------------------------------------------------------------

struct MotionModelTrackingResult
{
    bool success = false;
    bool retry = false;

    // Initial pose
    Sophus::SE3f initial_pose;

    // Number of matches before optimization
    int num_matches = 0;
    int num_matches_retry = 0;

    // Matches before optimization
    std::vector<std::pair<cv::KeyPoint, cv::KeyPoint>> keypoints_matches;

    // Number of matches after optimization
    int num_matches_optimized = 0;

    // Inliers after optimization
    std::vector<cv::KeyPoint> keypoints_inliers_optimized;
    // Outliers after optimization
    std::vector<cv::KeyPoint> keypoints_outliers_optimized;
    // Matches after optimization
    std::vector<std::pair<cv::KeyPoint, cv::KeyPoint>> keypoints_matches_optimized;

    // Rich match data with keypoint indices (supersedes the pair-based fields above,
    // which are also populated for backward compatibility).
    // frame_matches: all current↔last-frame matches after the final SearchByProjection,
    //   before pose optimization. is_inlier is false at this point.
    // frame_matches_optimized: subset where is_inlier == true after DiscardOutliersAndCountInliers.
    std::vector<MatchedKeypoint> frame_matches;
    std::vector<MatchedKeypoint> frame_matches_optimized;

    // Optimized Pose
    Sophus::SE3f pose;
};

struct RefKeyFrameTrackingResult
{
    bool success = false;

    // Number of matches before optimization
    int num_matches = 0;

    // Matches before optimization
    std::vector<std::pair<cv::KeyPoint, cv::KeyPoint>> keypoints_matches;

    // Number of matches after optimization
    int num_matches_optimized = 0;

    // Inliers after optimization
    std::vector<cv::KeyPoint> keypoints_inliers_optimized;
    // Outliers after optimization
    std::vector<cv::KeyPoint> keypoints_outliers_optimized;
    // Matches after optimization
    std::vector<std::pair<cv::KeyPoint, cv::KeyPoint>> keypoints_matches_optimized;

    // Rich match data with keypoint indices.
    // kf_matches: all current↔ref-KF matches from SearchByBoW, before optimization.
    // kf_matches_optimized: inlier subset after DiscardOutliersAndCountInliers.
    std::vector<MatchedKeypoint> kf_matches;
    std::vector<MatchedKeypoint> kf_matches_optimized;

    // Optimized Pose
    Sophus::SE3f pose;
};

struct LocalMapTrackingResult
{
    bool success = false;
    int num_matches = 0;

    // Inliers
    std::vector<cv::KeyPoint> keypoints_inliers;

    // Outliers
    std::vector<cv::KeyPoint> keypoints_outliers;

    // Matches
    std::vector<std::pair<cv::KeyPoint, cv::KeyPoint>> keypoints_matches;

    // Full map point observations split by inlier/outlier status.
    // Populated inside TrackLocalMap() during the inlier-counting loop.
    std::vector<MapPointObservation> inlier_observations;
    std::vector<MapPointObservation> outlier_observations;

    // Pose
    Sophus::SE3f pose;
};

struct TrackingResult
{
    bool ref_keyframe_tracking_primary = false;
    bool motion_model_tracking_primary = false;
    bool ref_keyframe_tracking_fallback = false;

    bool success = false;

    RefKeyFrameTrackingResult ref_key_frame_result;
    MotionModelTrackingResult motion_model_result;
    LocalMapTrackingResult local_map_result;

    // (1) All ORB keypoints detected in this frame, plus stereo matching results.
    //     Populated at the start of Track() from mCurrentFrame, before any tracking.
    FrameKeypointData keypoint_data;

    // (3) All map points that are inlier observations in this frame after the full
    //     tracking pipeline (TrackLocalMap + pose optimization). Superset of
    //     local_map_result.inlier_observations because it is populated at the very
    //     end of Track() after UpdateAfterTracking().
    std::vector<MapPointObservation> all_tracked_map_points;

    // (4) Stereo keypoints with valid depth that are NOT matched to any existing map
    //     point after tracking. These are candidates that LocalMapping will turn into
    //     new MapPoints. Always empty for monocular.
    std::vector<NewMapPointCandidate> new_map_point_candidates;

    // (5) Input images as received by the tracking layer — already grayscale, rectified
    //     (stereo pinhole) or undistorted (monocular).
    //     Together with the keypoint and map-point data above, these make TrackingResult
    //     fully self-contained for offline visualisation.
    //
    //     image_left  — left image (stereo) or the single image (monocular). Never empty.
    //     image_right — right image (stereo only). Empty (default-constructed cv::Mat)
    //                   for monocular.
    //
    //     Both images are cloned (own their data); the originals are temporaries and go
    //     out of scope after GrabImage* returns.
    cv::Mat image_left;
    cv::Mat image_right;

    Sophus::SE3f pose;
};

class Tracking
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System* pSys, IBowVocabulary* pVoc, Atlas* pAtlas, const std::string& strSettingPath, const int sensor,
             Settings* settings, const bool newMaps);

    ~Tracking();

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    TrackingResult GrabImageStereo(const cv::Mat& imRectLeft, const cv::Mat& imRectRight, const double& timestamp,
                                   const std::optional<Sophus::SE3f>& posePrior = std::nullopt);
    TrackingResult GrabImageMonocular(const cv::Mat& im, const double& timestamp,
                                      const std::optional<Sophus::SE3f>& posePrior = std::nullopt);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    bool isLastFrameKeyframe();

    KeyFrame* GetLastKeyFrame() { return mpLastKeyFrame; }

    void CreateMapInAtlas();

    int GetMatchesInliers();

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    std::vector<MapPoint*> GetLocalMapMPS();

    // Tracking states
    enum eTrackingState
    {
        NO_IMAGES_YET = 0,
        NOT_INITIALIZED = 1,
        OK = 2,
        LOST = 3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Intialization and Tracking Parameters
    int mMonocularInitMinKeypoints = 100;
    float mMonocularInitNNRatio = 0.9f;
    int mMonocularInitSearchWindowSize = 100;
    int mMonocularInitMinMatches = 100;
    int mStereoInitMinKeypoints = 500;
    float mReferenceKeyframeNNRatio = 0.7f;
    int mReferenceKeyframeMinBoWMatches = 15;
    int mReferenceKeyframeMinOptimizedMapMatches = 10;
    // In SIFT mode, BoW-based reference-keyframe tracking is disabled.
    bool mUseBoWReferenceKeyframeTracking = true;
    float mMotionModelNNRatio = 0.9f;
    int mMotionModelProjectionSearchTh = 30;
    int mMotionModelMinInitialMatches = 20;
    int mMotionModelRetryProjectionSearchTh = 60;
    int mMotionModelMinRetryMatches = 20;
    int mMotionModelMinOptimizedMapMatches = 10;
    int mLocalMapGenericMinInliers = 10;
    int mLocalMapVisualMinInliers = 30;

protected:
    // Main tracking function. It is independent of the input sensor.
    TrackingResult Track();
    void TrackStereo(TrackingResult& tracking_result);
    void TrackFrame(TrackingResult& tracking_result);

    // Stereo Initialization
    void StereoInitialization();

    // Monocular Initialization
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    void UpdateLastFrame();
    RefKeyFrameTrackingResult TrackReferenceKeyFrameWithBoW();
    RefKeyFrameTrackingResult TrackReferenceKeyFrameNoBoW();
    MotionModelTrackingResult TrackWithMotionModel();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    LocalMapTrackingResult TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Internal helpers to keep Track logic simpler
    void PrepareFrameForTracking();
    void UpdateMapChangeState(Map* pCurrentMap);
    bool Initialize();
    void UpdateAfterTracking(bool bOK);
    int DiscardOutliersAndCountInliers(Frame& frame, int& nmatches, bool clearTrackInViewFlag);
    void BuildDepthIndex(const Frame& frame, int N, std::vector<std::pair<float, int>>& vDepthIdx) const;

    // Compute velocity from priors
    void ComputeVelocityFromPriors();

    // Load settings
    void loadFromSettings(Settings* settings);

    bool mbMapUpdated;

    bool mbAtlasNewMaps;

    // Other Thread Pointers
    LocalMapping* mpLocalMapper;

    // ORB
    FeatureExtractor *mpFeatureExtractorLeft, *mpFeatureextractorRight;
    FeatureExtractor* mpIniFeatureExtractor;

    // BoW
    IBowVocabulary* mpORBVocabulary;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    // Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    // Store the last frame image
    cv::Mat mImGrayLast;

    // System
    System* mpSystem;

    // Atlas
    Atlas* mpAtlas;

    // Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;

    // New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // New KeyFrame decision thresholds
    int mNewKFMinTrackedClosePoints;
    int mNewKFMinNonTrackedClosePoints;
    float mNewKFRefRatioMono;
    float mNewKFRefRatioStereoFewKFs;
    float mNewKFRefRatioStereo;
    float mNewKFWeakTrackingRatio;
    int mNewKFMinInliers;
    int mNewKFMaxKFsInQueue;

    // Minimum KFs in map before LOST triggers reset (instead of reusing map)
    int mLostResetMinKFs;

    // Threshold close/far points
    // Points seen as close by the stereo sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    // Current matches in frame
    int mnMatchesInliers;

    // Last Frame, KeyFrame Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    double mTimeStampLost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    // Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    GeometricCamera* mpCamera;

    int initID, lastID;
};

}  // namespace ORB_SLAM3

#endif  // TRACKING_H
