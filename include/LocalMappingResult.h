/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D.
 * Tardós, University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
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

#ifndef LOCALMAPPINGRESULT_H
#define LOCALMAPPINGRESULT_H

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ORB_SLAM3
{

// ---------------------------------------------------------------------------
// Shared value types
// ---------------------------------------------------------------------------

/// Lightweight representation of a map point created this iteration.
/// The client accumulates these to build its own sparse map.
struct NewMappingMapPoint
{
    unsigned long id = 0;                       ///< MapPoint::mnId
    Eigen::Vector3f pos_world = Eigen::Vector3f::Zero();  ///< World-frame position
    unsigned long first_kf_id = 0;              ///< mnId of the KeyFrame that created it
};

// ---------------------------------------------------------------------------
// Per-stage result structs
// ---------------------------------------------------------------------------

/// Results from ProcessNewKeyFrame().
///
/// Pops the next KeyFrame from the queue, computes its BoW descriptor,
/// associates existing MapPoints, and inserts the frame into the Atlas.
struct ProcessNewKeyFrameResult
{
    /// KeyFrame::mnId of the frame that was processed.
    unsigned long keyframe_id = 0;

    /// Frame::mnFrameId — the sequential tracking frame index.
    unsigned long frame_id = 0;

    /// Sensor timestamp of the processed KeyFrame (seconds).
    double timestamp = 0.0;

    /// World-to-camera pose (Tcw) at the time of insertion, before LBA refinement.
    Sophus::SE3f pose;

    /// Total map-point slots in the KeyFrame's observation vector.
    int num_kf_map_point_slots = 0;

    /// Number of valid (non-null, non-bad) MapPoints among those slots.
    int num_associated_map_points = 0;

    /// Stereo MapPoints created by Tracking that were registered into the
    /// recent-added list (mlpRecentAddedMapPoints) during this call.
    int num_stereo_map_points_registered = 0;

    /// Depth of the new-KeyFrame queue after this frame was popped.
    int queue_size_after = 0;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

/// Results from MapPointCulling().
///
/// Walks mlpRecentAddedMapPoints and removes unreliable points.
struct MapPointCullingResult
{
    /// Size of mlpRecentAddedMapPoints before culling.
    int num_recent_map_points_before = 0;

    /// Points already flagged bad by another thread — removed from list only,
    /// SetBadFlag() was NOT called here.
    int num_culled_already_bad = 0;

    /// Points culled because GetFoundRatio() < threshold — SetBadFlag() called.
    int num_culled_low_found_ratio = 0;

    /// Points culled because they have too few observations after the minimum
    /// KF-age window has elapsed — SetBadFlag() called.
    int num_culled_too_few_observations = 0;

    /// Points removed from the recent list because they aged beyond
    /// mMPCullingMaxKFAgeInRecent without another culling trigger.
    /// SetBadFlag() is NOT called — they graduate to the main map.
    int num_graduated = 0;

    /// Size of mlpRecentAddedMapPoints after culling.
    int num_recent_map_points_after = 0;

    /// IDs of map points on which SetBadFlag() was called during this stage
    /// (low found ratio or too few observations).
    /// The client should remove these from its own map.
    std::vector<unsigned long> culled_map_point_ids;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

/// Results from CreateNewMapPoints().
///
/// Searches for feature matches across neighbouring KeyFrames using the
/// epipolar constraint, triangulates valid pairs, and inserts new MapPoints.
struct CreateNewMapPointsResult
{
    /// Number of neighbour KeyFrames considered.
    int num_neighbour_kfs = 0;

    /// Total epipolar matches found across all neighbour pairs before
    /// triangulation checks.
    int num_epipolar_matches = 0;

    /// Number of stereo-unproject attempts (bPointStereo path).
    int num_stereo_unproject_attempts = 0;

    /// Number of MapPoints successfully created this stage.
    int num_created = 0;

    /// Subset of num_created that came from stereo unprojection rather than
    /// multi-view triangulation.
    int num_created_from_stereo = 0;

    /// True if the loop was aborted early because a new KeyFrame arrived.
    bool aborted_early = false;

    /// The newly created MapPoints.
    std::vector<NewMappingMapPoint> new_map_points;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

/// Results from SearchInNeighbors().
///
/// Fuses duplicate MapPoints between the current KeyFrame and its neighbours
/// (bidirectional projection search).
struct SearchInNeighborsResult
{
    /// Number of first-level neighbour KeyFrames added as fusion targets.
    int num_first_level_neighbours = 0;

    /// Number of second-level (covisible-of-covisible) KeyFrames also added.
    int num_second_level_neighbours = 0;

    /// Total fusion target KeyFrames (first + second level, deduplicated).
    int num_target_kfs = 0;

    /// True if the stage exited early because mbAbortBA was set.
    bool aborted_early = false;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

/// Results from LocalBundleAdjustment (LBA).
///
/// LBA is throttled by mOptimizeEveryTSeconds so it may be skipped entirely.
struct LocalBundleAdjustmentResult
{
    /// True if LBA was skipped this iteration.
    bool skipped = false;

    /// Reason LBA was skipped (empty string when skipped == false).
    /// Values: "throttled", "too_few_keyframes", "stop_requested", "new_kf_arrived".
    std::string skip_reason;

    /// Number of KeyFrames held fixed (anchor nodes) in the optimisation graph.
    int num_fixed_kfs = 0;

    /// mnId of every fixed KeyFrame (lFixedCameras inside
    /// Optimizer::LocalBundleAdjustment). These are covisible neighbours whose
    /// own neighbours fall outside the local window. Their poses are unchanged.
    std::vector<unsigned long> fixed_keyframe_ids;

    /// Number of KeyFrames whose poses were optimised.
    int num_optimised_kfs = 0;

    /// mnId of every optimised KeyFrame (lLocalKeyFrames inside
    /// Optimizer::LocalBundleAdjustment). The current KeyFrame is always first.
    std::vector<unsigned long> optimised_keyframe_ids;

    /// Number of MapPoints included in the optimisation.
    int num_map_points = 0;

    /// Number of reprojection edges in the optimisation graph.
    int num_edges = 0;

    /// MapPoint IDs rejected as outliers by the post-optimisation reprojection-
    /// error pass (EraseObservation / SetBadFlag called).
    /// The client should remove these from its own map.
    std::vector<unsigned long> outlier_map_point_ids;

    /// Convenience count; equals outlier_map_point_ids.size().
    int num_outlier_map_points = 0;

    /// Wall-clock duration of this stage (milliseconds). Zero when skipped.
    double duration_ms = 0.0;
};

/// Results from KeyFrameCulling().
///
/// Marks redundant KeyFrames bad when >= mKeyFrameCullingRedundantRatio of
/// their close MapPoints are observed by at least mKeyFrameCullingMinObsInOthers
/// other KeyFrames at an equal or finer scale.
struct KeyFrameCullingResult
{
    /// Number of local KeyFrames examined.
    int num_kfs_checked = 0;

    /// Number of KeyFrames marked bad (SetBadFlag called).
    int num_kfs_culled = 0;

    /// True if the loop exited early due to mbAbortBA or the max-check cap.
    bool aborted_early = false;

    /// mnId values of KeyFrames that were marked bad.
    std::vector<unsigned long> culled_keyframe_ids;

    /// Wall-clock duration of this stage (milliseconds).
    double duration_ms = 0.0;
};

// ---------------------------------------------------------------------------
// Top-level result
// ---------------------------------------------------------------------------

/// Complete result of one RunLoop() iteration of the LocalMapping thread.
///
/// Produced at the end of every iteration in which a KeyFrame was actually
/// processed. The callback is NOT invoked on idle iterations.
///
/// The struct is fully self-contained: no raw pointers into library internals.
struct LocalMappingResult
{
    // -----------------------------------------------------------------------
    // Context
    // -----------------------------------------------------------------------

    /// Monotonically increasing counter starting at 1.
    uint64_t iteration = 0;

    /// KeyFrame::mnId of the frame processed this iteration.
    unsigned long keyframe_id = 0;

    /// Frame::mnFrameId of the frame processed this iteration.
    unsigned long frame_id = 0;

    /// Sensor timestamp of the processed KeyFrame (seconds).
    double timestamp = 0.0;

    // -----------------------------------------------------------------------
    // Per-stage results
    // -----------------------------------------------------------------------

    ProcessNewKeyFrameResult process_new_keyframe;
    MapPointCullingResult map_point_culling;
    CreateNewMapPointsResult create_new_map_points;

    /// True when SearchInNeighbors was skipped because a new KeyFrame was
    /// waiting in the queue.
    bool search_in_neighbors_skipped = false;
    SearchInNeighborsResult search_in_neighbors;

    LocalBundleAdjustmentResult lba;
    KeyFrameCullingResult keyframe_culling;

    // -----------------------------------------------------------------------
    // Convenience map-point delta summary
    // -----------------------------------------------------------------------

    /// All newly created map points this iteration.
    /// Mirrors create_new_map_points.new_map_points.
    std::vector<NewMappingMapPoint> added_map_points;

    /// Map-point IDs removed by MapPointCulling (SetBadFlag called).
    /// Mirrors map_point_culling.culled_map_point_ids.
    std::vector<unsigned long> culled_map_point_ids;

    /// Map-point IDs rejected as outliers by LBA's reprojection-error pass.
    /// Mirrors lba.outlier_map_point_ids.
    /// Empty until the outlier-rejection pass is active in
    /// Optimizer::LocalBundleAdjustment.
    std::vector<unsigned long> lba_outlier_map_point_ids;

    // -----------------------------------------------------------------------
    // Timing summary
    // -----------------------------------------------------------------------

    /// Total wall-clock duration of the RunLoop() body for this iteration
    /// (milliseconds). Includes all sub-stage durations.
    double total_duration_ms = 0.0;
};

// ---------------------------------------------------------------------------
// Callback type
// ---------------------------------------------------------------------------

/// Callback invoked from the LocalMapping thread at the end of every iteration
/// that processes a KeyFrame. Registered via System::SetLocalMappingCallback().
///
/// Must be thread-safe and return quickly. Copy the result and hand it off to
/// another thread if heavy processing is needed.
using LocalMappingCallback = std::function<void(const LocalMappingResult&)>;

}  // namespace ORB_SLAM3

#endif  // LOCALMAPPINGRESULT_H
