#pragma once

#include <cstddef>
#include <opencv2/core/core.hpp>
#include <vector>

namespace ORB_SLAM3
{

// Encapsulates the 2-D spatial index used to accelerate GetFeaturesInArea queries.
// Replaces the fixed mGrid[64][48] + static helper members scattered across Frame.
//
// Usage:
//   FeatureGrid grid(64, 48, mnMinX, mnMaxX, mnMinY, mnMaxY);
//   grid.assignFeatures(mvKeysUn);
//   auto indices = grid.getFeaturesInArea(keysUn, x, y, r, minLevel, maxLevel);
class FeatureGrid
{
public:
    FeatureGrid() = default;

    // Configure grid dimensions and image bounds.
    // minX/maxX/minY/maxY come from Frame::ComputeImageBounds().
    FeatureGrid(int cols, int rows, float minX, float maxX, float minY, float maxY);

    // Populate the grid from undistorted keypoints. Clears previous state.
    void assignFeatures(const std::vector<cv::KeyPoint>& keysUn);

    // Returns indices of keypoints within the (x±r, y±r) window,
    // filtered by octave level if minLevel / maxLevel >= 0.
    // keysUn must be the same vector that was passed to assignFeatures().
    std::vector<size_t> getFeaturesInArea(const std::vector<cv::KeyPoint>& keysUn, float x, float y, float r,
                                          int minLevel = -1, int maxLevel = -1) const;

    // Maps an undistorted keypoint to a grid cell.
    // Returns false if the keypoint falls outside the image bounds.
    bool posInGrid(const cv::KeyPoint& kp, int& posX, int& posY) const;

    // Direct access to the underlying 3-D vector [col][row] → indices.
    // Used by KeyFrame to copy grid state from Frame.
    const std::vector<std::vector<std::vector<size_t>>>& data() const { return mGrid; }

    int numCols() const { return mNumCols; }
    int numRows() const { return mNumRows; }
    float gridElementWidthInv() const { return mfGridElementWidthInv; }
    float gridElementHeightInv() const { return mfGridElementHeightInv; }
    float minX() const { return mMinX; }
    float maxX() const { return mMaxX; }
    float minY() const { return mMinY; }
    float maxY() const { return mMaxY; }

private:
    int mNumCols{64}, mNumRows{48};
    float mMinX{}, mMaxX{}, mMinY{}, mMaxY{};
    float mfGridElementWidthInv{}, mfGridElementHeightInv{};

    // Layout: [col][row] → vector of keypoint indices.
    // Matches KeyFrame's dynamic mGrid layout (eliminates the copy mismatch
    // between Frame's fixed array and KeyFrame's dynamic vector).
    std::vector<std::vector<std::vector<size_t>>> mGrid;
};

}  // namespace ORB_SLAM3
