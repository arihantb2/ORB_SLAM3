#include "feature_extractor/FeatureGrid.h"

#include <algorithm>
#include <cmath>

namespace ORB_SLAM3
{

FeatureGrid::FeatureGrid(int cols, int rows, float minX, float maxX, float minY, float maxY)
    : mNumCols(cols), mNumRows(rows), mMinX(minX), mMaxX(maxX), mMinY(minY), mMaxY(maxY)
{
    mfGridElementWidthInv = static_cast<float>(cols) / (maxX - minX);
    mfGridElementHeightInv = static_cast<float>(rows) / (maxY - minY);
    mGrid.assign(cols, std::vector<std::vector<size_t>>(rows));
}

void FeatureGrid::assignFeatures(const std::vector<cv::KeyPoint>& keysUn)
{
    for (auto& col : mGrid)
        for (auto& cell : col)
            cell.clear();

    for (size_t i = 0; i < keysUn.size(); i++)
    {
        int posX, posY;
        if (posInGrid(keysUn[i], posX, posY))
            mGrid[posX][posY].push_back(i);
    }
}

std::vector<size_t> FeatureGrid::getFeaturesInArea(const std::vector<cv::KeyPoint>& keysUn, float x, float y, float r,
                                                   int minLevel, int maxLevel) const
{
    std::vector<size_t> vIndices;

    const int nMinCellX = std::max(0, static_cast<int>(std::floor((x - mMinX - r) * mfGridElementWidthInv)));
    if (nMinCellX >= mNumCols)
        return vIndices;

    const int nMaxCellX = std::min(mNumCols - 1, static_cast<int>(std::ceil((x - mMinX + r) * mfGridElementWidthInv)));
    if (nMaxCellX < 0)
        return vIndices;

    const int nMinCellY = std::max(0, static_cast<int>(std::floor((y - mMinY - r) * mfGridElementHeightInv)));
    if (nMinCellY >= mNumRows)
        return vIndices;

    const int nMaxCellY = std::min(mNumRows - 1, static_cast<int>(std::ceil((y - mMinY + r) * mfGridElementHeightInv)));
    if (nMaxCellY < 0)
        return vIndices;

    const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

    for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
    {
        for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
        {
            const auto& vCell = mGrid[ix][iy];
            if (vCell.empty())
                continue;

            for (size_t idx : vCell)
            {
                const cv::KeyPoint& kpUn = keysUn[idx];

                if (bCheckLevels)
                {
                    if (kpUn.octave < minLevel)
                        continue;
                    if (maxLevel >= 0 && kpUn.octave > maxLevel)
                        continue;
                }

                const float distx = kpUn.pt.x - x;
                const float disty = kpUn.pt.y - y;
                if (std::fabs(distx) < r && std::fabs(disty) < r)
                    vIndices.push_back(idx);
            }
        }
    }
    return vIndices;
}

bool FeatureGrid::posInGrid(const cv::KeyPoint& kp, int& posX, int& posY) const
{
    posX = static_cast<int>(std::round((kp.pt.x - mMinX) * mfGridElementWidthInv));
    posY = static_cast<int>(std::round((kp.pt.y - mMinY) * mfGridElementHeightInv));

    if (posX < 0 || posX >= mNumCols || posY < 0 || posY >= mNumRows)
        return false;
    return true;
}

}  // namespace ORB_SLAM3
