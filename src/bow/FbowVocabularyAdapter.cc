#include "bow/FbowVocabularyAdapter.h"

#include <stdexcept>

namespace ORB_SLAM3
{

namespace
{
fbow::fBow ToFbow(const BowVector& vec)
{
    fbow::fBow out;
    for (BowVector::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        out[it->first].var = it->second;
    }
    return out;
}

cv::Mat StackDescriptors(const std::vector<cv::Mat>& descriptors)
{
    if (descriptors.empty())
    {
        return cv::Mat();
    }

    int rows = 0;
    const int cols = descriptors.front().cols;
    const int type = descriptors.front().type();
    for (size_t i = 0; i < descriptors.size(); ++i)
    {
        if (descriptors[i].empty())
        {
            continue;
        }
        if (descriptors[i].cols != cols || descriptors[i].type() != type)
        {
            throw std::runtime_error("Descriptor shape mismatch for fbow transform");
        }
        rows += descriptors[i].rows;
    }

    cv::Mat stacked(rows, cols, type);
    int rowOffset = 0;
    for (size_t i = 0; i < descriptors.size(); ++i)
    {
        if (descriptors[i].empty())
        {
            continue;
        }
        const cv::Mat block = stacked.rowRange(rowOffset, rowOffset + descriptors[i].rows);
        descriptors[i].copyTo(block);
        rowOffset += descriptors[i].rows;
    }
    return stacked;
}
}  // namespace

bool FbowVocabularyAdapter::load(const std::string& filePath)
{
    vocab_.readFromFile(filePath);
    return vocab_.size() > 0;
}

size_t FbowVocabularyAdapter::size() const
{
    return vocab_.size();
}

bool FbowVocabularyAdapter::supportsDescriptorType(int cvType) const
{
    return cvType == CV_8UC1;
}

void FbowVocabularyAdapter::transform(const std::vector<cv::Mat>& descriptors, BowVector& bowVec,
                                      FeatureVector& featVec, int levelSup) const
{
    bowVec.clear();
    featVec.clear();
    const cv::Mat descriptorMatrix = StackDescriptors(descriptors);
    if (descriptorMatrix.empty())
    {
        return;
    }

    fbow::fBow fbowVec;
    fbow::fBow2 fbowFeatVec;
    vocab_.transform(descriptorMatrix, levelSup, fbowVec, fbowFeatVec);
    for (fbow::fBow::const_iterator it = fbowVec.begin(); it != fbowVec.end(); ++it)
    {
        bowVec[it->first] = static_cast<float>(it->second);
    }
    for (fbow::fBow2::const_iterator it = fbowFeatVec.begin(); it != fbowFeatVec.end(); ++it)
    {
        const std::vector<uint32_t>& src = it->second;
        std::vector<unsigned int>& dst = featVec[it->first];
        dst.assign(src.begin(), src.end());
    }
}

float FbowVocabularyAdapter::score(const BowVector& lhs, const BowVector& rhs) const
{
    const fbow::fBow fbowLhs = ToFbow(lhs);
    const fbow::fBow fbowRhs = ToFbow(rhs);
    return static_cast<float>(fbow::fBow::score(fbowLhs, fbowRhs));
}

}  // namespace ORB_SLAM3
