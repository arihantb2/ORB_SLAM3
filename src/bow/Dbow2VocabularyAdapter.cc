#include "bow/Dbow2VocabularyAdapter.h"

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"

namespace ORB_SLAM3
{

namespace
{
DBoW2::BowVector ToDbow2(const BowVector& bowVec)
{
    DBoW2::BowVector converted;
    for (BowVector::const_iterator it = bowVec.begin(); it != bowVec.end(); ++it)
    {
        converted[it->first] = it->second;
    }
    return converted;
}

void FromDbow2(const DBoW2::BowVector& in, BowVector& out)
{
    out.clear();
    for (DBoW2::BowVector::const_iterator it = in.begin(); it != in.end(); ++it)
    {
        out[it->first] = it->second;
    }
}

void FromDbow2(const DBoW2::FeatureVector& in, FeatureVector& out)
{
    out.clear();
    for (DBoW2::FeatureVector::const_iterator it = in.begin(); it != in.end(); ++it)
    {
        out[it->first] = std::vector<unsigned int>(it->second.begin(), it->second.end());
    }
}
}  // namespace

bool Dbow2VocabularyAdapter::load(const std::string& filePath)
{
    return vocab_.loadFromTextFile(filePath);
}

size_t Dbow2VocabularyAdapter::size() const
{
    return vocab_.size();
}

bool Dbow2VocabularyAdapter::supportsDescriptorType(int cvType) const
{
    return cvType == CV_8UC1;
}

void Dbow2VocabularyAdapter::transform(const std::vector<cv::Mat>& descriptors, BowVector& bowVec,
                                       FeatureVector& featVec, int levelSup) const
{
    DBoW2::BowVector dbowBow;
    DBoW2::FeatureVector dbowFeat;
    vocab_.transform(descriptors, dbowBow, dbowFeat, levelSup);
    FromDbow2(dbowBow, bowVec);
    FromDbow2(dbowFeat, featVec);
}

float Dbow2VocabularyAdapter::score(const BowVector& lhs, const BowVector& rhs) const
{
    const DBoW2::BowVector lhsDbow = ToDbow2(lhs);
    const DBoW2::BowVector rhsDbow = ToDbow2(rhs);
    return static_cast<float>(vocab_.score(lhsDbow, rhsDbow));
}

}  // namespace ORB_SLAM3
