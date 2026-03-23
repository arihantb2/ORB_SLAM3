#ifndef I_BOW_VOCABULARY_H
#define I_BOW_VOCABULARY_H

#include "bow/BowTypes.h"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace ORB_SLAM3
{

class IBowVocabulary
{
public:
    virtual ~IBowVocabulary() = default;

    virtual bool load(const std::string& filePath) = 0;
    virtual size_t size() const = 0;
    virtual bool supportsDescriptorType(int cvType) const = 0;

    virtual void transform(const std::vector<cv::Mat>& descriptors, BowVector& bowVec, FeatureVector& featVec,
                           int levelSup) const = 0;
    virtual float score(const BowVector& lhs, const BowVector& rhs) const = 0;
};

}  // namespace ORB_SLAM3

#endif  // I_BOW_VOCABULARY_H
