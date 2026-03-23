#ifndef FBOW_VOCABULARY_ADAPTER_H
#define FBOW_VOCABULARY_ADAPTER_H

#include "bow/IBowVocabulary.h"

#include "fbow.h"

namespace ORB_SLAM3
{

class FbowVocabularyAdapter : public IBowVocabulary
{
public:
    bool load(const std::string& filePath) override;
    size_t size() const override;
    bool supportsDescriptorType(int cvType) const override;

    void transform(const std::vector<cv::Mat>& descriptors, BowVector& bowVec, FeatureVector& featVec,
                   int levelSup) const override;
    float score(const BowVector& lhs, const BowVector& rhs) const override;

private:
    mutable fbow::Vocabulary vocab_;
};

}  // namespace ORB_SLAM3

#endif  // FBOW_VOCABULARY_ADAPTER_H
