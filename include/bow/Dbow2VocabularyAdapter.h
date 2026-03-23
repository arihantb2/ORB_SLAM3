#ifndef DBOW2_VOCABULARY_ADAPTER_H
#define DBOW2_VOCABULARY_ADAPTER_H

#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"
#include "bow/IBowVocabulary.h"

namespace ORB_SLAM3
{

class Dbow2VocabularyAdapter : public IBowVocabulary
{
public:
    using VocabularyImpl = DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>;

    bool load(const std::string& filePath) override;
    size_t size() const override;
    bool supportsDescriptorType(int cvType) const override;

    void transform(const std::vector<cv::Mat>& descriptors, BowVector& bowVec, FeatureVector& featVec,
                   int levelSup) const override;
    float score(const BowVector& lhs, const BowVector& rhs) const override;

private:
    VocabularyImpl vocab_;
};

}  // namespace ORB_SLAM3

#endif  // DBOW2_VOCABULARY_ADAPTER_H
