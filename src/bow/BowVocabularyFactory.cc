#include "bow/BowVocabularyFactory.h"

#include "bow/FbowVocabularyAdapter.h"

namespace ORB_SLAM3
{

std::unique_ptr<IBowVocabulary> CreateBowVocabulary()
{
    return std::unique_ptr<IBowVocabulary>(new FbowVocabularyAdapter());
}

}  // namespace ORB_SLAM3
