#ifndef BOW_VOCABULARY_FACTORY_H
#define BOW_VOCABULARY_FACTORY_H

#include "bow/IBowVocabulary.h"

#include <memory>
#include <string>

namespace ORB_SLAM3
{

std::unique_ptr<IBowVocabulary> CreateBowVocabulary(const std::string& vocabularyType);

}  // namespace ORB_SLAM3

#endif  // BOW_VOCABULARY_FACTORY_H
