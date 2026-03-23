#include "bow/BowVocabularyFactory.h"

#include "bow/Dbow2VocabularyAdapter.h"
#include "bow/FbowVocabularyAdapter.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace ORB_SLAM3
{

namespace
{
std::string ToLower(const std::string& in)
{
    std::string out = in;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}
}  // namespace

std::unique_ptr<IBowVocabulary> CreateBowVocabulary(const std::string& vocabularyType)
{
    const std::string type = ToLower(vocabularyType);
    if (type == "dbow2")
    {
        return std::unique_ptr<IBowVocabulary>(new Dbow2VocabularyAdapter());
    }
    if (type == "fbow")
    {
        return std::unique_ptr<IBowVocabulary>(new FbowVocabularyAdapter());
    }

    throw std::runtime_error("Unsupported vocabulary type: " + vocabularyType);
}

}  // namespace ORB_SLAM3
