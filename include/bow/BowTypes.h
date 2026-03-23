#ifndef BOW_TYPES_H
#define BOW_TYPES_H

#include <map>
#include <vector>

namespace ORB_SLAM3
{

using BowWordId = unsigned int;
using BowWordWeight = float;

using BowVector = std::map<BowWordId, BowWordWeight>;
using FeatureVector = std::map<BowWordId, std::vector<unsigned int>>;

}  // namespace ORB_SLAM3

#endif  // BOW_TYPES_H
