/**
 * Backward-compatible wrapper — ORBextractor now delegates to ORBFeatureExtractor.
 *
 * A thin subclass is used (rather than a type alias) so that the existing
 * forward declaration "class ORBextractor;" in Frame.h and Tracking.h remains
 * valid, and all existing "new ORBextractor(...)" call sites compile unchanged.
 *
 * Implementation lives in:
 *   include/feature_extractor/ORBFeatureExtractor.h
 *   src/feature_extractor/ORBFeatureExtractor.cc
 */
#pragma once

#include <feature_extractor/ORBFeatureExtractor.h>

namespace ORB_SLAM3
{

// Inheriting all constructors from ORBFeatureExtractor preserves the original
// five-argument constructor signature:
//   ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST)
class ORBextractor : public ORBFeatureExtractor
{
public:
    using ORBFeatureExtractor::ORBFeatureExtractor;
};

}  // namespace ORB_SLAM3
