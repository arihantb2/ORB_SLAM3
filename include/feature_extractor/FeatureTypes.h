#pragma once

namespace ORB_SLAM3
{

// Descriptor storage format — determines distance metric.
// BINARY:  CV_8UC1, 32 bytes/row  (ORB). Use Hamming distance.
// FLOAT32: CV_32FC1, 128 floats/row (SIFT). Use L2 distance.
enum class DescriptorType
{
    BINARY,
    FLOAT32
};

}  // namespace ORB_SLAM3
