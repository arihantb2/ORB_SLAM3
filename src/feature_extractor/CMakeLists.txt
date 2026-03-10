# feature_extractor — standalone shared library
#
# Contains: FeatureGrid, FeatureExtractor (base class), ORBFeatureExtractor.
# Dependencies: OpenCV only (no Eigen, GTSAM, DBoW2, Sophus, or ROS).
# Consumed by: ORB_SLAM3 library (linked via target_link_libraries below).

add_library(feature_extractor SHARED
    FeatureGrid.cc
    FeatureExtractor.cc
    ORBFeatureExtractor.cc
)

# Headers live under include/feature_extractor/ — expose the include root so
# consumers write: #include <feature_extractor/FeatureExtractor.h>
target_include_directories(feature_extractor
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/ORB_SLAM3>
)

target_link_libraries(feature_extractor
    PUBLIC
        opencv_core
        opencv_imgproc
        opencv_features2d
)

set_target_properties(feature_extractor PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Install alongside the main ORB_SLAM3 library so the installed package is
# self-contained. Registered under ORB_SLAM3Targets so the parent project's
# install(EXPORT) picks it up automatically.
install(TARGETS feature_extractor
    EXPORT  ORB_SLAM3Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT library
)
