find_package(GTest REQUIRED)

add_executable(orb_slam3_feature_extractor_tests
    FeatureExtractorTests.cpp
)

target_link_libraries(orb_slam3_feature_extractor_tests
    feature_extractor
    ORB_SLAM3
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_test(NAME orb_slam3_feature_extractor_tests COMMAND orb_slam3_feature_extractor_tests)
