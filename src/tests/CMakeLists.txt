# Unit tests for custom GTSAM factor types (ORB_SLAM3)
# Links against ORB_SLAM3 and uses GTSAM numericalDerivative for Jacobian checks.

find_package(GTest REQUIRED)

add_executable(orb_slam3_gtsam_factor_tests
    GTSAMFactorTests.cpp
)

target_include_directories(orb_slam3_gtsam_factor_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${GTSAM_INCLUDE_DIR}
)

target_link_libraries(orb_slam3_gtsam_factor_tests
    ORB_SLAM3
    gtsam
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_test(NAME orb_slam3_gtsam_factor_tests COMMAND orb_slam3_gtsam_factor_tests)

# Unit tests for MapPointPool chunk lifecycle and Map's MapPoint reclamation
# policy. Only needs the core ORB_SLAM3 library (Map/MapPoint/KeyFrame/
# MapPointPool) — no GTSAM dependency.
add_executable(orb_slam3_map_point_pool_tests
    MapPointPoolTests.cpp
)

target_link_libraries(orb_slam3_map_point_pool_tests
    ORB_SLAM3
    GTest::gtest_main
    ${OpenCV_LIBS}
)

add_test(NAME orb_slam3_map_point_pool_tests COMMAND orb_slam3_map_point_pool_tests)
