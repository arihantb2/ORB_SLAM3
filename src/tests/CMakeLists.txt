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
)

add_test(NAME orb_slam3_gtsam_factor_tests COMMAND orb_slam3_gtsam_factor_tests)
