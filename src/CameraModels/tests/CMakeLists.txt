add_executable(orb_slam3_metashape_camera_tests
    MetashapeCameraTests.cpp
)

target_link_libraries(orb_slam3_metashape_camera_tests
    ORB_SLAM3
)

add_test(NAME orb_slam3_metashape_camera_tests COMMAND orb_slam3_metashape_camera_tests)

add_executable(orb_slam3_pinhole_camera_tests
    PinholeCameraTests.cpp
)

target_link_libraries(orb_slam3_pinhole_camera_tests
    ORB_SLAM3
)

add_test(NAME orb_slam3_pinhole_camera_tests COMMAND orb_slam3_pinhole_camera_tests)

