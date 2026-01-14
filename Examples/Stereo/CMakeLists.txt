# Stereo examples

add_executable(stereo_kitti stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

add_executable(stereo_tum_vi stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(stereo_realsense_t265 stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})

    add_executable(stereo_realsense_D435i stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
endif()

# Install executables to Examples/Stereo directory
install(TARGETS stereo_kitti stereo_euroc stereo_tum_vi
    DESTINATION Examples/Stereo
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS stereo_realsense_t265 stereo_realsense_D435i
        DESTINATION Examples/Stereo
        COMPONENT examples)
endif()
