# Stereo examples (old)

add_executable(stereo_kitti_old stereo_kitti.cc)
target_link_libraries(stereo_kitti_old ${PROJECT_NAME})

add_executable(stereo_euroc_old stereo_euroc.cc)
target_link_libraries(stereo_euroc_old ${PROJECT_NAME})

add_executable(stereo_tum_vi_old stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(stereo_realsense_t265_old stereo_realsense_t265.cc)
    target_link_libraries(stereo_realsense_t265_old ${PROJECT_NAME})

    add_executable(stereo_realsense_D435i_old stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i_old ${PROJECT_NAME})
endif()

# Install executables to bin/Examples_old directory
install(TARGETS stereo_kitti_old stereo_euroc_old stereo_tum_vi_old
    DESTINATION bin/Examples_old
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS stereo_realsense_t265_old stereo_realsense_D435i_old
        DESTINATION bin/Examples_old
        COMPONENT examples)
endif()
