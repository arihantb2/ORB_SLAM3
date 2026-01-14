# Monocular examples (old)

add_executable(mono_tum_old mono_tum.cc)
target_link_libraries(mono_tum_old ${PROJECT_NAME})

add_executable(mono_kitti_old mono_kitti.cc)
target_link_libraries(mono_kitti_old ${PROJECT_NAME})

add_executable(mono_euroc_old mono_euroc.cc)
target_link_libraries(mono_euroc_old ${PROJECT_NAME})

add_executable(mono_tum_vi_old mono_tum_vi.cc)
target_link_libraries(mono_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(mono_realsense_t265_old mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265_old ${PROJECT_NAME})

    add_executable(mono_realsense_D435i_old mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i_old ${PROJECT_NAME})
endif()

# Install executables to bin/Examples_old directory
install(TARGETS mono_tum_old mono_kitti_old mono_euroc_old mono_tum_vi_old
    DESTINATION bin/Examples_old
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS mono_realsense_t265_old mono_realsense_D435i_old
        DESTINATION bin/Examples_old
        COMPONENT examples)
endif()
