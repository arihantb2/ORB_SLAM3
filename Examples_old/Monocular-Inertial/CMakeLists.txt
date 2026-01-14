# Monocular inertial examples (old)

add_executable(mono_inertial_euroc_old mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc_old ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi_old mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265_old mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265_old ${PROJECT_NAME})

    add_executable(mono_inertial_realsense_D435i_old mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i_old ${PROJECT_NAME})
endif()

# Install executables to bin/Examples_old directory
install(TARGETS mono_inertial_euroc_old mono_inertial_tum_vi_old
    DESTINATION bin/Examples_old
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS mono_inertial_realsense_t265_old mono_inertial_realsense_D435i_old
        DESTINATION bin/Examples_old
        COMPONENT examples)
endif()
