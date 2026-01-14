# Stereo Inertial examples (old)

add_executable(stereo_inertial_euroc_old stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc_old ${PROJECT_NAME})

add_executable(stereo_inertial_tum_vi_old stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265_old stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265_old ${PROJECT_NAME})

    add_executable(stereo_inertial_realsense_D435i_old stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i_old ${PROJECT_NAME})
endif()

# Install executables to Examples_old/Stereo-Inertial directory
install(TARGETS stereo_inertial_euroc_old stereo_inertial_tum_vi_old
    DESTINATION Examples_old/Stereo-Inertial
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS stereo_inertial_realsense_t265_old stereo_inertial_realsense_D435i_old
        DESTINATION Examples_old/Stereo-Inertial
        COMPONENT examples)
endif()
