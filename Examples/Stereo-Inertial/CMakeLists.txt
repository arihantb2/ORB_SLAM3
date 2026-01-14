# Stereo Inertial examples

add_executable(stereo_inertial_euroc stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

add_executable(stereo_inertial_tum_vi stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(stereo_inertial_realsense_t265 stereo_inertial_realsense_t265.cc)
    target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})

    add_executable(stereo_inertial_realsense_D435i stereo_inertial_realsense_D435i.cc)
    target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
endif()

# Install executables to Examples/Stereo-Inertial directory
install(TARGETS stereo_inertial_euroc stereo_inertial_tum_vi
    DESTINATION Examples/Stereo-Inertial
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS stereo_inertial_realsense_t265 stereo_inertial_realsense_D435i
        DESTINATION Examples/Stereo-Inertial
        COMPONENT examples)
endif()
