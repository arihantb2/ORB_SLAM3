# Monocular inertial examples

add_executable(mono_inertial_euroc mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(mono_inertial_realsense_t265 mono_inertial_realsense_t265.cc)
    target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})

    add_executable(mono_inertial_realsense_D435i mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
endif()

# Install executables to bin/Examples directory
install(TARGETS mono_inertial_euroc mono_inertial_tum_vi
    DESTINATION bin/Examples
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS mono_inertial_realsense_t265 mono_inertial_realsense_D435i
        DESTINATION bin/Examples
        COMPONENT examples)
endif()
