# Monocular examples

add_executable(mono_tum mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_tum_vi mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(mono_realsense_t265 mono_realsense_t265.cc)
    target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})

    add_executable(mono_realsense_D435i mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
endif()

# Install executables to bin/Examples directory
install(TARGETS mono_tum mono_kitti mono_euroc mono_tum_vi
    DESTINATION bin/Examples
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS mono_realsense_t265 mono_realsense_D435i
        DESTINATION bin/Examples
        COMPONENT examples)
endif()
