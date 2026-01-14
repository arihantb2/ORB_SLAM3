# RGB-D examples

add_executable(rgbd_tum rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
endif()

# Install executables to bin/Examples directory
install(TARGETS rgbd_tum
    DESTINATION bin/Examples
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS rgbd_realsense_D435i
        DESTINATION bin/Examples
        COMPONENT examples)
endif()
