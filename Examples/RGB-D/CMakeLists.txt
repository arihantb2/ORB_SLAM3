# RGB-D examples

add_executable(rgbd_tum rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
endif()

# Install executables to Examples/RGB-D directory
install(TARGETS rgbd_tum
    DESTINATION Examples/RGB-D
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS rgbd_realsense_D435i
        DESTINATION Examples/RGB-D
        COMPONENT examples)
endif()
