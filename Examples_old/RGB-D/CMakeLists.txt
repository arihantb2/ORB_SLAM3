# RGB-D examples (old)

add_executable(rgbd_tum_old rgbd_tum.cc)
target_link_libraries(rgbd_tum_old ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i_old rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i_old ${PROJECT_NAME})
endif()

# Install executables to Examples_old/RGB-D directory
install(TARGETS rgbd_tum_old
    DESTINATION Examples_old/RGB-D
    COMPONENT examples)

if(realsense2_FOUND)
    install(TARGETS rgbd_realsense_D435i_old
        DESTINATION Examples_old/RGB-D
        COMPONENT examples)
endif()
