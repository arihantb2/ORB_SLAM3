# RGB-D inertial examples (old)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i_old rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i_old ${PROJECT_NAME})
endif()

# Install executables to bin/Examples_old directory
if(realsense2_FOUND)
    install(TARGETS rgbd_inertial_realsense_D435i_old
        DESTINATION bin/Examples_old
        COMPONENT examples)
endif()
