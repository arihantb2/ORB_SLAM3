# RGB-D inertial examples (old)

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i_old rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i_old ${PROJECT_NAME})
endif()

# Install executables to Examples_old/RGB-D-Inertial directory
if(realsense2_FOUND)
    install(TARGETS rgbd_inertial_realsense_D435i_old
        DESTINATION Examples_old/RGB-D-Inertial
        COMPONENT examples)
endif()
