# RGB-D inertial examples

if(realsense2_FOUND)
    add_executable(rgbd_inertial_realsense_D435i rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
endif()

# Install executables to Examples/RGB-D-Inertial directory
if(realsense2_FOUND)
    install(TARGETS rgbd_inertial_realsense_D435i
        DESTINATION Examples/RGB-D-Inertial
        COMPONENT examples)
endif()
