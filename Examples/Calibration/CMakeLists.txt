# Calibration examples

if(realsense2_FOUND)
    add_executable(recorder_realsense_D435i recorder_realsense_D435i.cc)
    target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})

    add_executable(recorder_realsense_T265 recorder_realsense_T265.cc)
    target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
endif()

# Install executables to Examples/Calibration directory
if(realsense2_FOUND)
    install(TARGETS recorder_realsense_D435i recorder_realsense_T265
        DESTINATION Examples/Calibration
        COMPONENT examples)
endif()
