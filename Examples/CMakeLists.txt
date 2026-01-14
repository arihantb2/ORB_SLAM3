# Examples build configuration
# This file collects all example subdirectories

# Set runtime output directory for all Examples executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/Examples)

add_subdirectory(RGB-D)
add_subdirectory(RGB-D-Inertial)
add_subdirectory(Stereo)
add_subdirectory(Monocular)
add_subdirectory(Monocular-Inertial)
add_subdirectory(Stereo-Inertial)
add_subdirectory(Calibration)
