cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3 VERSION 0.0.1)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ORB_SLAM3_VERSION ${PROJECT_VERSION})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Match GTSAM's build to avoid Eigen ABI / double-free (see src/tests/CMakeLists.txt).
# Build GTSAM with: -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
option(ORB_SLAM3_USE_MARCH_NATIVE "Use -march=native for ORB_SLAM3 (set OFF if GTSAM was built without it)" OFF)
if(ORB_SLAM3_USE_MARCH_NATIVE)
  set(ORB_SLAM3_MARCH_FLAG "-march=native")
else()
  set(ORB_SLAM3_MARCH_FLAG "")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_BUILD_TYPE MATCHES "^[Dd][Ee][Bb][Uu][Gg]$")
    set(ORB_SLAM3_OPTS -Wall -O0 -g -fstack-protector-all)
else()
    set(ORB_SLAM3_OPTS -Wall -O3)
endif()
if(ORB_SLAM3_MARCH_FLAG)
    list(APPEND ORB_SLAM3_OPTS ${ORB_SLAM3_MARCH_FLAG})
endif()
add_compile_options(${ORB_SLAM3_OPTS})

# ==============================================================================
# Main project configuration
# ==============================================================================

# Set output directories for the main project
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Configure RPATH so installed libraries can find their dependencies
# Standard CMake practice: set RPATH to install lib directory
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)
find_package(Sophus 1.22 REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(fbow REQUIRED)
find_package(GTSAM REQUIRED)

# Build the main library
add_subdirectory(src)

# Unit tests
enable_testing()
add_subdirectory(src/CameraModels/tests)
add_subdirectory(src/tests)

# ---------------
# Installation instructions.
# ---------------
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM3
    COMPONENT library
    FILES_MATCHING PATTERN "*.h"
    PATTERN "Thirdparty" EXCLUDE)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/CameraModels/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM3/CameraModels
    COMPONENT library
    FILES_MATCHING PATTERN "*.h"
    PATTERN "Thirdparty" EXCLUDE)

install(FILES ${PROJECT_SOURCE_DIR}/package.xml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/orb_slam3)

set(ORB_SLAM3_INSTALL_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/cmake/ORB_SLAM3Config.cmake.in
    ${PROJECT_BINARY_DIR}/ORB_SLAM3Config.cmake
    INSTALL_DESTINATION ${ORB_SLAM3_INSTALL_CONFIG_DIR}
)

write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/ORB_SLAM3ConfigVersion.cmake
    VERSION ${ORB_SLAM3_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(EXPORT ORB_SLAM3Targets
    NAMESPACE ORB_SLAM3::
    DESTINATION ${ORB_SLAM3_INSTALL_CONFIG_DIR}
)

install(FILES
    ${PROJECT_BINARY_DIR}/ORB_SLAM3Config.cmake
    ${PROJECT_BINARY_DIR}/ORB_SLAM3ConfigVersion.cmake
    DESTINATION ${ORB_SLAM3_INSTALL_CONFIG_DIR}
)

export(EXPORT ORB_SLAM3Targets
    FILE ${PROJECT_BINARY_DIR}/ORB_SLAM3Targets.cmake
    NAMESPACE ORB_SLAM3::
)
