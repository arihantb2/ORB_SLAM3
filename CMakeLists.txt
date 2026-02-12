cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3 VERSION 0.0.1)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ORB_SLAM3_VERSION ${PROJECT_VERSION})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_BUILD_TYPE MATCHES "^[Dd][Ee][Bb][Uu][Gg]$")
    add_compile_options(-Wall -O0 -g -march=native -fstack-protector-all)
else()
    add_compile_options(-Wall -O3 -march=native)
endif()

# Extract vocabulary if needed
message(STATUS "Uncompress vocabulary ...")
set(VOCABULARY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt)
set(VOCABULARY_ARCHIVE ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz)
if(NOT EXISTS ${VOCABULARY_FILE} AND EXISTS ${VOCABULARY_ARCHIVE})
    add_custom_command(
        OUTPUT ${VOCABULARY_FILE}
        COMMAND ${CMAKE_COMMAND} -E tar xzf ${VOCABULARY_ARCHIVE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary
        COMMENT "Extracting vocabulary file"
    )
    add_custom_target(extract_vocabulary ALL DEPENDS ${VOCABULARY_FILE})
    message(STATUS "Vocabulary extraction will be performed during build")
elseif(EXISTS ${VOCABULARY_FILE})
    message(STATUS "Vocabulary file already exists: ${VOCABULARY_FILE}")
else()
    message(FATAL_ERROR
        "Vocabulary not found.\n"
        "Expected either:\n"
        "  - ${VOCABULARY_FILE}\n"
        "  - ${VOCABULARY_ARCHIVE}\n"
        "Without the vocabulary, ORB_SLAM3 will build but crash at runtime when loading fails.\n"
        "Please add the vocabulary file/archive under Vocabulary/ and re-run CMake."
    )
endif()

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
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(g2o REQUIRED CONFIG)

# Build the main library
add_subdirectory(src)

# Unit tests
enable_testing()
add_subdirectory(src/CameraModels/tests)

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

# Install Thirdparty headers (Sophus only; g2o installs itself to include/g2o/)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM3/Thirdparty
    COMPONENT library
    FILES_MATCHING REGEX ".*\\.h(pp)?"
    PATTERN "Sophus" EXCLUDE
    PATTERN "g2o" EXCLUDE)

# Install Sophus headers to correct location
install(DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus/sophus/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM3/Thirdparty/Sophus/sophus
    COMPONENT library
    FILES_MATCHING PATTERN "*.hpp")

# g2o config.h and headers are installed by g2o itself to include/g2o/

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
