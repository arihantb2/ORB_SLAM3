cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3 VERSION 0.0.1)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(ORB_SLAM3_VERSION ${PROJECT_VERSION})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})
option(BUILD_PYTHON_BINDINGS "Build Python bindings for ORB_SLAM3" ON)
message(STATUS "Build Python bindings: ${BUILD_PYTHON_BINDINGS}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_BUILD_TYPE MATCHES "^[Dd][Ee][Bb][Uu][Gg]$")
    add_compile_options(-Wall -O0 -g -march=native -fstack-protector-all)
else()
    add_compile_options(-Wall -O3 -march=native)
endif()

# ==============================================================================
# Build dependencies as subdirectories
# ==============================================================================

message(STATUS "Configuring and building Thirdparty/DBoW2 ...")
add_subdirectory(Thirdparty/DBoW2)

message(STATUS "Configuring and building Thirdparty/g2o ...")
add_subdirectory(Thirdparty/g2o)

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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules")

# Set output directories for the main project
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)

# g2o is always built internally from Thirdparty/g2o
# Set up g2o variables based on build tree
message(STATUS "Using g2o from build tree (Thirdparty/g2o)")

# Collect g2o libraries from targets (only those that exist, as some are conditionally built)
set(G2O_LIBRARIES "")
foreach(TARGET_NAME stuff core types_slam3d types_sba types_sim3 solver_dense solver_eigen)
    if(TARGET ${TARGET_NAME})
        list(APPEND G2O_LIBRARIES ${TARGET_NAME})
    endif()
endforeach()

find_package(realsense2)

# Build the main library
add_subdirectory(src)

# Build examples
# Examples and Examples_old directories have their own CMakeLists.txt files
# that collect their respective subdirectories

# add_subdirectory(Examples)

# ---------------
# Installation instructions.
# ---------------
# Install Thirdparty libraries
install(TARGETS DBoW2
    EXPORT ORB_SLAM3Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT library)

# Install g2o libraries from build tree
# Note: Target names are without prefix, but OUTPUT_NAME includes g2o_ prefix
# Collect only the targets that actually exist (some may be conditionally built)
set(G2O_INSTALL_TARGETS "")
foreach(TARGET_NAME stuff core types_slam3d types_sba types_sim3 solver_dense solver_eigen)
    if(TARGET ${TARGET_NAME})
        list(APPEND G2O_INSTALL_TARGETS ${TARGET_NAME})
    endif()
endforeach()

if(G2O_INSTALL_TARGETS)
    # Set up INTERFACE_INCLUDE_DIRECTORIES for all g2o targets
    # This ensures consumers can use g2o targets directly with proper includes
    foreach(TARGET_NAME ${G2O_INSTALL_TARGETS})
        if(TARGET ${TARGET_NAME})
            target_include_directories(${TARGET_NAME} INTERFACE
                $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/g2o>
                $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/Thirdparty/g2o>
                $<INSTALL_INTERFACE:include/ORB_SLAM3/Thirdparty/g2o>
            )
        endif()
    endforeach()
    
    install(TARGETS ${G2O_INSTALL_TARGETS}
        EXPORT ORB_SLAM3Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT library)
endif()

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

# Install Thirdparty headers (DBoW2, g2o source headers)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM3/Thirdparty
    COMPONENT library
    FILES_MATCHING REGEX ".*\\.h(pp)?"
    PATTERN "Sophus" EXCLUDE)

# Install Sophus headers to correct location
install(DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus/sophus/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM3/Thirdparty/Sophus/sophus
    COMPONENT library
    FILES_MATCHING PATTERN "*.hpp")

# Install generated g2o config.h to the correct location under ORB_SLAM3
install(FILES ${CMAKE_BINARY_DIR}/Thirdparty/g2o/g2o/config.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM3/Thirdparty/g2o/g2o
    COMPONENT library)

install(FILES ${PROJECT_SOURCE_DIR}/cmake_modules/FindORB_SLAM3.cmake
    DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake/${PROJECT_NAME})

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
# ----------------------------------------------------------------------------
# Python Bindings (Optional)
# ----------------------------------------------------------------------------
if(BUILD_PYTHON_BINDINGS)
    message(STATUS "Configuring Python bindings...")    
    add_subdirectory(python) 
else()
    message(STATUS "Skipping Python bindings configuration.")
endif()