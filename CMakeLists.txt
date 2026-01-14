cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3)

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
find_package(G2O REQUIRED)
find_package(realsense2)

# Build the main library
add_subdirectory(src)

# Build examples
# Examples and Examples_old directories have their own CMakeLists.txt files
# that collect their respective subdirectories

add_subdirectory(Examples)

# ---------------
# Installation instructions.
# ---------------
install(TARGETS DBoW2
    DESTINATION  lib
    COMPONENT library)

install(FILES ${G2O_LIBRARIES}
    DESTINATION lib
    COMPONENT library)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include/ORB_SLAM3
    COMPONENT library
    FILES_MATCHING PATTERN "*.h"
    PATTERN "Thirdparty" EXCLUDE)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/CameraModels/
    DESTINATION include/ORB_SLAM3/CameraModels
    COMPONENT library
    FILES_MATCHING PATTERN "*.h"
    PATTERN "Thirdparty" EXCLUDE)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/
    DESTINATION include/ORB_SLAM3/Thirdparty
    COMPONENT library
    FILES_MATCHING REGEX ".*\\.h(pp)?")
# ----------------------------------------------------------------------------
# Python Bindings (Optional)
# ----------------------------------------------------------------------------
if(BUILD_PYTHON_BINDINGS)
    message(STATUS "Configuring Python bindings...")    
    add_subdirectory(python) 
else()
    message(STATUS "Skipping Python bindings configuration.")
endif()