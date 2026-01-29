cmake_minimum_required(VERSION 3.4)
project(Sophus VERSION 1.1.0)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Release by default
# Turn on Debug with "-DCMAKE_BUILD_TYPE=Debug"
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)

# Set compiler specific settings (FixMe: Should not cmake do this for us automatically?)
IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
   SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
   SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra  -Wno-deprecated-register -Qunused-arguments -fcolor-diagnostics")
ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
   SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g")
   SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -std=c++11 -Wno-deprecated-declarations -ftemplate-backtrace-limit=0")
   SET(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_DEBUG} --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
   SET(CMAKE_EXE_LINKER_FLAGS_COVERAGE "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --coverage")
   SET(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} --coverage")
ELSEIF(CMAKE_CXX_COMPILER_ID MATCHES "^MSVC$")
   ADD_DEFINITIONS("-D _USE_MATH_DEFINES /bigobj /wd4305 /wd4244 /MP")
ENDIF()

# Find Eigen 3 (dependency)
find_package(Eigen3 3.3.0 REQUIRED)

# Define interface library target
add_library(sophus INTERFACE)

set(SOPHUS_HEADER_FILES
  sophus/average.hpp
  sophus/common.hpp
  sophus/geometry.hpp
  sophus/interpolate.hpp
  sophus/interpolate_details.hpp
  sophus/num_diff.hpp
  sophus/rotation_matrix.hpp
  sophus/rxso2.hpp
  sophus/rxso3.hpp
  sophus/se2.hpp
  sophus/se3.hpp
  sophus/sim2.hpp
  sophus/sim3.hpp
  sophus/sim_details.hpp
  sophus/so2.hpp
  sophus/so3.hpp
  sophus/types.hpp
  sophus/velocities.hpp
  sophus/formatstring.hpp
)

set(SOPHUS_OTHER_FILES
  sophus/test_macros.hpp
  sophus/example_ensure_handler.cpp
)

if(MSVC)
  # Define common math constants if we compile with MSVC
  target_compile_definitions (sophus INTERFACE _USE_MATH_DEFINES)
endif (MSVC)

# Add Eigen interface dependency, depending on available cmake info
if(TARGET Eigen3::Eigen)
  target_link_libraries(sophus INTERFACE Eigen3::Eigen)
  set(Eigen3_DEPENDENCY "find_dependency (Eigen3 ${Eigen3_VERSION})")
else(TARGET Eigen3::Eigen)
  target_include_directories (sophus SYSTEM INTERFACE ${EIGEN3_INCLUDE_DIR})
endif(TARGET Eigen3::Eigen)

# Associate target with include directory
# Install interface is managed by parent ORB_SLAM3 project
target_include_directories(sophus INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include/ORB_SLAM3/Thirdparty/Sophus>"
)

# Declare all used C++11 features
target_compile_features (sophus INTERFACE
  cxx_auto_type
  cxx_decltype
  cxx_nullptr
  cxx_right_angle_brackets
  cxx_variadic_macros
  cxx_variadic_templates
)

# Add sources as custom target so that they are shown in IDE's
add_custom_target(other SOURCES ${SOPHUS_OTHER_FILES})

# Create 'test' make target using ctest
option(BUILD_TESTS "Build tests." ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# Create examples make targets using ctest
option(BUILD_EXAMPLES "Build examples." ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Export package for use from the build tree (for build-tree usage only)
# Installation is handled by the parent ORB_SLAM3 CMakeLists.txt
set_target_properties(sophus PROPERTIES EXPORT_NAME Sophus)

export(TARGETS sophus NAMESPACE Sophus:: FILE SophusTargets.cmake)
# Note: install commands are disabled - handled by parent ORB_SLAM3 project
