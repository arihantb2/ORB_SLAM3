cmake_minimum_required(VERSION 3.16)
project(g2o VERSION 0.0.1 LANGUAGES CXX C)

include(GNUInstallDirs)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Shared libs; RPATH set for install
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
set(G2O_LIB_TYPE SHARED)
set(LIB_PREFIX g2o_)

find_package(Eigen3 REQUIRED)

# Use the build dir for this subdirectory (not top-level) so config.h and source tree resolve consistently.
# CACHE INTERNAL so all g2o subdirs see it when used in target_include_directories.
set(G2O_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "g2o source directory")
set(G2O_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "g2o build directory")
configure_file(config.h.in ${G2O_BUILD_DIR}/g2o/config.h @ONLY)

# RPATH for shared libs when installed
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_subdirectory(g2o)

install(FILES ${G2O_BUILD_DIR}/g2o/config.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/g2o/g2o)

# Convenience target — all g2o libs + includes (plain name; :: is reserved for CMake)
add_library(g2o_interface INTERFACE)
target_link_libraries(g2o_interface INTERFACE stuff core types_slam3d types_sba types_sim3 solver_dense solver_eigen)
target_include_directories(g2o_interface INTERFACE
  $<BUILD_INTERFACE:${G2O_SOURCE_DIR}>
  $<BUILD_INTERFACE:${G2O_BUILD_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
add_library(g2o::g2o ALIAS g2o_interface)
