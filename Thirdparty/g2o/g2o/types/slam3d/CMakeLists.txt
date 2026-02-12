add_library(types_slam3d ${G2O_LIB_TYPE}
  dquat2mat.cpp dquat2mat.h
  isometry3d_mappings.h
  isometry3d_mappings.cpp
  isometry3d_gradients.cpp
  isometry3d_gradients.h
  vertex_se3.cpp
  vertex_se3.h
  parameter_se3_offset.cpp
  parameter_se3_offset.h
  edge_se3.cpp
  edge_se3.h
  edge_se3_offset.cpp
  edge_se3_offset.h
  vertex_pointxyz.cpp
  vertex_pointxyz.h
  parameter_camera.cpp
  parameter_camera.h
  parameter_stereo_camera.cpp
  parameter_stereo_camera.h
  edge_se3_pointxyz.cpp
  edge_se3_pointxyz.h
  edge_se3_pointxyz_disparity.cpp
  edge_se3_pointxyz_disparity.h
  edge_se3_pointxyz_depth.cpp
  edge_se3_pointxyz_depth.h
  edge_se3_prior.cpp
  edge_se3_prior.h
  edge_se3_lotsofxyz.cpp
  edge_se3_lotsofxyz.h
  se3quat.h
  se3_ops.h se3_ops.hpp
  edge_pointxyz.cpp edge_pointxyz.h
  types_slam3d.cpp
  types_slam3d.h
)

set_target_properties(types_slam3d PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_slam3d)

target_include_directories(types_slam3d PUBLIC
  $<BUILD_INTERFACE:${G2O_SOURCE_DIR}>
  $<BUILD_INTERFACE:${G2O_BUILD_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(types_slam3d PUBLIC core)

if(BUILD_TESTING)
  add_executable(test_isometry3d_mappings test_isometry3d_mappings.cpp)
  target_link_libraries(test_isometry3d_mappings PRIVATE types_slam3d)

  add_executable(test_slam3d_jacobian test_slam3d_jacobian.cpp)
  target_link_libraries(test_slam3d_jacobian PRIVATE types_slam3d)
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/g2o/types/slam3d
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
