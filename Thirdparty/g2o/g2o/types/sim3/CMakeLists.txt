add_library(types_sim3 ${G2O_LIB_TYPE}
  types_seven_dof_expmap.cpp
  types_seven_dof_expmap.h
)

set_target_properties(types_sim3 PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sim3)

target_include_directories(types_sim3 PUBLIC
  $<BUILD_INTERFACE:${G2O_SOURCE_DIR}>
  $<BUILD_INTERFACE:${G2O_BUILD_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(types_sim3 PUBLIC types_sba)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/g2o/types/sim3
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
