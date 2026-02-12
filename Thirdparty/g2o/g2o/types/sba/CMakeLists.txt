add_library(types_sba ${G2O_LIB_TYPE}
  types_sba.h     types_six_dof_expmap.h
  types_sba.cpp   types_six_dof_expmap.cpp
  g2o_types_sba_api.h
)

set_target_properties(types_sba PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types_sba)

target_include_directories(types_sba PUBLIC
  $<BUILD_INTERFACE:${G2O_SOURCE_DIR}>
  $<BUILD_INTERFACE:${G2O_BUILD_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(types_sba PUBLIC core types_slam3d)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/g2o/types/sba
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
