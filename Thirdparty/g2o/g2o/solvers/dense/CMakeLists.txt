add_library(solver_dense ${G2O_LIB_TYPE}
  solver_dense.cpp linear_solver_dense.h
)

set_target_properties(solver_dense PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_dense)

target_include_directories(solver_dense PUBLIC
  $<BUILD_INTERFACE:${G2O_SOURCE_DIR}>
  $<BUILD_INTERFACE:${G2O_BUILD_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(solver_dense PUBLIC core)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/g2o/solvers/dense
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
