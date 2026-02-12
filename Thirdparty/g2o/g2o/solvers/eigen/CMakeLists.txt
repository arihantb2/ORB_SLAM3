add_library(solver_eigen ${G2O_LIB_TYPE}
  solver_eigen.cpp
  linear_solver_eigen.h
)

set_target_properties(solver_eigen PROPERTIES OUTPUT_NAME ${LIB_PREFIX}solver_eigen)

target_include_directories(solver_eigen PUBLIC
  $<BUILD_INTERFACE:${G2O_SOURCE_DIR}>
  $<BUILD_INTERFACE:${G2O_BUILD_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(solver_eigen PUBLIC core)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/g2o/solvers/eigen
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
