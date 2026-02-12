add_library(stuff ${G2O_LIB_TYPE}
  filesys_tools.h    string_tools.h
  color_macros.h     macros.h         timeutil.cpp
  command_args.cpp   misc.h           sparse_helper.cpp  timeutil.h
  command_args.h     sparse_helper.h
  filesys_tools.cpp  os_specific.h    string_tools.cpp
  property.cpp       property.h
  sampler.cpp        sampler.h        unscented.h
  tictoc.cpp tictoc.h
  g2o_stuff_api.h
)

set_target_properties(stuff PROPERTIES OUTPUT_NAME ${LIB_PREFIX}stuff)

target_include_directories(stuff PUBLIC
  $<BUILD_INTERFACE:${G2O_SOURCE_DIR}>
  $<BUILD_INTERFACE:${G2O_BUILD_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(stuff PUBLIC Eigen3::Eigen)

if(CMAKE_SYSTEM_NAME MATCHES "Linux" AND NOT ANDROID)
  target_link_libraries(stuff PRIVATE rt)
endif()

# Headers to include/g2o/; target install done by consumer (e.g. ORB_SLAM3) when bundling
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/g2o/stuff
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
