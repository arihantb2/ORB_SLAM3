find_package(lcm_log_player REQUIRED)
find_package(acfr_lcm_types REQUIRED)
find_package(static_tf REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# --- VO base class library ---
add_library(vo SHARED
    src/visual_odometry.cpp
)

target_include_directories(vo PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/ORB_SLAM3>
)

target_link_libraries(vo PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    static_tf::static_tf
    acfr_lcm_types::acfr_lcm_types
)

install(TARGETS vo
    EXPORT ORB_SLAM3Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

enable_testing()
add_executable(vo_tests src/test/visual_odometry_tests.cpp)
target_link_libraries(vo_tests PRIVATE vo)
add_test(NAME vo_tests COMMAND vo_tests)

# At build time, ORB_SLAM3 library headers live at ${PROJECT_SOURCE_DIR}/include/Foo.h
# (no ORB_SLAM3/ subdirectory). Create a symlink so the wrapper can use the
# installed-style <ORB_SLAM3/Foo.h> includes unchanged.
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/compat_include)
file(CREATE_LINK
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/compat_include/ORB_SLAM3
    SYMBOLIC)

# --- Wrapper executable ---
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${diagnostic_msgs_INCLUDE_DIRS}
)

set(WRAPPER_SOURCES
    src/main.cpp
    src/orb_slam3_wrapper.cpp
    src/tracking_ros_publisher.cpp
    src/local_mapping_ros_publisher.cpp
    src/local_mapping_visualization_publisher.cpp
)

add_executable(orb_slam3_wrapper_main ${WRAPPER_SOURCES})

target_include_directories(orb_slam3_wrapper_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/compat_include
)

target_link_libraries(orb_slam3_wrapper_main
    vo
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    Boost::program_options
    lcm_log_player::lcm_log_player
    acfr_lcm_types::acfr_lcm_types
)

ament_target_dependencies(orb_slam3_wrapper_main
    rclcpp
    tf2
    tf2_ros
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    diagnostic_msgs
)

install(TARGETS orb_slam3_wrapper_main
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
