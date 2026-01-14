# CMakeLists.txt for ORB_SLAM2 Python Bindings

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# 1. Find the Python interpreter and development headers first
# This will detect 3.12 automatically and set the correct variables
find_package(Python 3.11 COMPONENTS Interpreter Development NumPy REQUIRED)

# 2. Find Boost Python using the version detected above
# Using "python" instead of "python311" allows CMake to find the best match
find_package(Boost 1.80.0 REQUIRED COMPONENTS python)

# RPATH fiddling
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(TARGET_MODULE_NAME orbslam3)
add_library(${TARGET_MODULE_NAME} SHARED
    src/ORBSlamPython.cpp
    src/pyboost_cv4_converter.cpp
)

target_include_directories(${TARGET_MODULE_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Python_INCLUDE_DIRS}        # Updated variable name
    ${Python_NumPy_INCLUDE_DIRS}  # Updated variable name
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
)

set_target_properties(${TARGET_MODULE_NAME} PROPERTIES PREFIX "")

target_link_libraries(${TARGET_MODULE_NAME} 
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Boost_LIBRARIES}
    ${Python_LIBRARIES}           # Updated variable name
    ${Pangolin_LIBRARIES}
)

# Use the dynamic site-packages path instead of hardcoded 3.11
install(TARGETS ${TARGET_MODULE_NAME} 
        DESTINATION ${Python_SITEARCH})