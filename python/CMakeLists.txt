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

# Install destination must be relative, otherwise CMAKE_INSTALL_PREFIX is ignored.
include(GNUInstallDirs)

# Default to an in-prefix site-packages path (previous behavior), with an override.
set(_orbslam_py_default_install_dir
    "${CMAKE_INSTALL_LIBDIR}/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages"
)
set(ORBSLAM_PYTHON_INSTALL_DIR
    "${_orbslam_py_default_install_dir}"
    CACHE PATH
    "Install directory for orbslam3 Python module (relative to CMAKE_INSTALL_PREFIX)"
)

# If CMake reports a relative Python site-arch path, prefer it (keeps distro layouts).
if(DEFINED Python_SITEARCH AND NOT IS_ABSOLUTE "${Python_SITEARCH}")
    if(ORBSLAM_PYTHON_INSTALL_DIR STREQUAL "${_orbslam_py_default_install_dir}")
        set(ORBSLAM_PYTHON_INSTALL_DIR
            "${Python_SITEARCH}"
            CACHE PATH
            "Install directory for orbslam3 Python module (relative to CMAKE_INSTALL_PREFIX)"
            FORCE
        )
    endif()
elseif(DEFINED Python_SITEARCH AND IS_ABSOLUTE "${Python_SITEARCH}")
    message(STATUS
        "Python_SITEARCH is absolute (${Python_SITEARCH}); "
        "installing orbslam3 into ${ORBSLAM_PYTHON_INSTALL_DIR} under CMAKE_INSTALL_PREFIX"
    )
endif()

install(TARGETS ${TARGET_MODULE_NAME} 
        DESTINATION "${ORBSLAM_PYTHON_INSTALL_DIR}")
