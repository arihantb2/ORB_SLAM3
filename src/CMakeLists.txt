# feature_extractor sub-library (FeatureExtractor base class + ORBFeatureExtractor)
# Must be added before the main library so the target exists when linked below.
add_subdirectory(feature_extractor)

# ORB_SLAM3 library

add_library(${PROJECT_NAME} SHARED
    CameraCalibrationInput.cc
    System.cc
    Tracking.cc
    LocalMapping.cc
    # ORBextractor.cc is retired — logic now lives in feature_extractor
    ORBmatcher.cc
    Converter.cc
    MapPoint.cc
    KeyFrame.cc
    Atlas.cc
    Map.cc
    MapDrawer.cc
    Optimizer.cc
    Frame.cc
    Viewer.cc
    GTSAMTypes.cc
    CameraModels/Pinhole.cpp
    CameraModels/Metashape.cpp
    MLPnPsolver.cpp
    GeometricTools.cc
    TwoViewReconstruction.cc
    Settings.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/Sophus>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/ORB_SLAM3>
    $<INSTALL_INTERFACE:include/ORB_SLAM3/Thirdparty/Sophus>
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${GTSAM_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    feature_extractor
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    gtsam
    DBoW2::DBoW2
    -lcrypto
)

# If RealSense SDK is found the library is added
if(realsense2_FOUND)
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
        ${realsense2_LIBRARY}
    )
endif()

# Install the library
install(TARGETS ${PROJECT_NAME}
    EXPORT ORB_SLAM3Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT library
)
