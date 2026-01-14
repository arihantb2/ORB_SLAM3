# ORB_SLAM3 library

add_library(${PROJECT_NAME} SHARED
    System.cc
    Tracking.cc
    LocalMapping.cc
    LoopClosing.cc
    ORBextractor.cc
    ORBmatcher.cc
    FrameDrawer.cc
    Converter.cc
    MapPoint.cc
    KeyFrame.cc
    Atlas.cc
    Map.cc
    MapDrawer.cc
    Optimizer.cc
    Frame.cc
    KeyFrameDatabase.cc
    Sim3Solver.cc
    Viewer.cc
    ImuTypes.cc
    G2oTypes.cc
    CameraModels/Pinhole.cpp
    CameraModels/KannalaBrandt8.cpp
    OptimizableTypes.cpp
    MLPnPsolver.cpp
    GeometricTools.cc
    TwoViewReconstruction.cc
    Config.cc
    Settings.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC 
    ${PROJECT_SOURCE_DIR}/include 
    ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus 
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
    ${G2O_INCLUDE_DIRS} 
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${G2O_LIBRARIES}
    DBoW2
    -lboost_serialization
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
    DESTINATION lib
    COMPONENT library
)
