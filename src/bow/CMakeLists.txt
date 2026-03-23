#
# bow — vocabulary factory + adapters (DBoW2/fbow)
#
add_library(${PROJECT_NAME}_bow STATIC
    BowVocabularyFactory.cc
    Dbow2VocabularyAdapter.cc
    FbowVocabularyAdapter.cc
)

target_include_directories(${PROJECT_NAME}_bow PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/ORB_SLAM3>
)

target_include_directories(${PROJECT_NAME}_bow PRIVATE
    ${fbow_INCLUDE_DIRS}
    ${fbow_INCLUDE_DIRS}/fbow
)

target_link_libraries(${PROJECT_NAME}_bow
    PUBLIC
        ${OpenCV_LIBS}
        DBoW2::DBoW2
        ${fbow_LIBS}
)

set_target_properties(${PROJECT_NAME}_bow PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

install(TARGETS ${PROJECT_NAME}_bow
    EXPORT  ORB_SLAM3Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT library
)

