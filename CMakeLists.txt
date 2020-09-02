cmake_minimum_required(VERSION 3.5.1)

project(planning VERSION 1.0)
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})
include(GNUInstallDirs)

add_library(planning SHARED
    src/graph.cpp
    src/latlong_utm.cpp
    src/link.cpp
    src/node.cpp
    src/planning_graph.cpp
    src/position.cpp
    src/util.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${EIGEN_LIBRARIES})

set_target_properties(planning PROPERTIES VERSION ${PROJECT_VERSION})

target_include_directories(planning PRIVATE includes)
target_include_directories(planning PRIVATE src)


install(TARGETS planning EXPORT planningConfig
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}/${PROJECT_NAME})

install(DIRECTORY includes/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

install(EXPORT planningConfig DESTINATION share/planning/cmake)


export(TARGETS ${PROJECT_NAME} FILE planningConfig.cmake)
