cmake_minimum_required(VERSION 3.5.1)

project(planning VERSION 1.0)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${Eigen3_INCLUDE_DIRS})
include(GNUInstallDirs)

add_library(planning SHARED
    src/Graph.cpp
    src/LatLongToUTM.cpp
    src/Link.cpp
    src/Node.cpp
    src/Planning_Graph.cpp
    src/Position.cpp
    src/Util.cpp
)

target_link_libraries(planning Eigen3::Eigen ${GTEST_LIBRARIES} pthread )

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
