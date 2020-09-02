cmake_minimum_required(VERSION 2.6)
 
# Locate GTest

find_package(GTest REQUIRED)
find_package (Eigen3 REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(../../../includes)
include_directories(../../../src)

# Link runTests with what we want to test and the GTest and pthread library
add_executable(planning_graph_test ../planning_graph_test.cpp)
target_link_libraries(planning_graph_test ${EIGEN_LIBRARIES} ${GTEST_LIBRARIES} pthread )

add_executable(util_test ../util_test.cpp)
target_link_libraries(util_test ${GTEST_LIBRARIES} pthread )

add_executable(graph_test ../graph_test.cpp)
target_link_libraries(graph_test ${EIGEN_LIBRARIES} ${GTEST_LIBRARIES} pthread )

add_executable(node_test ../node_test.cpp)
target_link_libraries(node_test ${EIGEN_LIBRARIES} ${GTEST_LIBRARIES} pthread )


