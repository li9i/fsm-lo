cmake_minimum_required(VERSION 2.8.3)
project(fsm_lidom_ros)

SET(CMAKE_CXX_COMPILER /usr/bin/g++)
add_compile_options(-std=c++11 -Ofast)

#find_package(CGAL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  CGAL
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
  geometry_msgs
  message_runtime
  INCLUDE_DIRS include
)

INCLUDE_DIRECTORIES(include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3/)

# sm ###########################################################################
add_library(fsm_lidom src/fsm.cpp)
target_link_libraries(fsm_lidom
  fftw3
  CGAL::CGAL_Core
  ${catkin_LIBRARIES})
add_dependencies(fsm_lidom
  ${catkin_EXPORTED_TARGETS})

# The executable ###############################################################
add_executable(fsm_node
  src/fsm_node.cpp)

target_link_libraries(fsm_node
  fsm_lidom
  ${catkin_LIBRARIES})

add_dependencies(fsm_node
  ${catkin_EXPORTED_TARGETS}
)
