cmake_minimum_required(VERSION 2.8.3)
project(bh_gazebo)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
)
find_package(gazebo REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
)

# Gazebo plugin
add_library(gazebo_floating_joint_plugin SHARED src/bh_gazebo/floating_joint_plugin.cpp)
target_link_libraries(gazebo_floating_joint_plugin ${GAZEBO_LIBRARIES})