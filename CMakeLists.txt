cmake_minimum_required(VERSION 2.8.3)
project(cartesian_impedance_controller)

find_package(Boost 1.49 REQUIRED)
find_package(PkgConfig)

# Find RBDyn library and dependencies
pkg_search_module(Eigen3 REQUIRED eigen3)
pkg_check_modules(mc_rbdyn_urdf REQUIRED mc_rbdyn_urdf)
pkg_check_modules(RBDyn REQUIRED RBDyn)
pkg_check_modules(SpaceVecAlg REQUIRED SpaceVecAlg)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  controller_interface
  controller_manager
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  trajectory_msgs
)

add_message_files(FILES
  ControllerConfig.msg
  ControllerState.msg
)

generate_messages(
DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/damping.cfg
  cfg/stiffness.cfg
  cfg/wrench.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs controller_interface controller_manager dynamic_reconfigure eigen_conversions geometry_msgs hardware_interface message_runtime pluginlib realtime_tools roscpp sensor_msgs std_msgs tf tf_conversions trajectory_msgs
  DEPENDS Boost Eigen3 mc_rbdyn_urdf RBDyn SpaceVecAlg tinyxml2
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_ros
)

###########
## Build ##
###########

# Sets build type to "Release" in case no build type has not been set before. This is necessary to run this controller at 1 kHz.
# On our RT system with an i5 11th Gen processor this reduces update evaluations from about 800 to 300 microseconds
if(NOT CMAKE_BUILD_TYPE) 
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# Core library
add_library(${PROJECT_NAME} src/cartesian_impedance_controller.cpp)
add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_compile_options(${PROJECT_NAME} PUBLIC -std=c++14)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${SpaceVecAlg_INCLUDE_DIRS} ${RBDyn_INCLUDE_DIRS} ${mc_rbdyn_urdf_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Eigen3_LIBRARIES})

# ROS Integration
add_library(${PROJECT_NAME}_ros src/cartesian_impedance_controller_ros.cpp)
add_dependencies(
  ${PROJECT_NAME}_ros
  ${${PROJECT_NAME}_ros_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_compile_options(${PROJECT_NAME}_ros PUBLIC -std=c++14)
target_include_directories(${PROJECT_NAME}_ros PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${tinyxml2_INCLUDE_DIRS} ${SpaceVecAlg_INCLUDE_DIRS} ${RBDyn_INCLUDE_DIRS} ${mc_rbdyn_urdf_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_ros PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${tinyxml2_LIBRARIES} RBDyn mc_rbdyn_urdf ${Eigen3_LIBRARIES})


## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
