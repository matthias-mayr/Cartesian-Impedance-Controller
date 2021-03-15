cmake_minimum_required(VERSION 2.8.3)
project(cartesian_impedance_controller)

option(ENABLE_SIMD "Build with all SIMD instructions on the current local machine" ON)

find_package(PkgConfig)

# Find RBDyn library and dependencies
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Boost 1.49 REQUIRED)

pkg_check_modules(tinyxml2 REQUIRED tinyxml2)
pkg_check_modules(SpaceVecAlg REQUIRED SpaceVecAlg)
pkg_check_modules(RBDyn REQUIRED RBDyn)
pkg_check_modules(mc_rbdyn_urdf REQUIRED mc_rbdyn_urdf)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  pluginlib
  controller_interface
  hardware_interface
  iiwa_tools
  eigen_conversions
  std_msgs
  controller_manager
  urdf
  tf
  tf_conversions
  actionlib
  actionlib_msgs
  trajectory_msgs
  control_msgs
  message_generation
  dynamic_reconfigure
)

add_message_files(FILES
CartesianImpedanceControlMode.msg
CartesianQuantity.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/impedance_config.cfg
)

catkin_package(
     INCLUDE_DIRS include
     #CATKIN_DEPENDS rospy actionlib roscpp pluginlib controller_interface hardware_interface iiwa_tools eigen_conversions std_msgs controller_manager urdf tf tf_conversions actionlib actionlib_msgs trajectory_msgs control_msgs
     DEPENDS Boost tinyxml2 SpaceVecAlg RBDyn mc_rbdyn_urdf Eigen3
     LIBRARIES ${CMAKE_PROJECT_NAME}
)

###########
## Build ##
###########
add_library(${CMAKE_PROJECT_NAME} src/cartesian_impedance_controller.cpp src/cartesian_impedance_controller_base.cpp)

add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

# Yes, this is necessary. Otherwise the control plugin crashes.
if(ENABLE_SIMD)
  if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    set(CXX_COMPILER_VERSION ${GCC_VERSION})
    target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC -march=native)
    if(GCC_VERSION VERSION_GREATER 7.0)
      target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC -faligned-new)
    endif()
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC -march=native -faligned-new)
  endif()
endif()

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC -std=c++11)
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${tinyxml2_INCLUDE_DIRS} ${SpaceVecAlg_INCLUDE_DIRS} ${RBDyn_INCLUDE_DIRS} ${mc_rbdyn_urdf_INCLUDE_DIRS})
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${tinyxml2_LIBRARIES} ${SpaceVecAlg_LIBRARIES} ${RBDyn_LIBRARIES} ${mc_rbdyn_urdf_LIBRARIES} ${Eigen3_LIBRARIES})
