cmake_minimum_required(VERSION 3.16)
project(cartesian_impedance_controller)

# Compiler warnings (for GNU and Clang)
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -Werror=conversion -Werror=unused-but-set-variable
    -Werror=return-type -Werror=shadow -Werror=format
    -Werror=range-loop-construct -Werror=missing-braces
  )
endif()

# Export all symbols on Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  control_msgs
  sensor_msgs
  control_toolbox
  controller_interface
  geometry_msgs
  joint_trajectory_controller
  hardware_interface
  kinematics_interface
  rclcpp_lifecycle
  rclcpp_action
  realtime_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  std_msgs
  trajectory_msgs
  generate_parameter_library
  urdf
  controller_manager
  ros2_control
)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(RBDyn REQUIRED)
find_package(mc_rbdyn_urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerConfig.msg"
  "msg/ControllerState.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

set(library_name cartesian_impedance_controller_lib)

generate_parameter_library(cartesian_impedance_controller_parameters
  src/cartesian_impedance_controller_parameters.yaml
)

add_library(${library_name} SHARED
  src/cartesian_impedance_controller_ros.cpp
  src/cartesian_impedance_controller.cpp
)

target_compile_features(${library_name} PUBLIC cxx_std_17)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp> 
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${library_name} PUBLIC
  "${cpp_typesupport_target}"
  cartesian_impedance_controller_parameters
  Eigen3::Eigen
  RBDyn
  mc_rbdyn_urdf::mc_rbdyn_urdf
)

pluginlib_export_plugin_description_file(controller_interface cartesian_impedance_controller_plugin.xml)

ament_target_dependencies(${library_name} PUBLIC 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}  
)

install(
  DIRECTORY include/
  DESTINATION include/cartesian_impedance_controller
)

install(
  TARGETS ${library_name} cartesian_impedance_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
