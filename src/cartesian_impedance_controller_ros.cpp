#include <cartesian_impedance_controller/cartesian_impedance_controller_ros.hpp>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/helpers.hpp>
#include <controller_interface/controller_interface_base.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(29, 0, 0)
#include "urdf/model.hpp"
#else
#include "urdf/model.h"
#endif

namespace cartesian_impedance_controller
{

using CallbackReturn = controller_interface::CallbackReturn;
using namespace std::chrono_literals;

CartesianImpedanceControllerRos::CartesianImpedanceControllerRos()
  : controller_interface::ControllerInterface(), traj_duration_(0, 0)
{
  // The Eigen vectors will be resized in on_configure.
  q_.resize(0);
  dq_.resize(0);
  tau_m_.resize(0);
  tau_c_.resize(0);
}

CartesianImpedanceControllerRos::~CartesianImpedanceControllerRos()
{
}

CallbackReturn CartesianImpedanceControllerRos::on_init()
{
  auto node = get_node();
  auto logger = node->get_logger();

  try
  {
    parameter_handler_ = std::make_shared<ParamListener>(node);
    params_ = parameter_handler_->get_params();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  std::string urdf_string;
  if (!node->get_parameter("robot_description", urdf_string))
  {
    RCLCPP_ERROR(logger, "Failed to get robot_description parameter");
    return CallbackReturn::ERROR;
  }
  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianImpedanceControllerRos::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& jn : params_.joints)
  {
    conf.names.push_back(jn + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

controller_interface::InterfaceConfiguration CartesianImpedanceControllerRos::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& jn : params_.joints)
  {
    conf.names.push_back(jn + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(jn + "/" + hardware_interface::HW_IF_VELOCITY);
    conf.names.push_back(jn + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

CallbackReturn CartesianImpedanceControllerRos::on_configure(const rclcpp_lifecycle::State&)
{
  auto node = get_node();
  auto logger = node->get_logger();

  if (!parameter_handler_)
  {
    RCLCPP_ERROR(logger, "Parameter handler not initialized.");
    return CallbackReturn::ERROR;
  }

  rt_trajectory_ = std::make_shared<realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory>>();
  parameter_handler_->refresh_dynamic_parameters();
  params_ = parameter_handler_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(logger, "No 'joints' parameter specified. Aborting configure.");
    return CallbackReturn::ERROR;
  }
  dof_ = params_.joints.size();
  RCLCPP_INFO(logger, "Found %zu joints.", dof_);

  q_ = Eigen::VectorXd::Zero(dof_);
  dq_ = Eigen::VectorXd::Zero(dof_);
  tau_m_ = Eigen::VectorXd::Zero(dof_);
  tau_c_ = Eigen::VectorXd::Zero(dof_);
  command_joint_names_ = params_.joints;

  end_effector_ = params_.end_effector;
  wrench_ee_frame_ = params_.wrench_ee_frame;
  delta_tau_max_ = params_.delta_tau_max;
  update_frequency_ = params_.update_frequency;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_last_time_ = get_node()->now();

  std::stringstream joints_ss;
  joints_ss << "Joints: ";
  for (const auto& joint : params_.joints)
  {
    joints_ss << joint << " ";
  }
  RCLCPP_INFO(logger, "%s", joints_ss.str().c_str());
  RCLCPP_INFO(logger, "End Effector: %s", params_.end_effector.c_str());
  RCLCPP_INFO(logger, "Wrench EE Frame: %s", params_.wrench_ee_frame.c_str());
  RCLCPP_INFO(logger, "Update Frequency: %.2f", params_.update_frequency);
  RCLCPP_INFO(logger, "Filtering parameters - Nullspace: %.2f, Stiffness: %.2f, Pose: %.2f, Wrench: %.2f",
              params_.filtering.nullspace, params_.filtering.stiffness, params_.filtering.pose,
              params_.filtering.wrench);
  RCLCPP_INFO(logger, "Stiffness - Translation: (%.2f, %.2f, %.2f), Rotation: (%.2f, %.2f, %.2f), Nullspace: %.2f",
              params_.stiffness.translation.x, params_.stiffness.translation.y, params_.stiffness.translation.z,
              params_.stiffness.rotation.x, params_.stiffness.rotation.y, params_.stiffness.rotation.z,
              params_.stiffness.nullspace_stiffness);
  RCLCPP_INFO(logger,
              "Damping - Translation: (%.2f, %.2f, %.2f), Rotation: (%.2f, %.2f, %.2f), Nullspace: %.2f, Update: %s",
              params_.damping.translation.x, params_.damping.translation.y, params_.damping.translation.z,
              params_.damping.rotation.x, params_.damping.rotation.y, params_.damping.rotation.z,
              params_.damping.nullspace_damping, params_.damping.update_damping_factors ? "true" : "false");

  trajectory_sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "joint_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianImpedanceControllerRos::trajCb, this, std::placeholders::_1));

  traj_as_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      node->get_node_base_interface(), node->get_node_clock_interface(), node->get_node_logging_interface(),
      node->get_node_waitables_interface(), std::string(node->get_name()) + "/follow_joint_trajectory",
      std::bind(&CartesianImpedanceControllerRos::trajGoalCb, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CartesianImpedanceControllerRos::trajCancelCb, this, std::placeholders::_1),
      std::bind(&CartesianImpedanceControllerRos::trajAcceptCb, this, std::placeholders::_1));
  
  auto state_pub =
      get_node()->create_publisher<cartesian_impedance_controller::msg::ControllerState>("controller_state", 10);
  auto torques_pub = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("commanded_torques", 10);

  rt_pub_state_ =
      std::make_shared<realtime_tools::RealtimePublisher<cartesian_impedance_controller::msg::ControllerState>>(
          state_pub);
  rt_pub_torques_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(torques_pub);

  sub_controller_config_ = get_node()->create_subscription<cartesian_impedance_controller::msg::ControllerConfig>(
      "set_config",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianImpedanceControllerRos::controllerConfigCb, this, std::placeholders::_1));

  sub_cart_stiffness_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "set_cartesian_stiffness", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianImpedanceControllerRos::cartesianStiffnessCb, this, std::placeholders::_1));

  sub_cart_wrench_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "set_cartesian_wrench", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianImpedanceControllerRos::wrenchCommandCb, this, std::placeholders::_1));

  sub_damping_factors_ = node->create_subscription<geometry_msgs::msg::Wrench>(
      "set_damping_factors", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianImpedanceControllerRos::cartesianDampingFactorCb, this, std::placeholders::_1));

  sub_reference_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "reference_pose", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianImpedanceControllerRos::referencePoseCb, this, std::placeholders::_1));

  setNumberOfJoints(dof_);

  if (!initRBDyn())
  {
    RCLCPP_ERROR(logger, "Failed to initialize RBDyn. Check robot_description param!");
    return CallbackReturn::ERROR;
  }

  double filter_ns = params_.filtering.nullspace;
  double filter_stiff = params_.filtering.stiffness;
  double filter_pose = params_.filtering.pose;
  double filter_wrench = params_.filtering.wrench;
  setFiltering(update_frequency_, filter_ns, filter_stiff, filter_pose, filter_wrench);

  double stiff_tx = params_.stiffness.translation.x;
  double stiff_ty = params_.stiffness.translation.y;
  double stiff_tz = params_.stiffness.translation.z;
  double stiff_rx = params_.stiffness.rotation.x;
  double stiff_ry = params_.stiffness.rotation.y;
  double stiff_rz = params_.stiffness.rotation.z;
  double stiff_ns = params_.stiffness.nullspace_stiffness;

  double damp_tx = params_.damping.translation.x;
  double damp_ty = params_.damping.translation.y;
  double damp_tz = params_.damping.translation.z;
  double damp_rx = params_.damping.rotation.x;
  double damp_ry = params_.damping.rotation.y;
  double damp_rz = params_.damping.rotation.z;
  double damp_ns = params_.damping.nullspace_damping;

  if (params_.damping.update_damping_factors)
    setDampingFactors(damp_tx, damp_ty, damp_tz, damp_rx, damp_ry, damp_rz, damp_ns);

  damping_factors_.resize(7);
  damping_factors_ << damp_tx, damp_ty, damp_tz, damp_rx, damp_ry, damp_rz, damp_ns;
  nullspace_stiffness_target_ = stiff_ns;

  RCLCPP_INFO(logger, "on_configure() successful.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceControllerRos::on_activate(const rclcpp_lifecycle::State&)
{
  auto node = get_node();
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "Activating Cartesian Impedance Controller...");

  auto& command_interfaces = this->command_interfaces_;
  const auto& state_interfaces = this->state_interfaces_;

  joint_command_handles_.clear();
  joint_position_state_.clear();
  joint_velocity_state_.clear();
  joint_effort_state_.clear();

  joint_command_handles_.resize(dof_, nullptr);
  joint_position_state_.resize(dof_, nullptr);
  joint_velocity_state_.resize(dof_, nullptr);
  joint_effort_state_.resize(dof_, nullptr);

  if (q_.size() != dof_)
    q_.resize(dof_);
  if (dq_.size() != dof_)
    dq_.resize(dof_);
  if (tau_m_.size() != dof_)
    tau_m_.resize(dof_);
  if (tau_c_.size() != dof_)
    tau_c_.resize(dof_);

  for (const auto& interface : state_interfaces)
  {
    const std::string& joint_name = interface.get_prefix_name();
    const std::string& interface_type = interface.get_interface_name();

    auto it = std::find(params_.joints.begin(), params_.joints.end(), joint_name);
    if (it == params_.joints.end())
      continue;
    const size_t index = std::distance(params_.joints.begin(), it);

    if (interface_type == hardware_interface::HW_IF_POSITION)
      joint_position_state_[index] = &interface;
    else if (interface_type == hardware_interface::HW_IF_VELOCITY)
      joint_velocity_state_[index] = &interface;
    else if (interface_type == hardware_interface::HW_IF_EFFORT)
      joint_effort_state_[index] = &interface;
  }

  for (auto& interface : command_interfaces)
  {
    if (interface.get_interface_name() != hardware_interface::HW_IF_EFFORT)
      continue;
    const std::string& joint_name = interface.get_prefix_name();
    auto it = std::find(params_.joints.begin(), params_.joints.end(), joint_name);
    if (it == params_.joints.end())
      continue;
    const size_t index = std::distance(params_.joints.begin(), it);
    joint_command_handles_[index] = &interface;
  }

  update(rclcpp::Time(0), rclcpp::Duration(0, 0));
  initDesiredPose(position_, orientation_);
  initNullspaceConfig(q_d_nullspace_);
  for (size_t i = 0; i < dof_; i++)
  {
    if (!joint_command_handles_[i])
    {
      RCLCPP_ERROR(logger, "Command handle for joint %zu is null!", i);
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(logger, "Controller activated successfully.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceControllerRos::on_deactivate(const rclcpp_lifecycle::State&)
{
  write_zero_commands_to_hardware();
  RCLCPP_INFO(get_node()->get_logger(), "on_deactivate() done.");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceControllerRos::update(const rclcpp::Time&, const rclcpp::Duration&)
{
  if (parameter_handler_->is_old(params_))
  {
    RCLCPP_INFO(get_node()->get_logger(), "Parameters are outdated. Refreshing...");
    parameter_handler_->refresh_dynamic_parameters();
    params_ = parameter_handler_->get_params();
    applyRuntimeParameters();
  }

  if (traj_running_)
    trajUpdate();

  read_state_from_hardware();
  calculateCommandedTorques();  // Populates tau_c_
  write_command_to_hardware();
  publishMsgsAndTf();

  if (auto current_traj = rt_trajectory_->readFromRT())
  {
  }

  return controller_interface::return_type::OK;
}

CallbackReturn CartesianImpedanceControllerRos::on_cleanup(const rclcpp_lifecycle::State&)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceControllerRos::on_error(const rclcpp_lifecycle::State&)
{
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceControllerRos::applyRuntimeParameters()
{
  if (params_.stiffness.update_stiffness)
  {
    double sx = params_.stiffness.translation.x;
    double sy = params_.stiffness.translation.y;
    double sz = params_.stiffness.translation.z;
    double rx = params_.stiffness.rotation.x;
    double ry = params_.stiffness.rotation.y;
    double rz = params_.stiffness.rotation.z;
    double ns = params_.stiffness.nullspace_stiffness;
    RCLCPP_INFO(get_node()->get_logger(),
                "Updating stiffness: trans=(%.2f, %.2f, %.2f), rot=(%.2f, %.2f, %.2f), nullspace=%.2f", sx, sy, sz, rx,
                ry, rz, ns);
    setStiffness(sx, sy, sz, rx, ry, rz, ns, true);
  }

  if (params_.damping.update_damping_factors)
  {
    double tx = params_.damping.translation.x;
    double ty = params_.damping.translation.y;
    double tz = params_.damping.translation.z;
    double rx = params_.damping.rotation.x;
    double ry = params_.damping.rotation.y;
    double rz = params_.damping.rotation.z;
    double ns = params_.damping.nullspace_damping;
    setDampingFactors(tx, ty, tz, rx, ry, rz, ns);
  }

  if (params_.wrench.apply_wrench)
  {
    double fx = params_.wrench.force_x;
    double fy = params_.wrench.force_y;
    double fz = params_.wrench.force_z;
    double tx = params_.wrench.torque_x;
    double ty = params_.wrench.torque_y;
    double tz = params_.wrench.torque_z;

    Eigen::Matrix<double, 6, 1> F;
    F << fx, fy, fz, tx, ty, tz;

    if (!transformWrench(&F, wrench_ee_frame_, root_frame_))
      RCLCPP_WARN(get_node()->get_logger(), "Could not transform param-based wrench. Not applying it.");
    else
      applyWrench(F);
  }
}

bool CartesianImpedanceControllerRos::initRBDyn()
{
  auto node = get_node();
  auto logger = node->get_logger();

  std::string urdf_string;
  if (!node->get_parameter("robot_description", urdf_string))
  {
    RCLCPP_ERROR(logger, "No robot URDF found in 'robot_description' parameter!");
    return false;
  }

  try
  {
    rbdyn_wrapper_.init_rbdyn(urdf_string, end_effector_);
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(logger, "Error initializing RBDyn: %s", e.what());
    return false;
  }

  root_frame_ = rbdyn_wrapper_.root_link();
  RCLCPP_INFO(logger, "Root frame is '%s'.", root_frame_.c_str());

  if (rbdyn_wrapper_.n_joints() < static_cast<int>(dof_))
  {
    RCLCPP_ERROR(logger, "URDF has fewer joints (%zu) than the number to be controlled (%zu)",
                 rbdyn_wrapper_.n_joints(), dof_);
    return false;
  }

  return true;
}

void CartesianImpedanceControllerRos::read_state_from_hardware()
{
  if (q_.size() != dof_ || dq_.size() != dof_ || tau_m_.size() != dof_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Mismatched Eigen vector sizes in read_state_from_hardware.");
    return;
  }

  for (size_t i = 0; i < dof_; i++)
  {
    if (!joint_position_state_[i] || !joint_velocity_state_[i] || !joint_effort_state_[i])
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint state handle at index %zu is nullptr.", i);
      q_(i) = dq_(i) = tau_m_(i) = 0.0;
      continue;
    }
    q_(i) = joint_position_state_[i]->get_value();
    dq_(i) = joint_velocity_state_[i]->get_value();
    tau_m_(i) = joint_effort_state_[i]->get_value();
  }

  getJacobian(q_, dq_, &jacobian_);
  getFk(q_, &position_, &orientation_);
}

void CartesianImpedanceControllerRos::write_command_to_hardware()
{
  for (size_t i = 0; i < dof_; i++)
    joint_command_handles_[i]->set_value(tau_c_(i));
}

void CartesianImpedanceControllerRos::write_zero_commands_to_hardware()
{
  for (size_t i = 0; i < dof_; i++)
    joint_command_handles_[i]->set_value(0.0);
}

bool CartesianImpedanceControllerRos::getFk(const Eigen::VectorXd& q, Eigen::Vector3d* position,
                                            Eigen::Quaterniond* orientation)
{
  auto logger = get_node()->get_logger();

  if (!position || !orientation)
  {
    RCLCPP_ERROR(logger, "getFk: Null pointer provided for position or orientation.");
    return false;
  }

  rbdyn_wrapper::EefState ee_state;

  if (q.size() != this->n_joints_)
  {
    RCLCPP_ERROR(logger, "getFk: Provided joint vector size (%ld) does not match the controller's joint count (%zu).",
                 q.size(), this->n_joints_);
    return false;
  }

  if (this->rbdyn_wrapper_.n_joints() != this->n_joints_)
  {
    Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
    q_rb.head(q.size()) = q;
    try
    {
      ee_state = this->rbdyn_wrapper_.perform_fk(q_rb);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "getFk: Exception during perform_fk with expanded joint vector: %s", e.what());
      return false;
    }
  }
  else
  {
    try
    {
      ee_state = this->rbdyn_wrapper_.perform_fk(q);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger, "getFk: Exception during perform_fk: %s", e.what());
      return false;
    }
  }

  *position = ee_state.translation;
  *orientation = ee_state.orientation;
  RCLCPP_DEBUG(logger, "position: %f, %f, %f", position->x(), position->y(), position->z());
  return true;
}

bool CartesianImpedanceControllerRos::getJacobian(const Eigen::VectorXd& q, const Eigen::VectorXd& dq,
                                                  Eigen::MatrixXd* jacobian)
{
  if (!jacobian)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Jacobian pointer is null.");
    return false;
  }

  if (this->rbdyn_wrapper_.n_joints() != this->n_joints_)
  {
    Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
    q_rb.head(q.size()) = q;
    Eigen::VectorXd dq_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
    dq_rb.head(dq.size()) = dq;
    *jacobian = this->rbdyn_wrapper_.jacobian(q_rb, dq_rb);
  }
  else
  {
    *jacobian = this->rbdyn_wrapper_.jacobian(q, dq);
  }
  *jacobian = jacobian_perm_ * *jacobian;
  return true;
}

void CartesianImpedanceControllerRos::controllerConfigCb(
    const cartesian_impedance_controller::msg::ControllerConfig::SharedPtr msg)
{
  setStiffness(msg->cartesian_stiffness.force.x, msg->cartesian_stiffness.force.y, msg->cartesian_stiffness.force.z,
               msg->cartesian_stiffness.torque.x, msg->cartesian_stiffness.torque.y, msg->cartesian_stiffness.torque.z,
               msg->nullspace_stiffness, false);
  setDampingFactors(msg->cartesian_damping_factors.force.x, msg->cartesian_damping_factors.force.y, msg->cartesian_damping_factors.force.z,
                    msg->cartesian_damping_factors.torque.x, msg->cartesian_damping_factors.torque.y, msg->cartesian_damping_factors.torque.z,
                    msg->nullspace_damping_factor);

  if (msg->q_d_nullspace.size() == dof_)
  {
    Eigen::VectorXd qd(dof_);
    for (size_t i = 0; i < dof_; i++)
    {
      qd(i) = msg->q_d_nullspace[i];
    }
    setNullspaceConfig(qd);
  }
  else
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Nullspace configuration has wrong dimension: got %zu, expected %zu",
                msg->q_d_nullspace.size(), dof_);
  }
}

void CartesianImpedanceControllerRos::publishMsgsAndTf()
{
  if (rt_pub_torques_ && rt_pub_torques_->trylock())
  {
    rt_pub_torques_->msg_.data.resize(tau_c_.size());
    for (size_t i = 0; i < tau_c_.size(); ++i)
    {
      rt_pub_torques_->msg_.data[i] = tau_c_(i);
    }
    rt_pub_torques_->unlockAndPublish();
  }

  Eigen::Matrix<double, 6, 1> error = getPoseError();

  if (params_.verbosity.verbose_print)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Cartesian Position: [%f, %f, %f]", position_(0), position_(1), position_(2));

    RCLCPP_INFO(get_node()->get_logger(), "Pose Error: [%f, %f, %f, %f, %f, %f]", error(0), error(1), error(2),
                error(3), error(4), error(5));

    RCLCPP_INFO(get_node()->get_logger(), "Cartesian Stiffness (diag): [%f, %f, %f, %f, %f, %f]",
                cartesian_stiffness_.diagonal()(0), cartesian_stiffness_.diagonal()(1),
                cartesian_stiffness_.diagonal()(2), cartesian_stiffness_.diagonal()(3),
                cartesian_stiffness_.diagonal()(4), cartesian_stiffness_.diagonal()(5));

    RCLCPP_INFO(get_node()->get_logger(), "Cartesian Damping (diag): [%f, %f, %f, %f, %f, %f]",
                cartesian_damping_.diagonal()(0), cartesian_damping_.diagonal()(1), cartesian_damping_.diagonal()(2),
                cartesian_damping_.diagonal()(3), cartesian_damping_.diagonal()(4), cartesian_damping_.diagonal()(5));

    RCLCPP_INFO(get_node()->get_logger(), "Nullspace Stiffness: %f", nullspace_stiffness_);

    {
      std::stringstream ss;
      ss << "q_d_nullspace: [";
      for (int i = 0; i < q_d_nullspace_.size(); ++i)
      {
        ss << q_d_nullspace_(i);
        if (i != q_d_nullspace_.size() - 1)
        {
          ss << ", ";
        }
      }
      ss << "]";
      RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());
    }
    {
      std::stringstream ss;
      ss << "tau_c: [";
      for (int i = 0; i < tau_c_.size(); ++i)
      {
        ss << tau_c_(i);
        if (i != tau_c_.size() - 1)
        {
          ss << ", ";
        }
      }
      ss << "]";
      RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());
    }
  }

  if (params_.verbosity.tf_frames)
  {
    rclcpp::Time now = get_node()->now();
    if (now > tf_last_time_)
    {
      if (!tf_br_)
      {
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());
      }

      geometry_msgs::msg::TransformStamped fk_tf;
      fk_tf.header.stamp = now;
      fk_tf.header.frame_id = root_frame_;
      fk_tf.child_frame_id = end_effector_ + "_ee_fk";
      fk_tf.transform.translation.x = position_(0);
      fk_tf.transform.translation.y = position_(1);
      fk_tf.transform.translation.z = position_(2);
      fk_tf.transform.rotation =
          tf2::toMsg(tf2::Quaternion(orientation_.x(), orientation_.y(), orientation_.z(), orientation_.w()));
      tf_br_->sendTransform(fk_tf);

      geometry_msgs::msg::TransformStamped ref_tf;
      ref_tf.header.stamp = now;
      ref_tf.header.frame_id = root_frame_;
      ref_tf.child_frame_id = end_effector_ + "_ee_ref_pose";
      ref_tf.transform.translation.x = position_d_(0);
      ref_tf.transform.translation.y = position_d_(1);
      ref_tf.transform.translation.z = position_d_(2);
      ref_tf.transform.rotation =
          tf2::toMsg(tf2::Quaternion(orientation_d_.x(), orientation_d_.y(), orientation_d_.z(), orientation_d_.w()));
      tf_br_->sendTransform(ref_tf);

      tf_last_time_ = now;
    }
  }

  if (params_.verbosity.state_msgs)
  {
    if (rt_pub_state_ && rt_pub_state_->trylock())
    {
      auto& state_msg = rt_pub_state_->msg_;
      state_msg.header.stamp = get_node()->now();

      state_msg.joint_state.name = params_.joints;
      state_msg.joint_state.position.assign(q_.data(), q_.data() + q_.size());
      state_msg.joint_state.velocity.assign(dq_.data(), dq_.data() + dq_.size());
      state_msg.joint_state.effort.assign(tau_m_.data(), tau_m_.data() + tau_m_.size());
      state_msg.commanded_torques.assign(tau_c_.data(), tau_c_.data() + tau_c_.size());
      state_msg.nullspace_config.assign(q_d_nullspace_.data(), q_d_nullspace_.data() + q_d_nullspace_.size());

      state_msg.current_pose.position.x = position_(0);
      state_msg.current_pose.position.y = position_(1);
      state_msg.current_pose.position.z = position_(2);
      state_msg.current_pose.orientation =
          tf2::toMsg(tf2::Quaternion(orientation_.x(), orientation_.y(), orientation_.z(), orientation_.w()));
      state_msg.reference_pose.position.x = position_d_target_(0);
      state_msg.reference_pose.position.y = position_d_target_(1);
      state_msg.reference_pose.position.z = position_d_target_(2);
      state_msg.reference_pose.orientation.w = orientation_d_target_.w();
      state_msg.reference_pose.orientation.x = orientation_d_target_.x();
      state_msg.reference_pose.orientation.y = orientation_d_target_.y();
      state_msg.reference_pose.orientation.z = orientation_d_target_.z();

      state_msg.pose_error.position.x = error(0);
      state_msg.pose_error.position.y = error(1);
      state_msg.pose_error.position.z = error(2);
      double angle = error.tail<3>().norm();
      if (angle > 1e-6)
      {
        Eigen::Vector3d axis = error.tail<3>() / angle;
        Eigen::AngleAxisd aa(angle, axis);
        Eigen::Quaterniond q_err(aa);
        state_msg.reference_pose.orientation =
            tf2::toMsg(tf2::Quaternion(orientation_d_target_.x(), orientation_d_target_.y(), orientation_d_target_.z(),
                                       orientation_d_target_.w()));
      }
      else
      {
        state_msg.pose_error.orientation.w = 1.0;
        state_msg.pose_error.orientation.x = 0.0;
        state_msg.pose_error.orientation.y = 0.0;
        state_msg.pose_error.orientation.z = 0.0;
      }

      state_msg.cartesian_stiffness.force.x = cartesian_stiffness_.diagonal()(0);
      state_msg.cartesian_stiffness.force.y = cartesian_stiffness_.diagonal()(1);
      state_msg.cartesian_stiffness.force.z = cartesian_stiffness_.diagonal()(2);
      state_msg.cartesian_stiffness.torque.x = cartesian_stiffness_.diagonal()(3);
      state_msg.cartesian_stiffness.torque.y = cartesian_stiffness_.diagonal()(4);
      state_msg.cartesian_stiffness.torque.z = cartesian_stiffness_.diagonal()(5);

      state_msg.cartesian_damping.force.x = cartesian_damping_.diagonal()(0);
      state_msg.cartesian_damping.force.y = cartesian_damping_.diagonal()(1);
      state_msg.cartesian_damping.force.z = cartesian_damping_.diagonal()(2);
      state_msg.cartesian_damping.torque.x = cartesian_damping_.diagonal()(3);
      state_msg.cartesian_damping.torque.y = cartesian_damping_.diagonal()(4);
      state_msg.cartesian_damping.torque.z = cartesian_damping_.diagonal()(5);

      Eigen::Matrix<double, 6, 1> applied_wrench = getAppliedWrench();
      state_msg.commanded_wrench.force.x = applied_wrench(0);
      state_msg.commanded_wrench.force.y = applied_wrench(1);
      state_msg.commanded_wrench.force.z = applied_wrench(2);
      state_msg.commanded_wrench.torque.x = applied_wrench(3);
      state_msg.commanded_wrench.torque.y = applied_wrench(4);
      state_msg.commanded_wrench.torque.z = applied_wrench(5);

      state_msg.nullspace_stiffness = nullspace_stiffness_;
      state_msg.nullspace_damping = nullspace_damping_;

      Eigen::Matrix<double, 6, 1> dx = jacobian_ * dq_;
      state_msg.cartesian_velocity = std::sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));

      rt_pub_state_->unlockAndPublish();
    }
  }
}

void CartesianImpedanceControllerRos::cartesianDampingFactorCb(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
  damping_factors_(0) = msg->force.x;
  damping_factors_(1) = msg->force.y;
  damping_factors_(2) = msg->force.z;
  damping_factors_(3) = msg->torque.x;
  damping_factors_(4) = msg->torque.y;
  damping_factors_(5) = msg->torque.z;
  setDampingFactors(damping_factors_(0), damping_factors_(1), damping_factors_(2), damping_factors_(3),
                    damping_factors_(4), damping_factors_(5), damping_factors_(6));
}

void CartesianImpedanceControllerRos::cartesianStiffnessCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  Eigen::Matrix<double, 7, 1> stiffness_vector;
  stiffness_vector << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
      msg->wrench.torque.y, msg->wrench.torque.z, nullspace_stiffness_target_;
  setStiffness(stiffness_vector, true);
}

void CartesianImpedanceControllerRos::referencePoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->header.frame_id == root_frame_)
  {
    Eigen::Vector3d pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond ori(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                           msg->pose.orientation.z);
    setReferencePose(pos, ori);
    return;
  }

  geometry_msgs::msg::PoseStamped transformed_pose;
  try
  {
    transformed_pose = tf_buffer_->transform(*msg, root_frame_, tf2::durationFromSec(0.1));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Transform failed: %s", ex.what());
    return;
  }

  Eigen::Vector3d pos(transformed_pose.pose.position.x, transformed_pose.pose.position.y,
                      transformed_pose.pose.position.z);
  Eigen::Quaterniond ori(transformed_pose.pose.orientation.w, transformed_pose.pose.orientation.x,
                         transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z);
  setReferencePose(pos, ori);
}

void CartesianImpedanceControllerRos::wrenchCommandCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  Eigen::Matrix<double, 6, 1> F;
  F << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
      msg->wrench.torque.z;
  std::string from_frame = msg->header.frame_id.empty() ? wrench_ee_frame_ : msg->header.frame_id;
  if (!transformWrench(&F, from_frame, root_frame_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not transform wrench. Not applying it.");
    return;
  }
  applyWrench(F);
}

bool CartesianImpedanceControllerRos::transformWrench(Eigen::Matrix<double, 6, 1>* wrench,
                                                      const std::string& from_frame, const std::string& to_frame) const
{
  geometry_msgs::msg::TransformStamped transform;
  try
  {
    transform = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "transformWrench() exception: %s", ex.what());
    return false;
  }
  tf2::Vector3 force_in((*wrench)(0), (*wrench)(1), (*wrench)(2));
  tf2::Vector3 torque_in((*wrench)(3), (*wrench)(4), (*wrench)(5));
  tf2::Transform tf;
  tf2::fromMsg(transform.transform, tf);
  tf2::Vector3 force_out = tf * force_in;
  tf2::Vector3 torque_out = tf.getBasis() * torque_in;
  (*wrench)(0) = force_out.x();
  (*wrench)(1) = force_out.y();
  (*wrench)(2) = force_out.z();
  (*wrench)(3) = torque_out.x();
  (*wrench)(4) = torque_out.y();
  (*wrench)(5) = torque_out.z();
  return true;
}

void CartesianImpedanceControllerRos::trajCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory from topic");
  if (traj_as_active_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Preempting running action server goal due to new trajectory");
    traj_running_ = false;
  }
  trajStart(*msg);
  rt_trajectory_->writeFromNonRT(*msg);
}

rclcpp_action::GoalResponse CartesianImpedanceControllerRos::trajGoalCb(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received FollowJointTrajectory action goal request");
  if (goal->trajectory.points.empty() || goal->trajectory.joint_names.size() != dof_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty trajectory OR mismatched joints in action. Rejecting.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CartesianImpedanceControllerRos::trajCancelCb(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>)
{
  RCLCPP_INFO(get_node()->get_logger(), "Canceling trajectory action");
  traj_running_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void CartesianImpedanceControllerRos::trajAcceptCb(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Accepted FollowJointTrajectory action goal");

  traj_as_active_ = true;
  traj_as_goal_ = goal_handle;

  trajStart(goal->trajectory);
  rt_trajectory_->writeFromNonRT(goal->trajectory);
}

void CartesianImpedanceControllerRos::trajStart(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  trajectory_ = trajectory;
  traj_index_ = 0;
  traj_running_ = true;
  traj_start_time_ = get_node()->now();

  if (!trajectory.points.empty())
  {
    traj_duration_ = rclcpp::Duration(trajectory.points.back().time_from_start.sec,
                                      trajectory.points.back().time_from_start.nanosec);
    RCLCPP_INFO(get_node()->get_logger(), "Started a trajectory with %zu points lasting %.2f s.",
                trajectory.points.size(), traj_duration_.seconds());
  }
  else
  {
    RCLCPP_WARN(get_node()->get_logger(), "Empty trajectory. Not running.");
    traj_running_ = false;
  }
}

void CartesianImpedanceControllerRos::trajUpdate()
{
  auto now = get_node()->now();
  double t_since_start = (now - traj_start_time_).seconds();

  if (t_since_start > traj_duration_.seconds())
  {
    RCLCPP_INFO(get_node()->get_logger(), "Finished executing trajectory.");
    traj_running_ = false;
    if (traj_as_active_ && traj_as_goal_)
    {
      auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
      traj_as_goal_->succeed(result);
      traj_as_active_ = false;
    }
    return;
  }

  if (now > (traj_start_time_ + rclcpp::Duration(trajectory_.points.at(traj_index_).time_from_start)))
  {
    Eigen::VectorXd q = Eigen::VectorXd::Map(trajectory_.points.at(traj_index_).positions.data(),
                                             trajectory_.points.at(traj_index_).positions.size());
    getFk(q, &position_d_target_, &orientation_d_target_);
    setReferencePose(position_d_target_, orientation_d_target_);
    setNullspaceConfig(q);
    traj_index_++;
  }
}

}  // end namespace cartesian_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceControllerRos,
                       controller_interface::ControllerInterface)