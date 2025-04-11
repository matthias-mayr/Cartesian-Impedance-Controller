#ifndef CARTESIAN_IMPEDANCE_CONTROLLER__CARTESIAN_IMPEDANCE_CONTROLLER_ROS_HPP_
#define CARTESIAN_IMPEDANCE_CONTROLLER__CARTESIAN_IMPEDANCE_CONTROLLER_ROS_HPP_

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <urdf/model.h>

#include <cartesian_impedance_controller/cartesian_impedance_controller.hpp>
#include <cartesian_impedance_controller/rbdyn_wrapper.h>
#include <cartesian_impedance_controller/cartesian_impedance_controller_parameters.hpp>
#include <cartesian_impedance_controller/msg/controller_config.hpp>
#include <cartesian_impedance_controller/msg/controller_state.hpp>

#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace cartesian_impedance_controller
{

class ParamListener;

class CartesianImpedanceControllerRos : public controller_interface::ControllerInterface,
                                        public CartesianImpedanceController
{
public:
  CartesianImpedanceControllerRos();
  ~CartesianImpedanceControllerRos() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  const Eigen::VectorXi perm_indices_ = (Eigen::Matrix<int, 6, 1>() << 3, 4, 5, 0, 1, 2).finished();
  const Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_ =
      Eigen::PermutationMatrix<Eigen::Dynamic, 6>(perm_indices_);

  std::vector<std::string> command_joint_names_;
  size_t dof_{ 0 };
  std::shared_ptr<realtime_tools::RealtimeBuffer<trajectory_msgs::msg::JointTrajectory>> rt_trajectory_;

  std::vector<hardware_interface::LoanedCommandInterface*> joint_command_handles_;
  std::vector<const hardware_interface::LoanedStateInterface*> joint_position_state_;
  std::vector<const hardware_interface::LoanedStateInterface*> joint_velocity_state_;
  std::vector<const hardware_interface::LoanedStateInterface*> joint_effort_state_;

  std::shared_ptr<ParamListener> parameter_handler_;
  Params params_;

  std::string end_effector_;
  std::string wrench_ee_frame_;
  std::string root_frame_;
  std::string robot_description_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Time tf_last_time_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  rbdyn_wrapper rbdyn_wrapper_;

  std::shared_ptr<realtime_tools::RealtimePublisher<cartesian_impedance_controller::msg::ControllerState>>
      rt_pub_state_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_pub_torques_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_cart_stiffness_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_cart_wrench_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_damping_factors_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_reference_pose_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<cartesian_impedance_controller::msg::ControllerConfig>::SharedPtr sub_controller_config_;

  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_as_;
  bool traj_running_ = false;
  bool traj_as_active_ = false;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> traj_as_goal_;

  trajectory_msgs::msg::JointTrajectory trajectory_;
  size_t traj_index_ = 0;
  rclcpp::Time traj_start_time_;
  rclcpp::Duration traj_duration_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  Eigen::VectorXd tau_m_;
  Eigen::VectorXd damping_factors_;
  double nullspace_stiffness_target_;

  void read_state_from_hardware();
  void write_command_to_hardware();
  void write_zero_commands_to_hardware();
  bool getFk(const Eigen::VectorXd& q, Eigen::Vector3d* position, Eigen::Quaterniond* orientation);
  bool getJacobian(const Eigen::VectorXd& q, const Eigen::VectorXd& dq, Eigen::MatrixXd* jacobian);

  void controllerConfigCb(const cartesian_impedance_controller::msg::ControllerConfig::SharedPtr msg);
  void cartesianDampingFactorCb(const geometry_msgs::msg::Wrench::SharedPtr msg);
  void cartesianStiffnessCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void referencePoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void wrenchCommandCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void trajCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  rclcpp_action::GoalResponse trajGoalCb(const rclcpp_action::GoalUUID& uuid,
                                         std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse trajCancelCb(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
  void trajAcceptCb(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
  void trajStart(const trajectory_msgs::msg::JointTrajectory& trajectory);
  void trajUpdate();

  bool transformWrench(Eigen::Matrix<double, 6, 1>* wrench, const std::string& from_frame,
                       const std::string& to_frame) const;
  void publishMsgsAndTf();
  void applyRuntimeParameters();
  bool initRBDyn();
};

}  // namespace cartesian_impedance_controller

#endif  // CARTESIAN_IMPEDANCE_CONTROLLER__CARTESIAN_IMPEDANCE_CONTROLLER_ROS_HPP_