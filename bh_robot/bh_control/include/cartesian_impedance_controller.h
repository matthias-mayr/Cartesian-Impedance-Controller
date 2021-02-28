// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once
#include "cartesian_impedance_controller_base.h"
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <iiwa_tools/iiwa_tools.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <realtime_tools/realtime_publisher.h>

#include <Eigen/Dense>

namespace bh_control
{

  class CartesianImpedanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>, public CartesianImpedanceController_base
  {


  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;


  private:
    
     //cartesian_impedance_controller base tools
    CartesianImpedanceController_base base_tools;
    
    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    // void update_parameters();
    bool get_fk(const Eigen::Matrix<double, 7, 1> &q, Eigen::Vector3d &translation, Eigen::Quaterniond &rotation);
    bool get_jacobian(const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq, Eigen::Matrix<double, 6, 7> &jacobian);

    std::vector<hardware_interface::JointHandle> joint_handles_;

    double filter_params_{0.005};
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{5.0};
    const double delta_tau_max_{1.0};
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_target_;
    Eigen::Matrix<double, 7, 1> tau_J_d_;

    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;

    // IIWA Tools - this is GPLv3
    iiwa_tools::IiwaTools _tools;
    std::string end_effector_;
    std::string robot_description_;
    unsigned int n_joints_;
    // The Jacobian of RBDyn comes with orientation in the first three lines. Needs to be interchanged.
    Eigen::VectorXi perm_indices_;
    Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_;

    // Dynamic reconfigure
    //   std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
    //       dynamic_server_compliance_param_;
    //   ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback();

    // Trajectory handling
    std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> as_;
    void goalCallback();
    void preemptCallback();
    boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> goal_;
    trajectory_msgs::JointTrajectory trajectory_;
    ros::Time traj_start_;
    ros::Duration traj_duration_;
    unsigned int traj_index_{0};
    bool traj_running_{false};
    void trajectoryStart(const trajectory_msgs::JointTrajectory &trajectory);
    void trajectoryUpdate();

    ros::Subscriber sub_trajectory_;
    void trajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg);

    // Equilibrium pose subscriber
    ros::Subscriber sub_equilibrium_pose_;
    void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

    void publish();

    // for debugging
    bool verbose_{false};
    tf::TransformBroadcaster tf_br_;
    realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> pub_torques_;
    tf::Transform tf_br_transform_;
    tf::Vector3 tf_pos_;
    tf::Quaternion tf_rot_;
  };
  PLUGINLIB_EXPORT_CLASS(bh_control::CartesianImpedanceController, controller_interface::ControllerBase);

} // namespace franka_example_controllers
