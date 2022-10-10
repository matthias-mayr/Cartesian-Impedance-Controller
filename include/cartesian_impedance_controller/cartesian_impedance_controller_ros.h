#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <cartesian_impedance_controller/ControllerConfig.h>
#include <cartesian_impedance_controller/ControllerState.h>
#include <cartesian_impedance_controller/dampingConfig.h>
#include <cartesian_impedance_controller/rbdyn_wrapper.h>
#include <cartesian_impedance_controller/stiffnessConfig.h>
#include <cartesian_impedance_controller/wrenchConfig.h>

namespace cartesian_impedance_controller
{
  class CartesianImpedanceControllerRos
      : public controller_interface::Controller<hardware_interface::EffortJointInterface>, public CartesianImpedanceController
  {

  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    bool initDynamicReconfigure(const ros::NodeHandle &nh);
    bool initJointHandles(hardware_interface::EffortJointInterface *hw, const ros::NodeHandle &nh);
    bool initMessaging(ros::NodeHandle *nh);
    bool initRBDyn(const ros::NodeHandle &nh);
    bool initTrajectories(ros::NodeHandle *nh);

    bool getFk(const Eigen::VectorXd &q, Eigen::Vector3d *position, Eigen::Quaterniond *rotation) const;
    bool getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, Eigen::MatrixXd *jacobian);
    void updateState();
    void setDamping(const geometry_msgs::Wrench &cart_stiffness, double nullspace);
    void setStiffness(const geometry_msgs::Wrench &cart_stiffness, double nullspace, bool auto_dammping = true);

    void controllerConfigCb(const cartesian_impedance_controller::ControllerConfigConstPtr &msg);
    void dampingCb(const geometry_msgs::WrenchConstPtr &msg);
    void referencePoseCb(const geometry_msgs::PoseStampedConstPtr &msg);
    void stiffnessCb(const geometry_msgs::WrenchStampedConstPtr &msg);
    void wrenchCommandCb(const geometry_msgs::WrenchStampedConstPtr &msg);

    bool transformWrench(Eigen::Matrix<double, 6, 1> *cartesian_wrench, const std::string &from_frame, const std::string &to_frame) const;
    void publish();

    void dynamicConfigCb(cartesian_impedance_controller::stiffnessConfig &config, uint32_t level);
    void dynamicDampingCb(cartesian_impedance_controller::dampingConfig &config, uint32_t level);
    void dynamicWrenchCb(cartesian_impedance_controller::wrenchConfig &config, uint32_t level);

    void trajGoalCb();
    void trajPreemptCb();
    void trajStart(const trajectory_msgs::JointTrajectory &trajectory);
    void trajUpdate();
    void trajCb(const trajectory_msgs::JointTrajectoryConstPtr &msg);

    std::vector<hardware_interface::JointHandle> joint_handles_;
    rbdyn_wrapper rbdyn_wrapper_;
    std::string end_effector_;
    std::string robot_description_;
    std::string root_frame_;

    Eigen::VectorXd tau_m_;

    ros::Subscriber sub_cart_stiffness_;
    ros::Subscriber sub_cart_wrench_;
    ros::Subscriber sub_damping_;
    ros::Subscriber sub_impedance_config_;
    ros::Subscriber sub_reference_pose_;

    tf::TransformListener tf_listener_;
    std::string wrench_ee_frame_;

    // Limits
    const double trans_stf_min_{0};
    const double trans_stf_max_{2000};
    const double rot_stf_min_{0};
    const double rot_stf_max_{500};
    const double ns_min_{0};
    const double ns_max_{100};
    const double dmp_min_{0.0};
    const double dmp_max_{1.0};

    // The Jacobian of RBDyn comes with orientation in the first three lines. Needs to be interchanged.
    const Eigen::VectorXi perm_indices_ = (Eigen::Matrix<int, 6, 1>() << 3, 4, 5, 0, 1, 2).finished();
    const Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_{Eigen::PermutationMatrix<Eigen::Dynamic, 6>(perm_indices_)};

    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<cartesian_impedance_controller::stiffnessConfig>>
        dynamic_server_compliance_param_;
    std::unique_ptr<dynamic_reconfigure::Server<cartesian_impedance_controller::dampingConfig>>
        dynamic_server_damping_param_;
    std::unique_ptr<dynamic_reconfigure::Server<cartesian_impedance_controller::wrenchConfig>>
        dynamic_server_wrench_param_;

    // Trajectory handling
    ros::Subscriber sub_trajectory_;
    std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> traj_as_;
    boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> traj_goal_;
    trajectory_msgs::JointTrajectory trajectory_;
    ros::Time traj_start_;
    ros::Duration traj_duration_;
    unsigned int traj_index_{0};
    bool traj_running_{false};

    // Extra output
    bool verbose_print_{false};
    bool verbose_state_{false};
    bool verbose_tf_{false};
    tf::TransformBroadcaster tf_br_;
    realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> pub_torques_;
    realtime_tools::RealtimePublisher<cartesian_impedance_controller::ControllerState> pub_state_;
    tf::Transform tf_br_transform_;
    tf::Vector3 tf_pos_;
    tf::Quaternion tf_rot_;
    ros::Time tf_last_time_ = ros::Time::now();
  };
  PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceControllerRos,
                         controller_interface::ControllerBase);

} // namespace cartesian_impedance_controller
