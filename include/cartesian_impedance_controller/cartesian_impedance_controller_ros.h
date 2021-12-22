#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <actionlib/server/simple_action_server.h>
#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <cartesian_impedance_controller/CartesianImpedanceControlMode.h>
#include <cartesian_impedance_controller/RobotImpedanceState.h>
#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <cartesian_impedance_controller/damping_configConfig.h>
#include <cartesian_impedance_controller/impedance_configConfig.h>
#include <cartesian_impedance_controller/rbdyn_wrapper.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <realtime_tools/realtime_publisher.h>
#include <Eigen/Dense>
#include <cartesian_impedance_controller/impedance_configConfig.h>
#include <cartesian_impedance_controller/damping_configConfig.h>
#include <cartesian_impedance_controller/wrench_configConfig.h>

namespace cartesian_impedance_controller
{
  class CartesianImpedanceControllerRos
      : public controller_interface::Controller<hardware_interface::EffortJointInterface>
  {

  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle) override;
    void starting(const ros::Time &) override;
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    bool initDynamicReconfigure(const ros::NodeHandle &nh);
    bool initJointHandles(hardware_interface::EffortJointInterface *hw, const ros::NodeHandle &nh);
    bool initMessaging(ros::NodeHandle &nh);
    bool initRBDyn(const ros::NodeHandle &nh);
    bool initTrajectories(ros::NodeHandle &nh);

    bool getFk(const Eigen::VectorXd &q, Eigen::Vector3d &position, Eigen::Quaterniond &rotation);
    bool getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, Eigen::MatrixXd &jacobian);
    void updateState();

    void dampingCb(const cartesian_impedance_controller::CartesianImpedanceControlMode &msg);
    void impedanceControlCb(const cartesian_impedance_controller::CartesianImpedanceControlMode &msg);
    void referencePoseCb(const geometry_msgs::PoseStampedConstPtr &msg);
    void stiffnessCb(const geometry_msgs::WrenchStampedConstPtr &msg);
    void wrenchCommandCb(const geometry_msgs::WrenchStampedConstPtr &msg);

    void transformWrench(Eigen::Matrix<double, 6, 1> &cartesian_wrench, std::string from_frame, std::string to_frame);
    void publishData(Eigen::VectorXd q, Eigen::VectorXd dq, Eigen::Vector3d position, Eigen::Quaterniond orientation,
                     Eigen::Vector3d position_d_, Eigen::Quaterniond orientation_d_, Eigen::VectorXd tau_d,
                     Eigen::Matrix<double, 6, 6> cartesian_stiffness_, double nullspace_stiffness_,
                     Eigen::Matrix<double, 6, 1> error, Eigen::Matrix<double, 6, 1> F, double cartesian_velocity);
    void publish();

    void dynamicConfigCb(cartesian_impedance_controller::impedance_configConfig &config, uint32_t level);
    void dynamicDampingCb(cartesian_impedance_controller::damping_configConfig &config, uint32_t level);
    void dynamicWrenchCb(cartesian_impedance_controller::wrench_configConfig &config, uint32_t level);

    void trajGoalCb();
    void trajPreemptCb();
    void trajStart(const trajectory_msgs::JointTrajectory &trajectory);
    void trajUpdate();
    void trajCb(const trajectory_msgs::JointTrajectoryConstPtr &msg);

    std::unique_ptr<CartesianImpedanceController> base_tools_;

    std::vector<hardware_interface::JointHandle> joint_handles_;
    rbdyn_wrapper rbdyn_wrapper_;
    unsigned int n_joints_;
    std::string end_effector_;
    std::string robot_description_;

    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    double nullspace_stiffness_;
    Eigen::VectorXd q_d_nullspace_;
    Eigen::VectorXd tau_J_d_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    Eigen::MatrixXd jacobian_;

    ros::Subscriber sub_cart_stiffness_;
    ros::Subscriber sub_cart_wrench_;
    ros::Subscriber sub_damping_;
    ros::Subscriber sub_impedance_config_;
    ros::Subscriber sub_reference_pose_;
    ros::Publisher pub_data_export_;

    tf::TransformListener tf_listener_;
    tf::StampedTransform transform_;
    std::string from_frame_wrench_;
    std::string to_frame_wrench_;

    double update_frequency_{500};
    double filtering_stiffness_{0.1};
    double filtering_pose_{0.1};
    double filtering_wrench_{0.1};

    // The Jacobian of RBDyn comes with orientation in the first three lines. Needs to be interchanged.
    const Eigen::VectorXi perm_indices_ = (Eigen::Matrix<int, 6, 1>() << 3, 4, 5, 0, 1, 2).finished();
    const Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_{Eigen::PermutationMatrix<Eigen::Dynamic, 6>(perm_indices_)};

    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<cartesian_impedance_controller::impedance_configConfig>>
        dynamic_server_compliance_param_;
    std::unique_ptr<dynamic_reconfigure::Server<cartesian_impedance_controller::damping_configConfig>>
        dynamic_server_damping_param_;
    std::unique_ptr<dynamic_reconfigure::Server<cartesian_impedance_controller::wrench_configConfig>>
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

    // for debugging
    bool verbose_{false};
    tf::TransformBroadcaster tf_br_;
    realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> pub_torques_;
    tf::Transform tf_br_transform_;
    tf::Vector3 tf_pos_;
    tf::Quaternion tf_rot_;
  };
  PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceControllerRos,
                         controller_interface::ControllerBase);

} // namespace cartesian_impedance_controller
