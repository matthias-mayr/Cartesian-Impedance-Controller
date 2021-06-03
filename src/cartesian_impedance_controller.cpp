#include "cartesian_impedance_controller/cartesian_impedance_controller.h"
#include "cartesian_impedance_controller/cartesian_impedance_controller_base.h"
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <iiwa_tools/iiwa_tools.h>
#include <dynamic_reconfigure/server.h>
#include "pseudo_inversion.h"

#include <Eigen/Dense>
#include <cartesian_impedance_controller/impedance_configConfig.h>
#include <dynamic_reconfigure/server.h>

namespace cartesian_impedance_controller
{

  bool CartesianImpedanceController::get_fk(const Eigen::Matrix<double, 7, 1> &q, Eigen::Vector3d &translation, Eigen::Quaterniond &orientation)
  {
    iiwa_tools::RobotState robot_state;
    robot_state.position = q;

    iiwa_tools::EefState ee_state = _tools.perform_fk(robot_state);
    translation = ee_state.translation;
    orientation = ee_state.orientation;
    return true;
  }

  bool CartesianImpedanceController::get_jacobian(const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq, Eigen::Matrix<double, 6, 7> &jacobian)
  {
    iiwa_tools::RobotState robot_state;
    robot_state.position = q;
    robot_state.velocity = dq;

    jacobian = _tools.jacobian(robot_state);
    jacobian = jacobian_perm_ * jacobian;
    return true;
  }

  bool CartesianImpedanceController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle)
  {

    ROS_INFO("CartesianImpedanceController namespace: %s", node_handle.getNamespace().c_str());
    node_handle.param<bool>("verbose", verbose_, false);
    perm_indices_ = Eigen::VectorXi(6);
    perm_indices_ << 3, 4, 5, 0, 1, 2;
    jacobian_perm_ = Eigen::PermutationMatrix<Eigen::Dynamic, 6>(perm_indices_);

    as_ = std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(node_handle, std::string("follow_joint_trajectory"), false));
    as_->registerGoalCallback(boost::bind(&CartesianImpedanceController::goalCallback, this));
    as_->registerPreemptCallback(boost::bind(&CartesianImpedanceController::preemptCallback, this));
    as_->start();

    // Set desired poses through this topic
    sub_desired_pose = node_handle.subscribe("target_pose", 1, &CartesianImpedanceController::ee_pose_Callback, this);

    //set cartesian stiffness values through this topic
    sub_CartesianImpedanceParams = node_handle.subscribe("cartesian_impedance_parameters", 1, &CartesianImpedanceController::cartesian_impedance_Callback, this);

    //set cartesian damping values through this topic
    sub_DampingParams = node_handle.subscribe("damping_parameters", 1, &CartesianImpedanceController::damping_parameters_Callback, this);

    //set nullspace configuration through this topic
    sub_NullspaceConfig = node_handle.subscribe("nullspace_configuration", 1 , &CartesianImpedanceController::nullspace_config_Callback, this);
    
    //set cartesian wrench through this topic
    sub_CartesianWrench = node_handle.subscribe("set_cartesian_wrench", 1, &CartesianImpedanceController::cartesian_wrench_Callback, this);

    //get params for applying Cartesian wrenches
    node_handle.param<std::string>("from_frame_wrench", from_frame_wrench_, "world");
    node_handle.param<std::string>("to_frame_wrench", to_frame_wrench_, "bh_link_ee");

    sub_trajectory_ = node_handle.subscribe("command", 1, &CartesianImpedanceController::trajectoryCallback, this);
    sub_equilibrium_pose_ = node_handle.subscribe(
        "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    // Get JointHandles
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joints", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(hw->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM(
            "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // Setup for iiwa_tools
    node_handle.param<std::string>("end_effector", end_effector_, "iiwa_link_ee");
    ROS_INFO_STREAM("End effektor link is: " << end_effector_);
    // Get the URDF XML from the parameter server
    std::string urdf_string;
    // search and wait for robot_description on param server
    node_handle.param<std::string>("robot_description", robot_description_, "/robot_description");
    while (urdf_string.empty())
    {
      ROS_INFO_ONCE_NAMED("CartesianImpedanceController", "Waiting for robot description in parameter %s on the ROS param server.",
                          robot_description_.c_str());
      node_handle.getParam(robot_description_, urdf_string);
      usleep(100000);
    }

    // Initialize iiwa tools
    _tools.init_rbdyn(urdf_string, end_effector_);
    // Number of joints
    n_joints_ = _tools.get_indices().size();
    ROS_INFO_STREAM_NAMED("CartesianImpedanceController", "Number of joints found in urdf: " << n_joints_);

    pub_torques_.init(node_handle, "commanded_torques", 20);
    pub_torques_.msg_.layout.dim.resize(1);
    pub_torques_.msg_.layout.data_offset = 0;
    pub_torques_.msg_.layout.dim[0].size = n_joints_;
    pub_torques_.msg_.layout.dim[0].stride = 0;
    pub_torques_.msg_.data.resize(n_joints_);

    //DYNAMIC RECONFIGURE
    //-------------------------------------------------------------------------------------------------------------------------------------
    //change stiffness
    dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle("cartesian_impedance_controller_reconfigure");
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<cartesian_impedance_controller::impedance_configConfig>>(dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(
        boost::bind(&CartesianImpedanceController::dynamicConfigCallback, this, _1, _2));

    //-------------------------------------------------------------------------------------------------------------------------------------

    // Initialize variables
    base_tools.initialize();

    //Initialize publisher of useful data
    pub_data_export_ = node_handle.advertise<cartesian_impedance_controller::RobotImpedanceState>("useful_data_to_analyze", 1);

    return true;
  }

  void CartesianImpedanceController::starting(const ros::Time & /*time*/)
  {
    // set x_attractor and q_d_nullspace
    ROS_INFO("Starting Cartesian Impedance Controller");
    Eigen::Matrix<double, 7, 1> q_initial;
    Eigen::Matrix<double, 7, 1> dq_initial;
    for (size_t i = 0; i < 7; ++i)
    {
      q_initial[i] = joint_handles_[i].getPosition();
      dq_initial[i] = joint_handles_[i].getVelocity();
    }

    // get end effector pose
    // set equilibrium point to current state
    Eigen::Vector3d initial_pos;
    Eigen::Quaterniond initial_quat;
    get_fk(q_initial, initial_pos, initial_quat);
    base_tools.set_desired_pose(initial_pos, initial_quat);

    // set nullspace equilibrium configuration to initial q
    set_nullspace_config(q_initial);

    //Save the time at start
    time_at_start_ = ros::Time::now().toSec();
  }

  void CartesianImpedanceController::update(const ros::Time & /*time*/,
                                            const ros::Duration &period /*period*/)

  {

    if (traj_running_)
    {
      trajectoryUpdate();
    }
    Eigen::Matrix<double, 7, 1> q;
    Eigen::Matrix<double, 7, 1> dq;
    Eigen::Matrix<double, 7, 1> q_interface;
    Eigen::Matrix<double, 7, 1> dq_interface;

    for (size_t i = 0; i < 7; ++i)
    {
      q[i] = joint_handles_[i].getPosition();
      dq[i] = joint_handles_[i].getVelocity();
    }
    // get jacobian
    Eigen::Matrix<double, 6, 7> jacobian;
    get_jacobian(q, dq, jacobian);
    Eigen::Matrix<double, 6, 1> dx;
    dx << jacobian * dq;
    double cartesian_velocity = sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));

    // get forward kinematics
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    get_fk(q, position, orientation);

    if (verbose_)
    {
      tf::vectorEigenToTF(position, tf_pos_);
      ROS_INFO_STREAM_THROTTLE(0.1, "\nCARTESIAN POSITION:\n"
                                        << position);
      tf_br_transform_.setOrigin(tf_pos_);
      tf::quaternionEigenToTF(orientation, tf_rot_);
      tf_br_transform_.setRotation(tf_rot_);
      tf_br_.sendTransform(tf::StampedTransform(tf_br_transform_, ros::Time::now(), "world", "fk_ee"));
    }

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
    // convert to axis angle
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    // compute "orientation error"
    error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
    // compute the control law
    Eigen::VectorXd tau_d = base_tools.get_commanded_torques(q, dq, position, orientation, jacobian);
    // saturate to torque rate
    base_tools.saturateTorqueRate(tau_d, tau_J_d_, delta_tau_max_);
    // get the robot state
    base_tools.get_robot_state(position_d_, orientation_d_, cartesian_stiffness_, nullspace_stiffness_, q_d_nullspace_, cartesian_damping_);

    // compute error to desired pose
    // position error
    tf::vectorEigenToTF(Eigen::Vector3d(error.head(3)), tf_pos_);
    tf_br_transform_.setOrigin(tf_pos_);

    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i));
    }

    if (verbose_)
    {
      ROS_INFO_STREAM_THROTTLE(0.1, "\nERROR:\n"
                                        << error);
      ROS_INFO_STREAM_THROTTLE(0.1, "\nParameters:\nCartesian Stiffness:\n"
                                        << cartesian_stiffness_ << "\nCartesian damping:\n"
                                        << cartesian_damping_ << "\nNullspace stiffness:\n"
                                        << nullspace_stiffness_ << "\nq_d_nullspace:\n"
                                        << q_d_nullspace_);
      ROS_INFO_STREAM_THROTTLE(0.1, "\ntau_d:\n"
                                        << tau_d);
    }
    
    try
    {
      // Update transformation of Cartesian Wrench
      tf_listener_.lookupTransform(from_frame_wrench_, to_frame_wrench_, ros::Time(0), transform_);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR_THROTTLE(1, "%s", ex.what());
    }
    
    //filtering
    double update_frequency = 1 / period.toSec();
    base_tools.set_filtering(update_frequency, 0.005, 1., 0.01);

    publish();
    //publish useful data to a topic
    publish_data(q, dq, position, orientation, position_d_, orientation_d_, tau_d, cartesian_stiffness_, nullspace_stiffness_, error, base_tools.get_applied_wrench(),cartesian_velocity);
  }
  //Publish data to export and analyze
  void CartesianImpedanceController::publish_data(Eigen::Matrix<double, 7, 1> q, Eigen::Matrix<double, 7, 1> dq, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Vector3d position_d_, Eigen::Quaterniond orientation_d_, Eigen::VectorXd tau_d, Eigen::Matrix<double, 6, 6> cartesian_stiffness_, double nullspace_stiffness_, Eigen::Matrix<double, 6, 1> error, Eigen::Matrix<double, 6, 1> F, double cartesian_velocity)
  {

    cartesian_impedance_controller::RobotImpedanceState data_to_analyze;
    data_to_analyze.time = ros::Time::now().toSec() - time_at_start_;
    data_to_analyze.position.x = position[0];
    data_to_analyze.position.y = position[1];
    data_to_analyze.position.z = position[2];

    data_to_analyze.position_d_.x = position_d_[0];
    data_to_analyze.position_d_.y = position_d_[1];
    data_to_analyze.position_d_.z = position_d_[2];

    data_to_analyze.orientation.x = orientation.coeffs()[0];
    data_to_analyze.orientation.y = orientation.coeffs()[1];
    data_to_analyze.orientation.z = orientation.coeffs()[2];
    data_to_analyze.orientation.w = orientation.coeffs()[3];

    data_to_analyze.orientation_d_.x = orientation_d_.coeffs()[0];
    data_to_analyze.orientation_d_.y = orientation_d_.coeffs()[1];
    data_to_analyze.orientation_d_.z = orientation_d_.coeffs()[2];
    data_to_analyze.orientation_d_.w = orientation_d_.coeffs()[3];

    data_to_analyze.tau_d.q1 = tau_d(0);
    data_to_analyze.tau_d.q2 = tau_d(1);
    data_to_analyze.tau_d.q3 = tau_d(2);
    data_to_analyze.tau_d.q4 = tau_d(3);
    data_to_analyze.tau_d.q5 = tau_d(4);
    data_to_analyze.tau_d.q6 = tau_d(5);
    data_to_analyze.tau_d.q7 = tau_d(6);

    data_to_analyze.q.q1 = q(0);
    data_to_analyze.q.q2 = q(1);
    data_to_analyze.q.q3 = q(2);
    data_to_analyze.q.q4 = q(3);
    data_to_analyze.q.q5 = q(4);
    data_to_analyze.q.q6 = q(5);
    data_to_analyze.q.q7 = q(6);

    data_to_analyze.dq.q1 = dq(0);
    data_to_analyze.dq.q2 = dq(1);
    data_to_analyze.dq.q3 = dq(2);
    data_to_analyze.dq.q4 = dq(3);
    data_to_analyze.dq.q5 = dq(4);
    data_to_analyze.dq.q6 = dq(5);
    data_to_analyze.dq.q7 = dq(6);

    data_to_analyze.cartesian_stiffness.x = cartesian_stiffness_(0, 0);
    data_to_analyze.cartesian_stiffness.y = cartesian_stiffness_(1, 1);
    data_to_analyze.cartesian_stiffness.z = cartesian_stiffness_(2, 2);
    data_to_analyze.cartesian_stiffness.a = cartesian_stiffness_(3, 3);
    data_to_analyze.cartesian_stiffness.b = cartesian_stiffness_(4, 4);
    data_to_analyze.cartesian_stiffness.c = cartesian_stiffness_(5, 5);

    data_to_analyze.nullspace_stiffness = nullspace_stiffness_;

    data_to_analyze.cartesian_wrench.f_x = F(0);
    data_to_analyze.cartesian_wrench.f_y = F(1);
    data_to_analyze.cartesian_wrench.f_z = F(2);
    data_to_analyze.cartesian_wrench.tau_x = F(3);
    data_to_analyze.cartesian_wrench.tau_y = F(4);
    data_to_analyze.cartesian_wrench.tau_z = F(5);

    data_to_analyze.error_position.x = error(0);
    data_to_analyze.error_position.y = error(1);
    data_to_analyze.error_position.z = error(2);
    data_to_analyze.error_rotation.x = error(3);
    data_to_analyze.error_rotation.y = error(4);
    data_to_analyze.error_rotation.z = error(5);

    data_to_analyze.cartesian_velocity = cartesian_velocity;

    pub_data_export_.publish(data_to_analyze);
  }

  void CartesianImpedanceController::ee_pose_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_);
    orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_.coeffs()) < 0.0)
    {
      orientation_d_.coeffs() << -orientation_d_.coeffs();
    }
    base_tools.set_desired_pose(position_d_, orientation_d_);
  }


  //------------------------------------------------------------------------------------------------------
  void CartesianImpedanceController::trajectoryStart(const trajectory_msgs::JointTrajectory &trajectory)
  {
    traj_duration_ = trajectory.points[trajectory.points.size() - 1].time_from_start;
    ROS_INFO_STREAM("Got a new trajectory with " << trajectory.points.size() << " points that takes " << traj_duration_ << "s.");
    trajectory_ = trajectory;
    traj_running_ = true;
    traj_start_ = ros::Time::now();
    traj_index_ = 0;
    trajectoryUpdate();
  }

  void CartesianImpedanceController::trajectoryUpdate()
  {
    if (ros::Time::now() > (traj_start_ + trajectory_.points[traj_index_].time_from_start))
    {
      // Get end effector pose
      Eigen::Matrix<double, 7, 1> q(trajectory_.points[traj_index_].positions.data());
      ROS_INFO_STREAM("Index " << traj_index_ << " q_nullspace: " << q.transpose());
      Eigen::Vector3d translation;
      Eigen::Quaterniond orientation;
      get_fk(q, translation, orientation);
      // Update end effector pose
      position_d_ = translation;
      orientation_d_ = orientation;
      // Update nullspace
      base_tools.set_nullspace_config(q);
      traj_index_++;
    }

    if (ros::Time::now() > (traj_start_ + traj_duration_))
    {
      ROS_INFO_STREAM("Finished executing trajectory.");
      if (as_->isActive())
      {
        as_->setSucceeded();
      }
      traj_running_ = false;
      return;
    }
  }
  void CartesianImpedanceController::cartesian_impedance_Callback(const cartesian_impedance_controller::CartesianImpedanceControlMode &msg)
  {
    double trans_stf_max = 1200;
    double trans_stf_min = 0;
    double rot_stf_max = 500;
    double rot_stf_min = 0;
    double null_max=50;
    double null_min=0;
    base_tools.set_stiffness(saturate(msg.cartesian_stiffness.x, trans_stf_min, trans_stf_max), saturate(msg.cartesian_stiffness.y, trans_stf_min, trans_stf_max), saturate(msg.cartesian_stiffness.z, trans_stf_min, trans_stf_max),
                             saturate(msg.cartesian_stiffness.a, trans_stf_min, trans_stf_max), saturate(msg.cartesian_stiffness.b, trans_stf_min, trans_stf_max), saturate(msg.cartesian_stiffness.c, trans_stf_min, trans_stf_max), saturate(msg.nullspace_stiffness,null_min,null_max));
  }

  void CartesianImpedanceController::damping_parameters_Callback(const cartesian_impedance_controller::CartesianImpedanceControlMode &msg)
  {
    double dmp_max = 1;
    double dmp_min = 0.1;
    base_tools.set_damping(saturate(msg.cartesian_damping.x, dmp_min, dmp_max), saturate(msg.cartesian_damping.y, dmp_min, dmp_max),
                           saturate(msg.cartesian_damping.z, dmp_min, dmp_max), saturate(msg.cartesian_damping.a, dmp_min, dmp_max),
                           saturate(msg.cartesian_damping.b, dmp_min, dmp_max), saturate(msg.cartesian_damping.c, dmp_min, dmp_max), 1);
  }

  void CartesianImpedanceController::nullspace_config_Callback(const cartesian_impedance_controller::CartesianImpedanceControlMode &msg)
  {
    Eigen::Matrix<double, 7, 1> q_d_nullspace_target_;
    q_d_nullspace_target_ << msg.q_d_nullspace.q1, msg.q_d_nullspace.q2,msg.q_d_nullspace.q3,
    msg.q_d_nullspace.q4,msg.q_d_nullspace.q5,msg.q_d_nullspace.q6,msg.q_d_nullspace.q7;
    base_tools.set_nullspace_config(q_d_nullspace_target_);
  }

  //Adds a wrench at the end-effector, using the world frame
  void CartesianImpedanceController::cartesian_wrench_Callback(const geometry_msgs::WrenchStampedConstPtr &msg)
  {
    Eigen::Matrix<double, 6, 1> F;
    F << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
        msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
    transform_wrench(F, from_frame_wrench_, to_frame_wrench_);
    base_tools.apply_wrench(F);
  }

  // Transform a Cartesian wrench from "from_frame" to "to_frame". E.g. from_frame= "world" , to_frame = "bh_link_ee"
  void CartesianImpedanceController::transform_wrench(Eigen::Matrix<double, 6, 1> &cartesian_wrench, std::string from_frame, std::string to_frame)
  {

    try
    {
      //tf_listener_.lookupTransform(from_frame_wrench_, to_frame_wrench_, ros::Time(0), transform_);
      tf::Vector3 v_f(cartesian_wrench(0), cartesian_wrench(1), cartesian_wrench(2));
      tf::Vector3 v_t(cartesian_wrench(3), cartesian_wrench(4), cartesian_wrench(5));
      tf::Vector3 v_f_rot = tf::quatRotate(transform_.getRotation(), v_f);
      tf::Vector3 v_t_rot = tf::quatRotate(transform_.getRotation(), v_t);
      cartesian_wrench << v_f_rot[0], v_f_rot[1], v_f_rot[2], v_t_rot[0], v_t_rot[1], v_t_rot[2];
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR_THROTTLE(1, "%s", ex.what());
    }
  }

  void CartesianImpedanceController::goalCallback()
  {
    goal_ = as_->acceptNewGoal();
    ROS_INFO("Accepted new goal");
    trajectoryStart(goal_->trajectory);
  }

  void CartesianImpedanceController::preemptCallback()
  {
    ROS_INFO("Actionserver got preempted.");
  }

  void CartesianImpedanceController::trajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr &msg)
  {
    ROS_INFO("Got trajectory msg");
    trajectoryStart(*msg);
  }
  //Dynamic reconfigure
  //--------------------------------------------------------------------------------------------------------------------------------------
  void CartesianImpedanceController::dynamicConfigCallback(cartesian_impedance_controller::impedance_configConfig &config, uint32_t level)
  {
    if (config.apply_stiffness)
    {
      double trans_stf_max = 1200;
      double trans_stf_min = 0;
      double rot_stf_max = 300;
      double rot_stf_min = 0;
      base_tools.set_stiffness(saturate(config.translation_x, trans_stf_min, trans_stf_max), saturate(config.translation_y, trans_stf_min, trans_stf_max), saturate(config.translation_z, trans_stf_min, trans_stf_max),
                               saturate(config.rotation_x, trans_stf_min, trans_stf_max), saturate(config.rotation_y, trans_stf_min, trans_stf_max), saturate(config.rotation_z, trans_stf_min, trans_stf_max), config.nullspace_stiffness);
    }
  }

  //--------------------------------------------------------------------------------------------------------------------------------------
  void CartesianImpedanceController::equilibriumPoseCallback(
      const geometry_msgs::PoseStampedConstPtr &msg)
  {
    position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_);
    orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_.coeffs()) < 0.0)
    {
      orientation_d_.coeffs() << -orientation_d_.coeffs();
    }
    base_tools.set_desired_pose(position_d_, orientation_d_);
  }

  void CartesianImpedanceController::publish()
  {
    // publish commanded torques
    if (pub_torques_.trylock())
    {
      for (unsigned i = 0; i < n_joints_; i++)
      {
        pub_torques_.msg_.data[i] = tau_J_d_[i];
      }
      pub_torques_.unlockAndPublish();
    }
    // Publish tf to the equilibrium pose
    tf::vectorEigenToTF(position_d_, tf_pos_);
    tf_br_transform_.setOrigin(tf_pos_);
    tf::quaternionEigenToTF(orientation_d_, tf_rot_);
    tf_br_transform_.setRotation(tf_rot_);
    tf_br_.sendTransform(tf::StampedTransform(tf_br_transform_, ros::Time::now(), "world", "eq_pose"));
  }

}