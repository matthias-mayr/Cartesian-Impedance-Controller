#include "cartesian_impedance_controller.h"
#include "cartesian_impedance_controller_base.h"
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
#include <cartesian_impedance_controller/wrench_configConfig.h>
#include "cartesian_impedance_controller/log_configConfig.h"
#include <dynamic_reconfigure/server.h>

#include "ros_logger/ros_logger.h"

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

    //the other traj generator
    sub_pose = node_handle.subscribe("target_pose", 1, &CartesianImpedanceController::ee_poseCallback, this);

    //for logging data
    //-----------------------------------------------------------------------------------------------------------------------------
    latest_request_subscriber = node_handle.subscribe("latest_request", 1, &CartesianImpedanceController::latest_requestCallback, this);
    //-----------------------------------------------------------------------------------------------------------------------------

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
    //apply a wrench at TCP
    dynamic_reconfigure_wrench_param_node_ = ros::NodeHandle("cartesian_wrench_reconfigure");
    dynamic_server_wrench_param_ = std::make_unique<dynamic_reconfigure::Server<cartesian_impedance_controller::wrench_configConfig>>(dynamic_reconfigure_wrench_param_node_);
    dynamic_server_wrench_param_->setCallback(
        boost::bind(&CartesianImpedanceController::dynamicWrenchCallback, this, _1, _2));
    //log data
    dynamic_log_node_ = ros::NodeHandle("log_reconfigure");
    dynamic_server_log_ = std::make_unique<dynamic_reconfigure::Server<cartesian_impedance_controller::log_configConfig>>(dynamic_log_node_);
    dynamic_server_log_->setCallback(
        boost::bind(&CartesianImpedanceController::logCallback, this, _1, _2));
    //-------------------------------------------------------------------------------------------------------------------------------------

    // Initialize variables
    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();

    complianceParamCallback();
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
    Eigen::Vector3d tmp_pos;
    Eigen::Quaterniond tmp_quat;
    get_fk(q_initial, tmp_pos, tmp_quat);

    position_d_ = tmp_pos;
    orientation_d_ = tmp_quat;

    position_d_target_ = position_d_;
    orientation_d_target_ = orientation_d_;

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;
    q_d_nullspace_target_ = q_d_nullspace_;
  }

  void CartesianImpedanceController::update(const ros::Time & /*time*/,
                                            const ros::Duration & /*period*/)
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
      q_interface[i] = joint_handles_[i].getPosition();
      dq_interface[i] = joint_handles_[i].getVelocity();
    }
    //put into q and dq
    base_tools.get_states(q, dq, q_interface, dq_interface);
  
    Eigen::Matrix<double, 6, 7> jacobian;
    get_jacobian(q, dq, jacobian);
    Eigen::Matrix<double,6,1> dx;
    dx << jacobian * dq;
    double v=sqrt(dx(0)*dx(0)+dx(1)*dx(1)+dx(2)*dx(2));
    // get forward kinematics
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    get_fk(q, position, orientation);

    //Log data
    //--------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------

    //TRAJECTORY
    //start exporting data
    if (begin_log)
    {
      //precision 0.1mm, keep push vals until pose close enough or time out
      if (distance_to_goal > 0.001 && v<0.001 || time_now - time_start < time_out) //Time out currently set to 10 seconds for easier debugging
      {
        time_now = (ros::Time::now()).toSec();
        distance_to_goal = (position_new_request - position).norm();
        geometry_msgs::PoseStamped current_pose;
        current_pose.header.stamp = ros::Time::now();
        current_pose.pose.position.x = position[0];
        current_pose.pose.position.y = position[1];
        current_pose.pose.position.z = position[2];
        current_pose.pose.orientation.x = orientation.coeffs()[0];
        current_pose.pose.orientation.y = orientation.coeffs()[1];
        current_pose.pose.orientation.z = orientation.coeffs()[2];
        current_pose.pose.orientation.w = orientation.coeffs()[3];
        pose_trajectory.push_back(current_pose);
      }
      else
      {
        logger.set_preferences(",", print_title_trajectory, over_write_trajectory); //separator, print first line, overwrite
        logger.log_to(path, file_name_trajectory);
        if (logger.log_push_all(pose_trajectory))
        {
          if (time_now - time_start < time_out)
          {
            ROS_INFO("LOG: Trajectory successfully executed.");
          }
          else
          {
            ROS_ERROR("LOG: Time-out. requested pose was not reached");
          }
        }
        else
        {
          ROS_ERROR("LOG: Failed to save trajectory");
        }
        //trajectory done. reset parameters to make it ready for a new recording.
        begin_log = false;
        pose_trajectory.clear();
        time_start = 0;
        time_now = 0;
      }
    }

    if (is_new_request && !begin_log)
    {
      ROS_INFO("LOG: Recieved a new request from node \"cartesian_trajectory_generator_ros\"");
      begin_log = true;
      is_new_request = false;
      time_start = (ros::Time::now()).toSec();
    }

    //SIMULATION
    if (begin_log_simulation)
    {
      if (time_now_simulation - time_start_simulation < simulation_time_total)
      {
        //push data
        time_now_simulation = (ros::Time::now()).toSec();
        time_VECTOR.push_back(time_now_simulation);
        position_VECTOR.push_back(position);
        orientation_VECTOR.push_back(orientation.coeffs());
        position_d_VECTOR.push_back(position_d_);
        orientation_d_VECTOR.push_back(orientation_d_.coeffs());
        translational_stiffness_VECTOR.push_back(translational_stiffness);
        rotational_stiffness_VECTOR.push_back(rotational_stiffness);
        nullspace_stiffness_VECTOR.push_back(nullspace_stiffness);
        v_VECTOR.push_back(v);
      }
      else
      {
        //log data
       
        logger.set_preferences(",", print_title_simulation, over_write_simulation); //separator, print first line, overwrite
        logger.log_to(path, file_name_simulation);
        logger.log_push_all(time_VECTOR, position_VECTOR,
                            orientation_VECTOR, position_d_VECTOR,
                            orientation_d_VECTOR, translational_stiffness_VECTOR,
                            rotational_stiffness_VECTOR, nullspace_stiffness_VECTOR,v_VECTOR);
        ROS_INFO("LOG: Simulation saved.");
      time_VECTOR.clear();
      position_VECTOR.clear();
      orientation_VECTOR.clear();
      position_d_VECTOR.clear();
      orientation_d_VECTOR.clear();
      translational_stiffness_VECTOR.clear();
      rotational_stiffness_VECTOR.clear();
      nullspace_stiffness_VECTOR.clear();
      v_VECTOR.clear();
      begin_log_simulation = false;
      }    
    }

    if (start_simulation)
    {
      start_simulation = false;
      ROS_INFO("Started to log data which will last %f seconds", simulation_time_total);
      time_start_simulation = (ros::Time::now()).toSec();
      begin_log_simulation = true;
    }

    //--------------------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------------------

    // if (verbose_){
    //   tf::vectorEigenToTF(position, tf_pos_);
    //   ROS_INFO_STREAM_THROTTLE(0.1, "\nCARTESIAN POSITION:\n" << position);
    //   tf_br_transform_.setOrigin(tf_pos_);
    //   tf::quaternionEigenToTF(orientation, tf_rot_);
    //   tf_br_transform_.setRotation(tf_rot_);
    //   tf_br_.sendTransform(tf::StampedTransform(tf_br_transform_, ros::Time::now(), "world", "fk_ee"));
    // }

    // compute error to desired pose
    // position error
    Eigen::VectorXd tau_d(7), tau_task(7), tau_nullspace(7), tau_wrench(7);
    Eigen::Matrix<double, 6, 1> error;

    //applying wrench through dynamic parameters
    if (apply_wrench)
    {
      tau_wrench << jacobian.transpose() * f;
    }
    else
    {
      for (int i = 0; i < 7; i++)
      {
        tau_wrench(i) = 0;
      }
    }

    base_tools.update_control(q, dq, position, orientation, jacobian, tau_d, tau_task, tau_nullspace, tau_wrench, error);
    tau_d << saturateTorqueRate(tau_d, tau_J_d_);
    // compute error to desired pose
    // position error
    tf::vectorEigenToTF(Eigen::Vector3d(error.head(3)), tf_pos_);
    tf_br_transform_.setOrigin(tf_pos_);

    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_d(i));
      // saves last desired torque. These values used to came from the panda.
      tau_J_d_[i] = tau_d(i);
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
      ROS_INFO_STREAM_THROTTLE(0.1, "\ntau_task:\n"
                                        << tau_task);
      ROS_INFO_STREAM_THROTTLE(0.1, "\ntau_nullspace:\n"
                                        << tau_nullspace);
    }

    publish();

    base_tools.update_parameters(filter_params_, nullspace_stiffness_,
                                 nullspace_stiffness_target_, delta_tau_max_, cartesian_stiffness_,
                                 cartesian_stiffness_target_, cartesian_damping_,
                                 cartesian_damping_target_, q_d_nullspace_,
                                 q_d_nullspace_target_, position_d_, orientation_d_,
                                 position_d_target_, orientation_d_target_);
  }
  Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
      const Eigen::Matrix<double, 7, 1> &tau_J_d)
  { // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void CartesianImpedanceController::ee_poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0)
    {
      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }
  }
  //for logging data
  //------------------------------------------------------------------------------------------------------
  void CartesianImpedanceController::latest_requestCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {

    latest_poseStamped_request.pose.position.x = msg->pose.position.x;
    latest_poseStamped_request.pose.position.y = msg->pose.position.y;
    latest_poseStamped_request.pose.position.z = msg->pose.position.z;
    latest_poseStamped_request.pose.orientation.x = msg->pose.orientation.x;
    latest_poseStamped_request.pose.orientation.y = msg->pose.orientation.y;
    latest_poseStamped_request.pose.orientation.z = msg->pose.orientation.z;
    latest_poseStamped_request.pose.orientation.w = msg->pose.orientation.w;

    position_new_request << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    distance_to_goal = (position_new_request - position_d_).norm();

    is_new_request = true;
  }

  void CartesianImpedanceController::logCallback(cartesian_impedance_controller::log_configConfig &config, uint32_t level)
  {
    file_name_trajectory = config.file_name_trajectory;
    print_title_trajectory = config.print_title_trajectory;
    over_write_trajectory = config.overwrite_trajectory;
    file_name_simulation = config.file_name_simulation;
    print_title_simulation = config.print_title_simulation;
    over_write_simulation = config.overwrite_simulation;
    if (!begin_log_simulation)
    {
      simulation_time_total = config.simulation_time;
      if (config.start_simulation)
      {
        start_simulation = config.start_simulation;
      }
    }
    config.start_simulation = false;
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
      position_d_target_ = translation;
      orientation_d_target_ = orientation;
      // Update nullspace
      q_d_nullspace_target_ = q;
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

  void CartesianImpedanceController::complianceParamCallback()
  {

    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << translational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << rotational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.setIdentity();
    // Damping ratio = 1
    cartesian_damping_target_.topLeftCorner(3, 3)
        << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();
    nullspace_stiffness_target_ = nullspace_stiffness;
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

    translational_stiffness = config.translational_stiffness;
    rotational_stiffness = config.rotational_stiffness;
    nullspace_stiffness = config.nullspace_stiffness;
    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << config.translational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << config.rotational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.setIdentity();
    // Damping ratio = 1
    cartesian_damping_target_.topLeftCorner(3, 3)
        << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
    nullspace_stiffness_target_ = config.nullspace_stiffness;
  }

  void CartesianImpedanceController::dynamicWrenchCallback(cartesian_impedance_controller::wrench_configConfig &config, uint32_t level)
  {
    Eigen::MatrixXd temp(6, 1);
    temp << config.f_x, config.f_y, config.f_z, config.tau_x, config.tau_y, config.tau_z;
    f.resizeLike(temp);
    f << temp;
    apply_wrench = config.apply_wrench;
  }
  //--------------------------------------------------------------------------------------------------------------------------------------
  void CartesianImpedanceController::equilibriumPoseCallback(
      const geometry_msgs::PoseStampedConstPtr &msg)
  {
    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0)
    {
      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }
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