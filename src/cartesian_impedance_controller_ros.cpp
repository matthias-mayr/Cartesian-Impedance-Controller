#include <cartesian_impedance_controller/cartesian_impedance_controller_ros.h>

namespace cartesian_impedance_controller
{
  // Saturate a variable x with the limits x_min and x_max
  double saturateValue(double x, double x_min, double x_max)
  {
    return std::min(std::max(x, x_min), x_max);
  }

  bool CartesianImpedanceControllerRos::initDynamicReconfigure(const ros::NodeHandle &nh)
  {
    dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<cartesian_impedance_controller::impedance_configConfig>>(ros::NodeHandle(std::string(nh.getNamespace() + "/stiffness_reconfigure")));
    dynamic_server_compliance_param_->setCallback(
        boost::bind(&CartesianImpedanceControllerRos::dynamicConfigCb, this, _1, _2));

    dynamic_server_damping_param_ = std::make_unique<dynamic_reconfigure::Server<cartesian_impedance_controller::damping_configConfig>>(ros::NodeHandle(std::string(nh.getNamespace() + "/damping_factors_reconfigure")));
    dynamic_server_damping_param_->setCallback(
        boost::bind(&CartesianImpedanceControllerRos::dynamicDampingCb, this, _1, _2));

    dynamic_server_wrench_param_ = std::make_unique<dynamic_reconfigure::Server<cartesian_impedance_controller::wrench_configConfig>>(ros::NodeHandle(std::string(nh.getNamespace() + "/cartesian_wrench_reconfigure")));
    dynamic_server_wrench_param_->setCallback(
        boost::bind(&CartesianImpedanceControllerRos::dynamicWrenchCb, this, _1, _2));
    return true;
  }

  bool CartesianImpedanceControllerRos::initJointHandles(hardware_interface::EffortJointInterface *hw, const ros::NodeHandle &nh)
  {
    std::vector<std::string> joint_names;
    if (!nh.getParam("joints", joint_names))
    {
      ROS_ERROR("Invalid or no joint_names parameters provided, aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      try
      {
        joint_handles_.push_back(hw->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM("Exception getting joint handles: " << ex.what());
        return false;
      }
    }
    this->n_joints_ = joint_names.size();
    return true;
  }

  bool CartesianImpedanceControllerRos::initMessaging(ros::NodeHandle &nh)
  {
    sub_cart_stiffness_ = nh.subscribe("set_cartesian_stiffness", 1,
                                       &CartesianImpedanceControllerRos::stiffnessCb, this);
    sub_cart_wrench_ = nh.subscribe("set_cartesian_wrench", 1,
                                    &CartesianImpedanceControllerRos::wrenchCommandCb, this);
    sub_damping_ = nh.subscribe("set_damping_factors", 1,
                                &CartesianImpedanceControllerRos::dampingCb, this);
    sub_impedance_config_ =
        nh.subscribe("set_stiffness", 1, &CartesianImpedanceControllerRos::impedanceControlCb, this);
    sub_reference_pose_ = nh.subscribe("target_pose", 1, &CartesianImpedanceControllerRos::referencePoseCb, this);

    pub_torques_.init(nh, "commanded_torques", 20);
    pub_torques_.msg_.layout.dim.resize(1);
    pub_torques_.msg_.layout.data_offset = 0;
    pub_torques_.msg_.layout.dim[0].size = n_joints_;
    pub_torques_.msg_.layout.dim[0].stride = 0;
    pub_torques_.msg_.data.resize(n_joints_);
    return true;
  }

  bool CartesianImpedanceControllerRos::initRBDyn(const ros::NodeHandle &nh)
  {
    // Get the URDF XML from the parameter server. Wait if needed.
    std::string urdf_string;
    nh.param<std::string>("robot_description", robot_description_, "/robot_description");
    while (urdf_string.empty())
    {
      ROS_INFO_ONCE("Waiting for robot description in parameter %s on the ROS param server.",
                    robot_description_.c_str());
      nh.getParam(robot_description_, urdf_string);
      usleep(100000);
    }
    rbdyn_wrapper_.init_rbdyn(urdf_string, end_effector_);
    if (this->rbdyn_wrapper_.n_joints() < this->n_joints_)
    {
      ROS_ERROR("Number of joints in the URDF is smaller than supplied number of joints. %i < %i", this->rbdyn_wrapper_.n_joints(), this->n_joints_);
      return false;
    }
    ROS_INFO_STREAM("Number of joints found in urdf: " << this->rbdyn_wrapper_.n_joints());
    return true;
  }

  bool CartesianImpedanceControllerRos::initTrajectories(ros::NodeHandle &nh)
  {
    sub_trajectory_ = nh.subscribe("command", 1, &CartesianImpedanceControllerRos::trajCb, this);
    traj_as_ = std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(
        new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
            nh, std::string("follow_joint_trajectory"), false));
    traj_as_->registerGoalCallback(boost::bind(&CartesianImpedanceControllerRos::trajGoalCb, this));
    traj_as_->registerPreemptCallback(boost::bind(&CartesianImpedanceControllerRos::trajPreemptCb, this));
    traj_as_->start();
    return true;
  }

  bool CartesianImpedanceControllerRos::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle)
  {
    ROS_INFO("Initializing Cartesian impedance controller in namespace: %s", node_handle.getNamespace().c_str());

    // Fetch parameters
    node_handle.param<bool>("verbose", verbose_, false);
    node_handle.param<std::string>("end_effector", end_effector_, "iiwa_link_ee");
    ROS_INFO_STREAM("End effektor link is: " << end_effector_);
    // Frames for applying commanded Cartesian wrenches
    node_handle.param<std::string>("from_frame_wrench", from_frame_wrench_, "world");
    node_handle.param<std::string>("to_frame_wrench", to_frame_wrench_, end_effector_);
    bool dynamic_reconfigure{true};
    node_handle.param<bool>("dynamic_reconfigure", dynamic_reconfigure, true);
    bool enable_trajectories{true};
    node_handle.param<bool>("handle_trajectories", enable_trajectories, true);
    double delta_tau_max{1.};
    node_handle.param<double>("delta_tau_max", delta_tau_max, 1.);
    node_handle.param<double>("update_frequency", update_frequency_, 500.);
    node_handle.param<double>("filtering/stiffness", filtering_stiffness_, 0.1);
    node_handle.param<double>("filtering/pose", filtering_pose_, 0.1);
    node_handle.param<double>("filtering/wrench", filtering_wrench_, 0.1);

    if (!this->initJointHandles(hw, node_handle) || !this->initMessaging(node_handle) || !this->initRBDyn(node_handle))
    {
      return false;
    }
    if (dynamic_reconfigure && !this->initDynamicReconfigure(node_handle))
    {
      return false;
    }
    if (enable_trajectories && !this->initTrajectories(node_handle))
    {
      return false;
    }

    base_tools_ = std::make_unique<CartesianImpedanceController>(this->n_joints_);
    base_tools_->setMaxTorqueDelta(delta_tau_max);

    // Size members
    q_ = Eigen::VectorXd(this->n_joints_);
    dq_ = Eigen::VectorXd(this->n_joints_);
    jacobian_ = Eigen::MatrixXd(6, joint_handles_.size());

    base_tools_->setFiltering(update_frequency_, filtering_stiffness_, filtering_pose_, filtering_wrench_);

    //Initialize publisher of useful data
    pub_data_export_ =
        node_handle.advertise<cartesian_impedance_controller::RobotImpedanceState>("useful_data_to_analyze", 1);

    return true;
  }

  void CartesianImpedanceControllerRos::starting(const ros::Time & /*time*/)
  {
    ROS_INFO("Starting Cartesian Impedance Controller");
    this->updateState();

    // set x_attractor and q_d_nullspace
    base_tools_->setDesiredPose(position_, orientation_);
    base_tools_->setNullspaceConfig(q_);
  }

  void CartesianImpedanceControllerRos::update(const ros::Time & /*time*/, const ros::Duration &period /*period*/)

  {
    if (traj_running_)
    {
      trajUpdate();
    }

    this->updateState();

    // Apply control law in base library
    this->tau_J_d_ = base_tools_->calculateCommandedTorques(q_, dq_, position_, orientation_, jacobian_);

    // Get the updated controller state
    base_tools_->getState(&position_d_, &orientation_d_, &cartesian_stiffness_, &nullspace_stiffness_, &q_d_nullspace_,
                          &cartesian_damping_);

    for (size_t i = 0; i < this->n_joints_; ++i)
    {
      joint_handles_[i].setCommand(this->tau_J_d_(i));
    }

    if (verbose_)
    {
      tf::vectorEigenToTF(Eigen::Vector3d(base_tools_->getPoseError().head(3)), tf_pos_);
      tf_br_transform_.setOrigin(tf_pos_);

      tf::vectorEigenToTF(position_, tf_pos_);
      ROS_INFO_STREAM_THROTTLE(0.1, "\nCARTESIAN POSITION:\n"
                                        << position_);
      tf_br_transform_.setOrigin(tf_pos_);
      tf::quaternionEigenToTF(orientation_, tf_rot_);
      tf_br_transform_.setRotation(tf_rot_);
      tf_br_.sendTransform(tf::StampedTransform(tf_br_transform_, ros::Time::now(), "world", "fk_ee"));

      Eigen::Matrix<double, 6, 1> dx;
      dx << jacobian_ * dq_;
      double cartesian_velocity = sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));
      ROS_INFO_STREAM_THROTTLE(0.1, "\nERROR:\n"
                                        << base_tools_->getPoseError());
      ROS_INFO_STREAM_THROTTLE(0.1, "\nParameters:\nCartesian Stiffness:\n"
                                        << cartesian_stiffness_ << "\nCartesian damping:\n"
                                        << cartesian_damping_ << "\nNullspace stiffness:\n"
                                        << nullspace_stiffness_ << "\nq_d_nullspace:\n"
                                        << q_d_nullspace_);
      ROS_INFO_STREAM_THROTTLE(0.1, "\ntau_d:\n"
                                        << tau_J_d_);
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

    publish();

    //publish useful data to a topic
    // publishData(q, dq, position, orientation, position_d_, orientation_d_, tau_d_, cartesian_stiffness_,
    // nullspace_stiffness_, error, base_tools_->getAppliedWrench(), cartesian_velocity);
  }

  bool CartesianImpedanceControllerRos::getFk(const Eigen::VectorXd &q, Eigen::Vector3d &position,
                                              Eigen::Quaterniond &orientation)
  {
    rbdyn_wrapper::EefState ee_state;
    if (this->rbdyn_wrapper_.n_joints() != this->n_joints_)
    {
      Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      q_rb.head(q.size()) = q;
      ee_state = this->rbdyn_wrapper_.perform_fk(q_rb);
    }
    else
    {
      ee_state = this->rbdyn_wrapper_.perform_fk(q);
    }
    position = ee_state.translation;
    orientation = ee_state.orientation;
    return true;
  }

  bool CartesianImpedanceControllerRos::getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                                    Eigen::MatrixXd &jacobian)
  {
    if (this->rbdyn_wrapper_.n_joints() != this->n_joints_)
    {
      Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      q_rb.head(q.size()) = q;
      Eigen::VectorXd dq_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      dq_rb.head(dq.size()) = dq;
      jacobian = this->rbdyn_wrapper_.jacobian(q_rb, dq_rb);
    }
    else
    {
      jacobian = this->rbdyn_wrapper_.jacobian(q, dq);
    }
    jacobian = jacobian_perm_ * jacobian;
    return true;
  }

  void CartesianImpedanceControllerRos::updateState()
  {
    for (size_t i = 0; i < this->n_joints_; ++i)
    {
      q_[i] = joint_handles_[i].getPosition();
      dq_[i] = joint_handles_[i].getVelocity();
    }
    getJacobian(q_, dq_, jacobian_);
    getFk(q_, position_, orientation_);
  }

  void CartesianImpedanceControllerRos::dampingCb(
      const cartesian_impedance_controller::CartesianImpedanceControlMode &msg)
  {
    double dmp_max = 1;
    double dmp_min = 0.1;
    base_tools_->setDamping(saturateValue(msg.cartesian_damping.x, dmp_min, dmp_max),
                            saturateValue(msg.cartesian_damping.y, dmp_min, dmp_max),
                            saturateValue(msg.cartesian_damping.z, dmp_min, dmp_max),
                            saturateValue(msg.cartesian_damping.a, dmp_min, dmp_max),
                            saturateValue(msg.cartesian_damping.b, dmp_min, dmp_max),
                            saturateValue(msg.cartesian_damping.c, dmp_min, dmp_max), msg.nullspace_damping);
  }

  void CartesianImpedanceControllerRos::impedanceControlCb(
      const cartesian_impedance_controller::CartesianImpedanceControlMode &msg)
  {
    double trans_stf_max = 2000;
    double trans_stf_min = 0;
    double rot_stf_max = 500;
    double rot_stf_min = 0;
    base_tools_->setStiffness(saturateValue(msg.cartesian_stiffness.x, trans_stf_min, trans_stf_max),
                              saturateValue(msg.cartesian_stiffness.y, trans_stf_min, trans_stf_max),
                              saturateValue(msg.cartesian_stiffness.z, trans_stf_min, trans_stf_max),
                              saturateValue(msg.cartesian_stiffness.a, trans_stf_min, trans_stf_max),
                              saturateValue(msg.cartesian_stiffness.b, trans_stf_min, trans_stf_max),
                              saturateValue(msg.cartesian_stiffness.c, trans_stf_min, trans_stf_max),
                              msg.nullspace_stiffness);
    Eigen::VectorXd q_d_nullspace_target_;
    q_d_nullspace_target_ << msg.q_d_nullspace.q1, msg.q_d_nullspace.q2, msg.q_d_nullspace.q3, msg.q_d_nullspace.q4,
        msg.q_d_nullspace.q5, msg.q_d_nullspace.q6, msg.q_d_nullspace.q7;
    base_tools_->setNullspaceConfig(q_d_nullspace_target_);
  }

  void CartesianImpedanceControllerRos::referencePoseCb(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_);
    orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
        msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_.coeffs()) < 0.0)
    {
      orientation_d_.coeffs() << -orientation_d_.coeffs();
    }
    base_tools_->setDesiredPose(position_d_, orientation_d_);
  }

  void CartesianImpedanceControllerRos::stiffnessCb(const geometry_msgs::WrenchStampedConstPtr &msg)
  {
    double trans_stf_max = 2000;
    double trans_stf_min = 0;
    double rot_stf_max = 500;
    double rot_stf_min = 0;
    base_tools_->setStiffness(saturateValue(msg->wrench.force.x, trans_stf_min, trans_stf_max),
                              saturateValue(msg->wrench.force.y, trans_stf_min, trans_stf_max),
                              saturateValue(msg->wrench.force.z, trans_stf_min, trans_stf_max),
                              saturateValue(msg->wrench.torque.x, trans_stf_min, trans_stf_max),
                              saturateValue(msg->wrench.torque.y, trans_stf_min, trans_stf_max),
                              saturateValue(msg->wrench.torque.z, trans_stf_min, trans_stf_max));
  }

  //Adds a wrench at the end-effector, using the world frame
  void CartesianImpedanceControllerRos::wrenchCommandCb(const geometry_msgs::WrenchStampedConstPtr &msg)
  {
    Eigen::Matrix<double, 6, 1> F;
    F << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
        msg->wrench.torque.z;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "world")
    {
      transformWrench(F, msg->header.frame_id, to_frame_wrench_);
    }
    else if (msg->header.frame_id.empty())
    {
      transformWrench(F, from_frame_wrench_, to_frame_wrench_);
    }
    base_tools_->applyWrench(F);
  }

  // Transform a Cartesian wrench from "from_frame" to "to_frame". E.g. from_frame= "world" , to_frame = "bh_link_ee"
  void CartesianImpedanceControllerRos::transformWrench(Eigen::Matrix<double, 6, 1> &cartesian_wrench,
                                                        std::string from_frame, std::string to_frame)
  {
    try
    {
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

  //Publish data to export and analyze
  void CartesianImpedanceControllerRos::publishData(Eigen::VectorXd q, Eigen::VectorXd dq, Eigen::Vector3d position,
                                                    Eigen::Quaterniond orientation, Eigen::Vector3d position_d_,
                                                    Eigen::Quaterniond orientation_d_, Eigen::VectorXd tau_d,
                                                    Eigen::Matrix<double, 6, 6> cartesian_stiffness_,
                                                    double nullspace_stiffness_, Eigen::Matrix<double, 6, 1> error,
                                                    Eigen::Matrix<double, 6, 1> F, double cartesian_velocity)
  {
    cartesian_impedance_controller::RobotImpedanceState data_to_analyze;
    data_to_analyze.time = ros::Time::now().toSec();
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

  void CartesianImpedanceControllerRos::publish()
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

  //Dynamic reconfigure
  //--------------------------------------------------------------------------------------------------------------------------------------
  void CartesianImpedanceControllerRos::dynamicConfigCb(
      cartesian_impedance_controller::impedance_configConfig &config, uint32_t level)
  {
    if (config.apply_stiffness)
    {
      double trans_stf_max = 2000;
      double trans_stf_min = 0;
      double rot_stf_max = 300;
      double rot_stf_min = 0;
      base_tools_->setStiffness(saturateValue(config.translation_x, trans_stf_min, trans_stf_max),
                                saturateValue(config.translation_y, trans_stf_min, trans_stf_max),
                                saturateValue(config.translation_z, trans_stf_min, trans_stf_max),
                                saturateValue(config.rotation_x, trans_stf_min, trans_stf_max),
                                saturateValue(config.rotation_y, trans_stf_min, trans_stf_max),
                                saturateValue(config.rotation_z, trans_stf_min, trans_stf_max), config.nullspace_stiffness);
    }
  }

  void CartesianImpedanceControllerRos::dynamicDampingCb(
      cartesian_impedance_controller::damping_configConfig &config, uint32_t level)
  {
    double dmp_max = 1;
    double dmp_min = 0.1;
    if (config.apply_damping_factors)
    {
      base_tools_->setDamping(
          saturateValue(config.translation_x, dmp_min, dmp_max), saturateValue(config.translation_y, dmp_min, dmp_max),
          saturateValue(config.translation_z, dmp_min, dmp_max), saturateValue(config.rotation_x, dmp_min, dmp_max),
          saturateValue(config.rotation_y, dmp_min, dmp_max), saturateValue(config.rotation_z, dmp_min, dmp_max),
          config.nullspace_damping);
    }
  }

  void CartesianImpedanceControllerRos::dynamicWrenchCb(cartesian_impedance_controller::wrench_configConfig &config,
                                                        uint32_t level)
  {
    if (config.apply_wrench)
    {
      Eigen::Vector6d F;
      F << config.f_x, config.f_y, config.f_z, config.tau_x, config.tau_y, config.tau_z;
      transformWrench(F, from_frame_wrench_, to_frame_wrench_);
      base_tools_->applyWrench(F);
    }
    else
    {
      Eigen::Vector6d F;
      F << 0., 0., 0., 0., 0., 0.;
      transformWrench(F, from_frame_wrench_, to_frame_wrench_);
      base_tools_->applyWrench(F);
    }
  }

  void CartesianImpedanceControllerRos::trajStart(const trajectory_msgs::JointTrajectory &trajectory)
  {
    traj_duration_ = trajectory.points[trajectory.points.size() - 1].time_from_start;
    ROS_INFO_STREAM("Got a new trajectory with " << trajectory.points.size() << " points that takes " << traj_duration_
                                                 << "s.");
    trajectory_ = trajectory;
    traj_running_ = true;
    traj_start_ = ros::Time::now();
    traj_index_ = 0;
    trajUpdate();
  }

  void CartesianImpedanceControllerRos::trajUpdate()
  {
    if (ros::Time::now() > (traj_start_ + trajectory_.points[traj_index_].time_from_start))
    {
      // Get end effector pose
      Eigen::VectorXd q = Eigen::VectorXd::Map(trajectory_.points[traj_index_].positions.data(),
                                               trajectory_.points[traj_index_].positions.size());
      ROS_INFO_STREAM("Index " << traj_index_ << " q_nullspace: " << q.transpose());
      Eigen::Vector3d translation;
      Eigen::Quaterniond orientation;
      getFk(q, translation, orientation);
      // Update end effector pose
      position_d_ = translation;
      orientation_d_ = orientation;
      // Update nullspace
      base_tools_->setNullspaceConfig(q);
      traj_index_++;
    }

    if (ros::Time::now() > (traj_start_ + traj_duration_))
    {
      ROS_INFO_STREAM("Finished executing trajectory.");
      if (traj_as_->isActive())
      {
        traj_as_->setSucceeded();
      }
      traj_running_ = false;
      return;
    }
  }

  void CartesianImpedanceControllerRos::trajGoalCb()
  {
    traj_goal_ = traj_as_->acceptNewGoal();
    ROS_INFO("Accepted new goal");
    trajStart(traj_goal_->trajectory);
  }

  void CartesianImpedanceControllerRos::trajPreemptCb()
  {
    ROS_INFO("Actionserver got preempted.");
  }

  void CartesianImpedanceControllerRos::trajCb(const trajectory_msgs::JointTrajectoryConstPtr &msg)
  {
    ROS_INFO("Got trajectory msg");
    trajStart(*msg);
  }

} // namespace cartesian_impedance_controller
