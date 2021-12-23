#include <cartesian_impedance_controller/cartesian_impedance_controller_ros.h>

namespace cartesian_impedance_controller
{
  // Saturate a variable x with the limits x_min and x_max
  double saturateValue(double x, double x_min, double x_max)
  {
    return std::min(std::max(x, x_min), x_max);
  }

  void EigenVectorToWrench(const Eigen::Matrix<double, 6, 1>& v, geometry_msgs::Wrench *wrench)
  {
    wrench->force.x = v(0);
    wrench->force.y = v(1);
    wrench->force.z = v(2);
    wrench->torque.x = v(4);
    wrench->torque.y = v(5);
    wrench->torque.z = v(6);
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
    ROS_INFO_STREAM("Number of joints specified in parameters: " << this->n_joints_);
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
        nh.subscribe("set_config", 1, &CartesianImpedanceControllerRos::controllerConfigCb, this);
    sub_reference_pose_ = nh.subscribe("target_pose", 1, &CartesianImpedanceControllerRos::referencePoseCb, this);

    pub_torques_.init(nh, "commanded_torques", 20);
    pub_torques_.msg_.layout.dim.resize(1);
    pub_torques_.msg_.layout.data_offset = 0;
    pub_torques_.msg_.layout.dim[0].size = n_joints_;
    pub_torques_.msg_.layout.dim[0].stride = 0;
    pub_torques_.msg_.data.resize(n_joints_);

    std::vector<std::string> joint_names;
    nh.getParam("joints", joint_names);
    pub_state_.init(nh, "controller_state", 10);
    pub_state_.msg_.header.seq = 0;
    for (size_t i = 0; i < this->n_joints_; i++)
    {
      pub_state_.msg_.joint_state.name.push_back(joint_names.at(i));
    }
    pub_state_.msg_.joint_state.position = std::vector<double>(this->n_joints_);
    pub_state_.msg_.joint_state.velocity = std::vector<double>(this->n_joints_);
    pub_state_.msg_.joint_state.effort = std::vector<double>(this->n_joints_);
    pub_state_.msg_.nullspace_config = std::vector<double>(this->n_joints_);
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
    try
    {
      rbdyn_wrapper_.init_rbdyn(urdf_string, end_effector_);
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR("Error when intializing RBDyn: %s", e.what());
      return false;
    }
    ROS_INFO_STREAM("Number of joints found in urdf: " << this->rbdyn_wrapper_.n_joints());
    if (this->rbdyn_wrapper_.n_joints() < this->n_joints_)
    {
      ROS_ERROR("Number of joints in the URDF is smaller than supplied number of joints. %i < %i", this->rbdyn_wrapper_.n_joints(), this->n_joints_);
      return false;
    }
    else if (this->rbdyn_wrapper_.n_joints() < this->n_joints_)
    {
      ROS_WARN("Number of joints in the URDF is greater than supplied number of joints: %i > %i. Assuming that the actuated joints come first.", this->rbdyn_wrapper_.n_joints(), this->n_joints_);
    }
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
    node_handle.param<bool>("verbosity/verbose_print", verbose_print_, false);
    node_handle.param<bool>("verbosity/state_msgs", verbose_state_, false);
    node_handle.param<bool>("verbosity/tf_frames", verbose_tf_, false);

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

    // Initialize base_tools and member variables
    base_tools_ = std::make_unique<CartesianImpedanceController>(this->n_joints_);
    base_tools_->setMaxTorqueDelta(delta_tau_max);
    if (this->n_joints_ < 6)
    {
      ROS_WARN("Number of joints is below 6. Functions might be limited.");
    }
    if (this->n_joints_ < 7)
    {
      ROS_WARN("Number of joints is below 7. No redundant joint for nullspace.");
    }
    q_ = Eigen::VectorXd(this->n_joints_);
    dq_ = Eigen::VectorXd(this->n_joints_);
    q_d_nullspace_ = Eigen::VectorXd(this->n_joints_);
    jacobian_ = Eigen::MatrixXd(6, joint_handles_.size());

    base_tools_->setFiltering(update_frequency_, filtering_stiffness_, filtering_pose_, filtering_wrench_);

    return true;
  }

  void CartesianImpedanceControllerRos::starting(const ros::Time & /*time*/)
  {
    this->updateState();

    // set x_attractor and q_d_nullspace
    base_tools_->setDesiredPose(position_, orientation_);
    base_tools_->setNullspaceConfig(q_);
    ROS_INFO("Started Cartesian Impedance Controller");
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

    // Get the updated internal controller state
    base_tools_->getState(&position_d_, &orientation_d_, &cartesian_stiffness_, &nullspace_stiffness_, &q_d_nullspace_,
                          &cartesian_damping_);

    // Write commands
    for (size_t i = 0; i < this->n_joints_; ++i)
    {
      joint_handles_[i].setCommand(this->tau_J_d_(i));
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

  void CartesianImpedanceControllerRos::controllerConfigCb(const cartesian_impedance_controller::ControllerConfigConstPtr &msg)
  {
    this->setStiffness(msg->cartesian_stiffness, msg->nullspace_stiffness);
    this->setDamping(msg->cartesian_damping, msg->nullspace_damping);

    if (msg->q_d_nullspace.size() == this->n_joints_)
    {
      for (size_t i = 0; i < this->n_joints_; i++)
      {
        this->q_d_nullspace_(i) = msg->q_d_nullspace.at(i);
      }
      base_tools_->setNullspaceConfig(this->q_d_nullspace_);
    }
  }

  void CartesianImpedanceControllerRos::dampingCb(const geometry_msgs::WrenchConstPtr &msg)
  {
    this->setDamping(*msg, this->nullspace_damping_);
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
    this->setStiffness(msg->wrench, this->nullspace_stiffness_);
  }

  void CartesianImpedanceControllerRos::setDamping(const geometry_msgs::Wrench &cart_stiffness, double nullspace)
  {
    constexpr double dmp_min = 0.0;
    constexpr double dmp_max = 1;
    base_tools_->setDamping(saturateValue(cart_stiffness.force.x, dmp_min, dmp_max),
                            saturateValue(cart_stiffness.force.y, dmp_min, dmp_max),
                            saturateValue(cart_stiffness.force.z, dmp_min, dmp_max),
                            saturateValue(cart_stiffness.torque.x, dmp_min, dmp_max),
                            saturateValue(cart_stiffness.torque.y, dmp_min, dmp_max),
                            saturateValue(cart_stiffness.torque.z, dmp_min, dmp_max),
                            saturateValue(nullspace, dmp_min, dmp_max));
  }

  void CartesianImpedanceControllerRos::setStiffness(const geometry_msgs::Wrench &cart_stiffness, double nullspace)
  {
    constexpr double trans_stf_min = 0;
    constexpr double trans_stf_max = 2000;
    constexpr double rot_stf_min = 0;
    constexpr double rot_stf_max = 500;
    constexpr double ns_min = 0;
    constexpr double ns_max = 10000;
    base_tools_->setStiffness(saturateValue(cart_stiffness.force.x, trans_stf_min, trans_stf_max),
                              saturateValue(cart_stiffness.force.y, trans_stf_min, trans_stf_max),
                              saturateValue(cart_stiffness.force.z, trans_stf_min, trans_stf_max),
                              saturateValue(cart_stiffness.torque.x, rot_stf_min, rot_stf_max),
                              saturateValue(cart_stiffness.torque.y, rot_stf_min, rot_stf_max),
                              saturateValue(cart_stiffness.torque.z, rot_stf_min, rot_stf_max),
                              saturateValue(nullspace, ns_min, ns_max));
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

  void CartesianImpedanceControllerRos::publish()
  {
    // publish commanded torques
    if (pub_torques_.trylock())
    {
      for (unsigned i = 0; i < this->n_joints_; i++)
      {
        pub_torques_.msg_.data[i] = tau_J_d_[i];
      }
      pub_torques_.unlockAndPublish();
    }

    const Eigen::Matrix<double, 6, 1> error{base_tools_->getPoseError()};

    if (verbose_print_)
    {
      ROS_INFO_STREAM_THROTTLE(0.1, "\nCARTESIAN POSITION:\n"
                                        << position_ << "\nERROR:\n"
                                        << error << "\nCartesian Stiffness:\n"
                                        << cartesian_stiffness_ << "\nCartesian damping:\n"
                                        << cartesian_damping_ << "\nNullspace stiffness:\n"
                                        << nullspace_stiffness_ << "\nq_d_nullspace:\n"
                                        << q_d_nullspace_ << "\ntau_d:\n"
                                        << tau_J_d_);
    }
    if (verbose_tf_)
    {

      tf::vectorEigenToTF(Eigen::Vector3d(base_tools_->getPoseError().head(3)), tf_pos_);
      tf_br_transform_.setOrigin(tf_pos_);

      tf::vectorEigenToTF(position_, tf_pos_);
      tf_br_transform_.setOrigin(tf_pos_);
      tf::quaternionEigenToTF(orientation_, tf_rot_);
      tf_br_transform_.setRotation(tf_rot_);
      tf_br_.sendTransform(tf::StampedTransform(tf_br_transform_, ros::Time::now(), "world", "fk_ee"));
      // Publish tf to the reference pose
      tf::vectorEigenToTF(position_d_, tf_pos_);
      tf_br_transform_.setOrigin(tf_pos_);
      tf::quaternionEigenToTF(orientation_d_, tf_rot_);
      tf_br_transform_.setRotation(tf_rot_);
      tf_br_.sendTransform(tf::StampedTransform(tf_br_transform_, ros::Time::now(), "world", this->end_effector_ + "_ref_pose"));
    }
    if (verbose_state_)
    {
      if (pub_state_.trylock())
      {
        pub_state_.msg_.header.stamp = ros::Time::now();
        tf::pointEigenToMsg(this->position_, pub_state_.msg_.current_pose.position);
        tf::quaternionEigenToMsg(this->orientation_, pub_state_.msg_.current_pose.orientation);
        tf::pointEigenToMsg(this->position_d_, pub_state_.msg_.reference_pose.position);
        tf::quaternionEigenToMsg(this->orientation_d_, pub_state_.msg_.reference_pose.orientation);
        tf::pointEigenToMsg(error.head(3), pub_state_.msg_.pose_error.position);
        Eigen::Quaterniond q = Eigen::AngleAxisd(error(4), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(error(5), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(error(6), Eigen::Vector3d::UnitZ());
        tf::quaternionEigenToMsg(q, pub_state_.msg_.pose_error.orientation);

        EigenVectorToWrench(this->cartesian_stiffness_.diagonal(), &pub_state_.msg_.cartesian_stiffness);
        EigenVectorToWrench(this->cartesian_damping_.diagonal(), &pub_state_.msg_.cartesian_damping);
        EigenVectorToWrench(base_tools_->getAppliedWrench(), &pub_state_.msg_.commanded_wrench);

        for (size_t i = 0; i < this->n_joints_; i++)
        {
          pub_state_.msg_.joint_state.position.at(i) = q_(i);
          pub_state_.msg_.joint_state.velocity.at(i) = dq_(i);
          pub_state_.msg_.joint_state.effort.at(i) = tau_J_d_(i);
          pub_state_.msg_.nullspace_config.at(i) = q_d_nullspace_(i);
        }
        pub_state_.msg_.nullspace_stiffness = this->nullspace_stiffness_;
        pub_state_.msg_.nullspace_damping = this->nullspace_damping_;
        Eigen::Matrix<double, 6, 1> dx = this->jacobian_ * this->dq_;
        pub_state_.msg_.cartesian_velocity = sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));

        pub_state_.unlockAndPublish();
        pub_state_.msg_.header.seq++;
      }
    }
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
    ROS_INFO("Got trajectory msg ");
    trajStart(*msg);
  }

} // namespace cartesian_impedance_controller
