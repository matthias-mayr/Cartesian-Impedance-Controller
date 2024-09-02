#include <cartesian_impedance_controller/cartesian_impedance_controller_ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

namespace cartesian_impedance_controller
{
  /*! \brief Saturate a variable x with the limits x_min and x_max
    *
    * \param[in] x Value
    * \param[in] x_min Minimal value
    * \param[in] x_max Maximum value
    * \return Saturated value
    */
  double saturateValue(double x, double x_min, double x_max)
  {
    return std::min(std::max(x, x_min), x_max);
  }

  /*! \brief Populates a wrench msg with value from Eigen vector
    *
    * It is assumed that the vector has the form transl_x, transl_y, transl_z, rot_x, rot_y, rot_z
    * \param[in] v Input vector
    * \param[out] wrench Wrench message
    */
  void EigenVectorToWrench(const Eigen::Matrix<double, 6, 1> &v, geometry_msgs::Wrench *wrench)
  {
    wrench->force.x = v(0);
    wrench->force.y = v(1);
    wrench->force.z = v(2);
    wrench->torque.x = v(3);
    wrench->torque.y = v(4);
    wrench->torque.z = v(5);
  }

  bool CartesianImpedanceControllerRos::initDynamicReconfigure(const ros::NodeHandle &nh)
  {
    this->dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<cartesian_impedance_controller::stiffnessConfig>>(ros::NodeHandle(std::string(nh.getNamespace() + "/stiffness_reconfigure")));
    this->dynamic_server_compliance_param_->setCallback(
        boost::bind(&CartesianImpedanceControllerRos::dynamicStiffnessCb, this, _1, _2));

    this->dynamic_server_damping_param_ = std::make_unique<dynamic_reconfigure::Server<cartesian_impedance_controller::dampingConfig>>(ros::NodeHandle(std::string(nh.getNamespace() + "/damping_factors_reconfigure")));
    dynamic_server_damping_param_->setCallback(
        boost::bind(&CartesianImpedanceControllerRos::dynamicDampingCb, this, _1, _2));

    this->dynamic_server_wrench_param_ = std::make_unique<dynamic_reconfigure::Server<cartesian_impedance_controller::wrenchConfig>>(ros::NodeHandle(std::string(nh.getNamespace() + "/cartesian_wrench_reconfigure")));
    dynamic_server_wrench_param_->setCallback(
        boost::bind(&CartesianImpedanceControllerRos::dynamicWrenchCb, this, _1, _2));
    return true;
  }

  bool CartesianImpedanceControllerRos::initJointHandles(hardware_interface::EffortJointInterface *hw, const ros::NodeHandle &nh)
  {
    std::vector<std::string> joint_names;
    if (!nh.getParam("joints", joint_names))
    {
      ROS_ERROR("Invalid or no 'joints' parameter provided, aborting controller init!");
      return false;
    }
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      try
      {
        this->joint_handles_.push_back(hw->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM("Exception getting joint handles: " << ex.what());
        return false;
      }
    }
    ROS_INFO_STREAM("Number of joints specified in parameters: " << joint_names.size());
    this->setNumberOfJoints(joint_names.size());
    return true;
  }

  bool CartesianImpedanceControllerRos::initMessaging(ros::NodeHandle *nh)
  {
    // Queue size of 1 since we are only interested in the last message
    this->sub_cart_stiffness_ = nh->subscribe("set_cartesian_stiffness", 1,
                                              &CartesianImpedanceControllerRos::cartesianStiffnessCb, this);
    this->sub_cart_wrench_ = nh->subscribe("set_cartesian_wrench", 1,
                                           &CartesianImpedanceControllerRos::wrenchCommandCb, this);
    this->sub_damping_factors_ = nh->subscribe("set_damping_factors", 1,
                                       &CartesianImpedanceControllerRos::cartesianDampingFactorCb, this);
    this->sub_controller_config_ =
        nh->subscribe("set_config", 1, &CartesianImpedanceControllerRos::controllerConfigCb, this);
    this->sub_reference_pose_ = nh->subscribe("reference_pose", 1, &CartesianImpedanceControllerRos::referencePoseCb, this);

    // Initializing the realtime publisher and the message
    this->pub_torques_.init(*nh, "commanded_torques", 20);
    this->pub_torques_.msg_.layout.dim.resize(1);
    this->pub_torques_.msg_.layout.data_offset = 0;
    this->pub_torques_.msg_.layout.dim[0].size = this->n_joints_;
    this->pub_torques_.msg_.layout.dim[0].stride = 0;
    this->pub_torques_.msg_.data.resize(this->n_joints_);

    std::vector<std::string> joint_names;
    nh->getParam("joints", joint_names);
    this->pub_state_.init(*nh, "controller_state", 10);
    this->pub_state_.msg_.header.seq = 0;
    this->pub_state_.msg_.header.frame_id = this->root_frame_;
    for (size_t i = 0; i < this->n_joints_; i++)
    {
      this->pub_state_.msg_.joint_state.name.push_back(joint_names.at(i));
    }
    this->pub_state_.msg_.joint_state.position = std::vector<double>(this->n_joints_);
    this->pub_state_.msg_.joint_state.velocity = std::vector<double>(this->n_joints_);
    this->pub_state_.msg_.joint_state.effort = std::vector<double>(this->n_joints_);
    this->pub_state_.msg_.commanded_torques = std::vector<double>(this->n_joints_);
    this->pub_state_.msg_.nullspace_config = std::vector<double>(this->n_joints_);
    return true;
  }

  bool CartesianImpedanceControllerRos::initRBDyn(const ros::NodeHandle &nh)
  {
    // Get the URDF XML from the parameter server. Wait if needed.
    std::string urdf_string;
    nh.param<std::string>("robot_description", robot_description_, "/robot_description");
    while (!nh.getParam(robot_description_, urdf_string))
    {
      ROS_INFO_ONCE("Waiting for robot description in parameter %s on the ROS param server.",
                    robot_description_.c_str());
      usleep(100000);
    }
    try
    {
      this->rbdyn_wrapper_.init_rbdyn(urdf_string, end_effector_);
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR("Error when intializing RBDyn: %s", e.what());
      return false;
    }
    ROS_INFO_STREAM("Number of joints found in urdf: " << this->rbdyn_wrapper_.n_joints());
    if (this->rbdyn_wrapper_.n_joints() < this->n_joints_)
    {
      ROS_ERROR("Number of joints in the URDF is smaller than supplied number of joints. %i < %zu", this->rbdyn_wrapper_.n_joints(), this->n_joints_);
      return false;
    }
    else if (this->rbdyn_wrapper_.n_joints() > this->n_joints_)
    {
      ROS_WARN("Number of joints in the URDF is greater than supplied number of joints: %i > %zu. Assuming that the actuated joints come first.", this->rbdyn_wrapper_.n_joints(), this->n_joints_);
    }
    return true;
  }

  bool CartesianImpedanceControllerRos::initTrajectories(ros::NodeHandle *nh)
  {
    this->sub_trajectory_ = nh->subscribe("joint_trajectory", 1, &CartesianImpedanceControllerRos::trajCb, this);
    this->traj_as_ = std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>>(
        new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(
            *nh, std::string("follow_joint_trajectory"), false));
    this->traj_as_->registerGoalCallback(boost::bind(&CartesianImpedanceControllerRos::trajGoalCb, this));
    this->traj_as_->registerPreemptCallback(boost::bind(&CartesianImpedanceControllerRos::trajPreemptCb, this));
    this->traj_as_->start();
    return true;
  }

  bool CartesianImpedanceControllerRos::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle)
  {
    ROS_INFO("Initializing Cartesian impedance controller in namespace: %s", node_handle.getNamespace().c_str());

    // Fetch parameters
    node_handle.param<std::string>("end_effector", this->end_effector_, "iiwa_link_ee");
    ROS_INFO_STREAM("End effector link is: " << this->end_effector_);
    // Frame for applying commanded Cartesian wrenches
    node_handle.param<std::string>("wrench_ee_frame", this->wrench_ee_frame_, this->end_effector_);
    ROS_INFO_STREAM("Wrench end effector frame is: " << this->wrench_ee_frame_);
    bool dynamic_reconfigure{true};
    node_handle.param<bool>("dynamic_reconfigure", dynamic_reconfigure, true);
    bool enable_trajectories{true};
    node_handle.param<bool>("handle_trajectories", enable_trajectories, true);
    node_handle.param<double>("delta_tau_max", this->delta_tau_max_, 1.);
    node_handle.param<double>("update_frequency", this->update_frequency_, 500.);
    node_handle.param<double>("filtering/nullspace_config", this->filter_params_nullspace_config_, 0.1);
    node_handle.param<double>("filtering/stiffness", this->filter_params_stiffness_, 0.1);
    node_handle.param<double>("filtering/pose", this->filter_params_pose_, 0.1);
    node_handle.param<double>("filtering/wrench", this->filter_params_wrench_, 0.1);
    node_handle.param<bool>("verbosity/verbose_print", this->verbose_print_, false);
    node_handle.param<bool>("verbosity/state_msgs", this->verbose_state_, false);
    node_handle.param<bool>("verbosity/tf_frames", this->verbose_tf_, false);

    if (!this->initJointHandles(hw, node_handle) || !this->initMessaging(&node_handle) || !this->initRBDyn(node_handle))
    {
      return false;
    }
    if (enable_trajectories && !this->initTrajectories(&node_handle))
    {
      return false;
    }
    this->root_frame_ = this->rbdyn_wrapper_.root_link();
    node_handle.setParam("root_frame", this->root_frame_);
    // The reference frame the stiffness and damping refer to
    node_handle.param<std::string>("control_frame", this->control_frame_, this->rbdyn_wrapper_.root_link());
    this->rbdyn_wrapper_.set_control_frame(this->control_frame_);
    ROS_INFO_STREAM("Control frame is: " << this->control_frame_);

    // Initialize base_tools and member variables
    this->setNumberOfJoints(this->joint_handles_.size());
    if (this->n_joints_ < 6)
    {
      ROS_WARN("Number of joints is below 6. Functions might be limited.");
    }
    if (this->n_joints_ < 7)
    {
      ROS_WARN("Number of joints is below 7. No redundant joint for nullspace.");
    }
    this->tau_m_ = Eigen::VectorXd(this->n_joints_);

    // Needs to be after base_tools init since the wrench callback calls it
    if (dynamic_reconfigure && !this->initDynamicReconfigure(node_handle))
    {
      return false;
    }

    ROS_INFO("Finished initialization.");
    return true;
  }

  void CartesianImpedanceControllerRos::starting(const ros::Time & /*time*/)
  {
    this->updateState();

    // Set reference pose to current pose and q_d_nullspace
    this->initDesiredPose(this->position_, this->orientation_);
    this->initNullspaceConfig(this->q_);
    ROS_INFO("Started Cartesian Impedance Controller");
  }

  void CartesianImpedanceControllerRos::update(const ros::Time & /*time*/, const ros::Duration &period /*period*/)
  {
    if (this->traj_running_)
    {
      trajUpdate();
    }

    this->updateState();

    // Apply control law in base library
    this->calculateCommandedTorques();

    // Write commands
    for (size_t i = 0; i < this->n_joints_; ++i)
    {
      this->joint_handles_[i].setCommand(this->tau_c_(i));
    }

    publishMsgsAndTf();
  }

  bool CartesianImpedanceControllerRos::getFk(const Eigen::VectorXd &q, Eigen::Vector3d *position,
                                              Eigen::Quaterniond *orientation) const
  {
    rbdyn_wrapper::EefState ee_state;
    // If the URDF contains more joints than there are controlled, only the state of the controlled ones are known
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
    *position = ee_state.translation;
    *orientation = ee_state.orientation;
    return true;
  }

  bool CartesianImpedanceControllerRos::getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                                    Eigen::MatrixXd *jacobian)
  {
    // If the URDF contains more joints than there are controlled, only the state of the controlled ones are known
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

  void CartesianImpedanceControllerRos::updateState()
  {
    for (size_t i = 0; i < this->n_joints_; ++i)
    {
      this->q_[i] = this->joint_handles_[i].getPosition();
      this->dq_[i] = this->joint_handles_[i].getVelocity();
      this->tau_m_[i] = this->joint_handles_[i].getEffort();
    }
    getJacobian(this->q_, this->dq_, &this->jacobian_);
    getFk(this->q_, &this->position_, &this->orientation_);

    if (control_frame_.compare(root_frame_) != 0 ) {

      Eigen::Matrix3d R = this->rbdyn_wrapper_.get_R_control_frame();
      T_control_w_adj_ << R, Eigen::Matrix3d::Zero(), 
                          Eigen::Matrix3d::Zero(), R;

    }
  }

  void CartesianImpedanceControllerRos::controllerConfigCb(const cartesian_impedance_controller::ControllerConfigConstPtr &msg)
  {
    this->setStiffness(msg->cartesian_stiffness, msg->nullspace_stiffness, false);
    this->setDampingFactors(msg->cartesian_damping_factors, msg->nullspace_damping_factor);

    if (msg->q_d_nullspace.size() == this->n_joints_)
    {
      Eigen::VectorXd q_d_nullspace(this->n_joints_);
      for (size_t i = 0; i < this->n_joints_; i++)
      {
        q_d_nullspace(i) = msg->q_d_nullspace.at(i);
      }
      this->setNullspaceConfig(q_d_nullspace);
    }
    else
    {
      ROS_WARN_STREAM("Nullspace configuration does not have the correct amount of entries. Got " << msg->q_d_nullspace.size() << " expected " << this->n_joints_ << ". Ignoring.");
    }
  }

  void CartesianImpedanceControllerRos::cartesianDampingFactorCb(const geometry_msgs::WrenchConstPtr &msg)
  {
    this->setDampingFactors(*msg, this->damping_factors_[6]);
  }

  void CartesianImpedanceControllerRos::referencePoseCb(const geometry_msgs::PoseStampedConstPtr &msg)
  {

    Eigen::Vector3d position_d;
    position_d << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    const Eigen::Quaterniond last_orientation_d_target(this->orientation_d_);
    Eigen::Quaterniond orientation_d;
    orientation_d.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
        msg->pose.orientation.w;

    //if empty, consider as wrt root link
    if  (!msg->header.frame_id.empty() && msg->header.frame_id.compare(this->root_frame_) != 0)
    {
      if (!transformPose(&position_d, &orientation_d, msg->header.frame_id, this->root_frame_))
      {
        ROS_ERROR("Could not transform Pose. Not applying it.");
        return;
      }
    }
    
    if (last_orientation_d_target.coeffs().dot(this->orientation_d_.coeffs()) < 0.0)
    {
      this->orientation_d_.coeffs() << -this->orientation_d_.coeffs();
    }
    this->setReferencePose(position_d, orientation_d);
  }

  void CartesianImpedanceControllerRos::cartesianStiffnessCb(const geometry_msgs::WrenchStampedConstPtr &msg)
  {
    this->setStiffness(msg->wrench, this->nullspace_stiffness_target_);
  }

  void CartesianImpedanceControllerRos::setDampingFactors(const geometry_msgs::Wrench &cart_damping, double nullspace)
  {
    CartesianImpedanceController::setDampingFactors(saturateValue(cart_damping.force.x, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.force.y, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.force.z, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.x, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.y, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.z, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(nullspace, dmp_factor_min_, dmp_factor_max_));
  }

  void CartesianImpedanceControllerRos::setStiffness(const geometry_msgs::Wrench &cart_stiffness, double nullspace, bool auto_damping)
  {
    CartesianImpedanceController::setStiffness(saturateValue(cart_stiffness.force.x, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.force.y, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.force.z, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.torque.x, rot_stf_min_, rot_stf_max_),
                                               saturateValue(cart_stiffness.torque.y, rot_stf_min_, rot_stf_max_),
                                               saturateValue(cart_stiffness.torque.z, rot_stf_min_, rot_stf_max_),
                                               saturateValue(nullspace, ns_min_, ns_max_), auto_damping);
  }

  void CartesianImpedanceControllerRos::wrenchCommandCb(const geometry_msgs::WrenchStampedConstPtr &msg)
  {
    Eigen::Matrix<double, 6, 1> F;
    F << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
        msg->wrench.torque.z;

    if (!msg->header.frame_id.empty() && msg->header.frame_id != this->root_frame_)
    {
      if (!transformWrench(&F, msg->header.frame_id, this->root_frame_))
      {
        ROS_ERROR("Could not transform wrench. Not applying it.");
        return;
      }
    }
    else if (msg->header.frame_id.empty())
    {
      if (!transformWrench(&F, this->wrench_ee_frame_, this->root_frame_))
      {
        ROS_ERROR("Could not transform wrench. Not applying it.");
        return;
      }
    }
    this->applyWrench(F);
  }

  bool CartesianImpedanceControllerRos::transformWrench(Eigen::Matrix<double, 6, 1> *cartesian_wrench,
                                                        const std::string &from_frame, const std::string &to_frame) const
  {
    try
    {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(to_frame, from_frame, ros::Time(0), transform);
      tf::Vector3 v_f(cartesian_wrench->operator()(0), cartesian_wrench->operator()(1), cartesian_wrench->operator()(2));
      tf::Vector3 v_t(cartesian_wrench->operator()(3), cartesian_wrench->operator()(4), cartesian_wrench->operator()(5));
      tf::Vector3 v_f_rot = tf::quatRotate(transform.getRotation(), v_f);
      tf::Vector3 v_t_rot = tf::quatRotate(transform.getRotation(), v_t);
      *cartesian_wrench << v_f_rot[0], v_f_rot[1], v_f_rot[2], v_t_rot[0], v_t_rot[1], v_t_rot[2];
      return true;
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR_THROTTLE(1, "%s", ex.what());
      return false;
    }
  }

  bool CartesianImpedanceControllerRos::transformPose(Eigen::Vector3d *pos, Eigen::Quaterniond *quat,
                                                        const std::string &from_frame, const std::string &to_frame) const
  {
    try
    {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(to_frame, from_frame, ros::Time(0), transform);
      tf::Vector3 v_pos(pos->operator()(0), pos->operator()(1), pos->operator()(2));
      tf::Vector3 v_pos_rot = tf::quatRotate(transform.getRotation(), v_pos);
      
      *pos << v_pos_rot[0]+transform.getOrigin().getX(), 
              v_pos_rot[1]+transform.getOrigin().getY(), 
              v_pos_rot[2]+transform.getOrigin().getZ();

      Eigen::Quaterniond eig_transform_quat(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
      *quat = quat->inverse() * eig_transform_quat;
      quat->normalize();

      return true;
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR_THROTTLE(1, "%s", ex.what());
      return false;
    }
  }

  void CartesianImpedanceControllerRos::publishMsgsAndTf()
  {
    // publish commanded torques
    if (this->pub_torques_.trylock())
    {
      for (size_t i = 0; i < this->n_joints_; i++)
      {
        this->pub_torques_.msg_.data[i] = this->tau_c_[i];
      }
      this->pub_torques_.unlockAndPublish();
    }

    const Eigen::Matrix<double, 6, 1> error{this->getPoseError()};

    if (this->verbose_print_)
    {
      ROS_INFO_STREAM_THROTTLE(0.1, "\nCartesian Position:\n"
                                        << this->position_ << "\nError:\n"
                                        << error << "\nCartesian Stiffness:\n"
                                        << this->cartesian_stiffness_ << "\nCartesian damping:\n"
                                        << this->cartesian_damping_ << "\nNullspace stiffness:\n"
                                        << this->nullspace_stiffness_ << "\nq_d_nullspace:\n"
                                        << this->q_d_nullspace_ << "\ntau_d:\n"
                                        << this->tau_c_);
    }
    if (this->verbose_tf_ && ros::Time::now() > this->tf_last_time_)
    {
      // Publish result of forward kinematics
      tf::vectorEigenToTF(this->position_, this->tf_pos_);
      this->tf_br_transform_.setOrigin(this->tf_pos_);
      tf::quaternionEigenToTF(this->orientation_, this->tf_rot_);
      this->tf_br_transform_.setRotation(this->tf_rot_);
      tf_br_.sendTransform(tf::StampedTransform(this->tf_br_transform_, ros::Time::now(), this->root_frame_, this->end_effector_ + "_ee_fk"));
      // Publish tf to the reference pose
      tf::vectorEigenToTF(this->position_d_, this->tf_pos_);
      this->tf_br_transform_.setOrigin(this->tf_pos_);
      tf::quaternionEigenToTF(this->orientation_d_, this->tf_rot_);
      this->tf_br_transform_.setRotation(this->tf_rot_);
      tf_br_.sendTransform(tf::StampedTransform(this->tf_br_transform_, ros::Time::now(), this->root_frame_, this->end_effector_ + "_ee_ref_pose"));
      this->tf_last_time_ = ros::Time::now();
    }
    if (this->verbose_state_ && this->pub_state_.trylock())
    {
      this->pub_state_.msg_.header.stamp = ros::Time::now();
      tf::pointEigenToMsg(this->position_, this->pub_state_.msg_.current_pose.position);
      tf::quaternionEigenToMsg(this->orientation_, this->pub_state_.msg_.current_pose.orientation);
      tf::pointEigenToMsg(this->position_d_, this->pub_state_.msg_.reference_pose.position);
      tf::quaternionEigenToMsg(this->orientation_d_, this->pub_state_.msg_.reference_pose.orientation);
      tf::pointEigenToMsg(error.head(3), this->pub_state_.msg_.pose_error.position);
      Eigen::Quaterniond q = Eigen::AngleAxisd(error(3), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(error(4), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(error(5), Eigen::Vector3d::UnitZ());
      tf::quaternionEigenToMsg(q, this->pub_state_.msg_.pose_error.orientation);

      EigenVectorToWrench(this->cartesian_stiffness_.diagonal(), &this->pub_state_.msg_.cartesian_stiffness);
      EigenVectorToWrench(this->cartesian_damping_.diagonal(), &this->pub_state_.msg_.cartesian_damping);
      EigenVectorToWrench(this->getAppliedWrench(), &this->pub_state_.msg_.commanded_wrench);

      for (size_t i = 0; i < this->n_joints_; i++)
      {
        this->pub_state_.msg_.joint_state.position.at(i) = this->q_(i);
        this->pub_state_.msg_.joint_state.velocity.at(i) = this->dq_(i);
        this->pub_state_.msg_.joint_state.effort.at(i) = this->tau_m_(i);
        this->pub_state_.msg_.nullspace_config.at(i) = this->q_d_nullspace_(i);
        this->pub_state_.msg_.commanded_torques.at(i) = this->tau_c_(i);
      }
      this->pub_state_.msg_.nullspace_stiffness = this->nullspace_stiffness_;
      this->pub_state_.msg_.nullspace_damping = this->nullspace_damping_;
      const Eigen::Matrix<double, 6, 1> dx = this->jacobian_ * this->dq_;
      this->pub_state_.msg_.cartesian_velocity = sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));

      this->pub_state_.unlockAndPublish();
      this->pub_state_.msg_.header.seq++;
    }
  }

  // Dynamic reconfigure
  // --------------------------------------------------------------------------------------------------------------------------------------
  void CartesianImpedanceControllerRos::dynamicStiffnessCb(
      cartesian_impedance_controller::stiffnessConfig &config, uint32_t level)
  {
    if (config.update_stiffness)
    {
      CartesianImpedanceController::setStiffness(saturateValue(config.translation_x, trans_stf_min_, trans_stf_max_),
                                                 saturateValue(config.translation_y, trans_stf_min_, trans_stf_max_),
                                                 saturateValue(config.translation_z, trans_stf_min_, trans_stf_max_),
                                                 saturateValue(config.rotation_x, trans_stf_min_, trans_stf_max_),
                                                 saturateValue(config.rotation_y, trans_stf_min_, trans_stf_max_),
                                                 saturateValue(config.rotation_z, trans_stf_min_, trans_stf_max_), config.nullspace_stiffness);
    }
  }

  void CartesianImpedanceControllerRos::dynamicDampingCb(
      cartesian_impedance_controller::dampingConfig &config, uint32_t level)
  {
    if (config.update_damping_factors)
    {
      CartesianImpedanceController::setDampingFactors(
          config.translation_x, config.translation_y, config.translation_z, config.rotation_x, config.rotation_y, config.rotation_z, config.nullspace_damping);
    }
  }

  void CartesianImpedanceControllerRos::dynamicWrenchCb(cartesian_impedance_controller::wrenchConfig &config,
                                                        uint32_t level)
  {
    Eigen::Vector6d F{Eigen::Vector6d::Zero()};
    if (config.apply_wrench)
    {
      F << config.f_x, config.f_y, config.f_z, config.tau_x, config.tau_y, config.tau_z;
      if (this->wrench_ee_frame_.compare( this->root_frame_) != 0 && !transformWrench(&F, this->wrench_ee_frame_, this->root_frame_))
      {
        ROS_ERROR("Could not transform wrench. Not applying it.");
        return;
      }
    }
    this->applyWrench(F);
  }

  void CartesianImpedanceControllerRos::trajCb(const trajectory_msgs::JointTrajectoryConstPtr &msg)
  {
    ROS_INFO("Got trajectory msg from trajectory topic.");
    if (this->traj_as_->isActive())
    {
      this->traj_as_->setPreempted();
      ROS_INFO("Preempted running action server goal.");
    }
    trajStart(*msg);
  }

  void CartesianImpedanceControllerRos::trajGoalCb()
  {
    this->traj_as_goal_ = this->traj_as_->acceptNewGoal();
    ROS_INFO("Accepted new goal from action server.");
    trajStart(this->traj_as_goal_->trajectory);
  }

  void CartesianImpedanceControllerRos::trajPreemptCb()
  {
    ROS_INFO("Actionserver got preempted.");
    this->traj_as_->setPreempted();
  }

  void CartesianImpedanceControllerRos::trajStart(const trajectory_msgs::JointTrajectory &trajectory)
  {
    this->traj_duration_ = trajectory.points[trajectory.points.size() - 1].time_from_start;
    ROS_INFO_STREAM("Starting a new trajectory with " << trajectory.points.size() << " points that takes " << this->traj_duration_ << "s.");
    this->trajectory_ = trajectory;
    this->traj_running_ = true;
    this->traj_start_ = ros::Time::now();
    this->traj_index_ = 0;
    trajUpdate();
    if (this->nullspace_stiffness_ < 5.)
    {
      ROS_WARN("Nullspace stiffness is low. The joints might not follow the planned path.");
    }
  }

  void CartesianImpedanceControllerRos::trajUpdate()
  {
    if (ros::Time::now() > (this->traj_start_ + trajectory_.points.at(this->traj_index_).time_from_start))
    {
      // Get end effector pose
      Eigen::VectorXd q = Eigen::VectorXd::Map(trajectory_.points.at(this->traj_index_).positions.data(),
                                               trajectory_.points.at(this->traj_index_).positions.size());
      if (this->verbose_print_)
      {
        ROS_INFO_STREAM("Index " << this->traj_index_ << " q_nullspace: " << q.transpose());
      }
      // Update end-effector pose and nullspace
      getFk(q, &this->position_d_target_, &this->orientation_d_target_);
      this->setNullspaceConfig(q);
      this->traj_index_++;
    }

    if (ros::Time::now() > (this->traj_start_ + this->traj_duration_))
    {
      ROS_INFO_STREAM("Finished executing trajectory.");
      if (this->traj_as_->isActive())
      {
        this->traj_as_->setSucceeded();
      }
      this->traj_running_ = false;
    }
  }
} // namespace cartesian_impedance_controller
