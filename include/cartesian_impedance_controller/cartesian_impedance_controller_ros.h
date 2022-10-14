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
  /*! \brief The ROS control implementation of the Cartesian impedance controller
  * 
  * It utilizes a list of joint names and the URDF description to control these joints.
  */
  class CartesianImpedanceControllerRos
      : public controller_interface::Controller<hardware_interface::EffortJointInterface>, public CartesianImpedanceController
  {

  public:
    /*! \brief Initializes the controller
    *
    * - Reads ROS parameters
    * - Initializes
    *   - joint handles
    *   - ROS messaging
    *   - RBDyn
    *   - rqt_reconfigure
    *   - Trajectory handling
    * \param[in] hw           Hardware interface
    * \param[in] node_handle  Node Handle
    * \return             True on success, false on failure
    */
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &node_handle) override;
    
    /*! \brief Starts the controller
    *
    * Updates the states and sets the desired pose and nullspace configuration to the current state.
    * \param[in] time Not used
    */
    void starting(const ros::Time &) override;

    /*! \brief Periodically called update function
    *
    * Updates the state and the trajectory. Calculated new commands and sets them.
    * Finally publishes ROS messages and tf transformations.
    * \param[in] time   Not used
    * \param[in] period Control period
    */
    void update(const ros::Time &, const ros::Duration &period) override;

  private:
    /*! \brief Initializes dynamic reconfigure
    * 
    * Initiliazes dynamic reconfigure for stiffness, damping and wrench.
    * \param[in] nh Nodehandle
    * \return True on success, false on failure. 
    */
    bool initDynamicReconfigure(const ros::NodeHandle &nh);

    /*! \brief Initializes the joint handles
    *
    * Fetches the joint names from the parameter server and initializes the joint handles.
    * \param[in] hw Hardware interface to obtain handles
    * \param[in] nh Nodehandle
    * \return True on success, false on failure.
    */
    bool initJointHandles(hardware_interface::EffortJointInterface *hw, const ros::NodeHandle &nh);

    /*! \brief Initializes messaging
    *
    * Initializes realtime publishers and the subscribers.
    * \param[in] nh Nodehandle
    * \return True on success, false on failure.
    */
    bool initMessaging(ros::NodeHandle *nh);

    /*! \brief Initializes RBDyn
    *
    * Reads the robot URDF and initializes RBDyn.
    * \param[in] nh Nodehandle
    * \return True on success, false on failure.
    */
    bool initRBDyn(const ros::NodeHandle &nh);

    /*! \brief Initializes trajectory handling
    *
    * Subscribes to joint trajectory topic and starts the trajectory action server.
    * \param[in] nh Nodehandle
    * \return Always true.
    */
    bool initTrajectories(ros::NodeHandle *nh);

    /*! \brief Get forward kinematics solution.
    *
    * Calls RBDyn to get the forward kinematics solution.
    * \param[in]  q            Joint position vector
    * \param[out] position     End-effector position
    * \param[out] orientation  End-effector orientation
    * \return Always true.
    */
    bool getFk(const Eigen::VectorXd &q, Eigen::Vector3d *position, Eigen::Quaterniond *rotation) const;

    /*! \brief Get Jacobian from RBDyn
    *
    * Gets the Jacobian for given joint positions and joint velocities.
    * \param[in]  q         Joint position vector        
    * \param[in]  dq        Joint velocity vector
    * \param[out] jacobian  Calculated Jacobian
    * \return True on success, false on failure.
    */
    bool getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, Eigen::MatrixXd *jacobian);

    /*! \brief Updates the state based on the joint handles.
    *
    * Gets latest joint positions, velocities and efforts and updates the forward kinematics as well as the Jacobian. 
    */
    void updateState();

    /*! \brief Sets damping for Cartesian space and nullspace.
    *
    * Long
    * \param[in] cart_damping   Cartesian damping [0,1]
    * \param[in] nullspace      Nullspace damping [0,1]
    */
    void setDamping(const geometry_msgs::Wrench &cart_damping, double nullspace);

    /*! \brief Sets Cartesian and nullspace stiffness
    *
    * Sets Cartesian and nullspace stiffness. Allows to set if automatic damping should be applied.
    * \param[in] cart_stiffness Cartesian stiffness
    * \param[in] nullspace      Nullspace stiffness
    * \param[in] auto_damping   Apply automatic damping 
    */
    void setStiffness(const geometry_msgs::Wrench &cart_stiffness, double nullspace, bool auto_damping = true);

    /*! \brief Message callback for Cartesian damping.
    *
    * Calls setDamping function.
    * @sa setDamping.
    * \param[in] msg Received message
    */
    void cartesianDampingCb(const geometry_msgs::WrenchConstPtr &msg);

    /*! \brief Message callback for Cartesian stiffness.
    *
    * Calls setStiffness function.
    * @sa setStiffness
    * \param[in] msg Received message
    */
    void cartesianStiffnessCb(const geometry_msgs::WrenchStampedConstPtr &msg);

    /*! \brief Message callback for the whole controller configuration.
    *
    * Sets stiffness, damping and nullspace.
    * @sa setDamping, setStiffness
    * \param[in] msg Received message
    */
    void controllerConfigCb(const cartesian_impedance_controller::ControllerConfigConstPtr &msg);

    /*! \brief Message callback for a Cartesian reference pose.
    *
    * Accepts new reference poses in the root frame - ignores them otherwise.
    * Sets the reference target pose.
    * @sa setReferencePose.
    * \param[in] msg Received message
    */
    void referencePoseCb(const geometry_msgs::PoseStampedConstPtr &msg);

    /*! \brief Message callback for Cartesian wrench messages.
    *
    * If the wrench is not given in end-effector frame, it will be transformed in the root frame.
    * Sets the wrench.
    * @sa applyWrench.
    * \param[in] msg Received message
    */
    void wrenchCommandCb(const geometry_msgs::WrenchStampedConstPtr &msg);

    /*! \brief Transforms the wrench in a target frame.
    *
    * Takes a vector with the wrench and transforms it to a given coordinate frame.
    * @sa wrenchCommandCb
    * \param[in] cartesian_wrench Vector with the Cartesian wrench
    * \param[in] from_frame       Source frame
    * \param[in] to_frame         Target frame
    * \return True on success, false on failure.
    */
    bool transformWrench(Eigen::Matrix<double, 6, 1> *cartesian_wrench, const std::string &from_frame, const std::string &to_frame) const;

    /*! \brief Verbose printing; publishes ROS messages and tf frames.
     *
     * Always publishes commanded torques.
     * Optional: request publishes tf frames for end-effector forward kinematics and the reference pose.
     * Optional: verbose printing
     * Optional: publishes state messages
     */
    void publishMsgsAndTf();

    /*! \brief Callback for stiffness dynamic reconfigure.
    *
    * Takes the dynamic reconfigure stiffness configuration, applies the limits and sets it.
    * \param[in] config 
    */
    void dynamicStiffnessCb(cartesian_impedance_controller::stiffnessConfig &config, uint32_t level);
    
    /*! \brief Callback for damping dynamic reconfigure.
    *
    * Takes the dynamic reconfigure configuration, applies limits and sets it.
    * \param[in] config 
    */
    void dynamicDampingCb(cartesian_impedance_controller::dampingConfig &config, uint32_t level);
    
    /*! \brief Callback for wrench dynamic reconfigure.
    *
    * Takes the dynamic reconfigure configuration, applies limits and sets it.
    * \param[in] config 
    */
    void dynamicWrenchCb(cartesian_impedance_controller::wrenchConfig &config, uint32_t level);

    /*! \brief Callback for a joint trajectory message.
    *
    * Preempts the action server if that one has a running goal.
    * \param[in] msg  Joint Trajectory Message
    */
    void trajCb(const trajectory_msgs::JointTrajectoryConstPtr &msg);

    /*! \brief Callback for a trajectory action goal.
    *
    * Accepts the new goal and starts the trajectory.
    */
    void trajGoalCb();

    /*! \brief Preempt function of the action server.
    *
    * Sets the goal as preempted.
    */
    void trajPreemptCb();

    /*! \brief Starts the trajectory.
    *
    * Resets the trajectory member variables. 
    */
    void trajStart(const trajectory_msgs::JointTrajectory &trajectory);

    /*! \brief Updates the trajectory.
    *
    * Called periodically from the update function if a trajectory is running.
    * A trajectory is run by going through it point by point, calculating forward kinematics and applying
    * the joint configuration to the nullspace control.
    */
    void trajUpdate();

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
