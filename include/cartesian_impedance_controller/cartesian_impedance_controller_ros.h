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
#include <cartesian_impedance_controller/rbdyn_wrapper.h>

#include <cartesian_impedance_controller/ControllerConfig.h>
#include <cartesian_impedance_controller/ControllerState.h>
#include <cartesian_impedance_controller/dampingConfig.h>
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
    void setDampingFactors(const geometry_msgs::Wrench &cart_damping, double nullspace);

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
    * Calls setDampingFactors function.
    * @sa setDampingFactors.
    * \param[in] msg Received message
    */
    void cartesianDampingFactorCb(const geometry_msgs::WrenchConstPtr &msg);

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
    * @sa setDampingFactors, setStiffness
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
    * If the wrench is not given in end-effector frame, it will be transformed in the root frame. Once when a new wrench message arrives.
    * Sets the wrench using the base library.
    * @sa applyWrench.
    * \param[in] msg Received message
    */
    void wrenchCommandCb(const geometry_msgs::WrenchStampedConstPtr &msg);

    /*! \brief Transforms the wrench in a target frame.
    *
    * Takes a vector with the wrench and transforms it to a given coordinate frame. E.g. from_frame= "world" , to_frame = "bh_link_ee"

    * @sa wrenchCommandCb
    * \param[in] cartesian_wrench Vector with the Cartesian wrench
    * \param[in] from_frame       Source frame
    * \param[in] to_frame         Target frame
    * \return True on success, false on failure.
    */
    bool transformWrench(Eigen::Matrix<double, 6, 1> *cartesian_wrench, const std::string &from_frame, const std::string &to_frame) const;

    /*! \brief Transforms the pose (3D vector and quaternion) in a target frame.
    *
    * Takes a vector and a quaternion and transforms it to a given coordinate frame. 
    * E.g. from_frame= "world" , to_frame = "bh_link_ee"
    * 
    * @sa referencePoseCb
    * \param[in] pos Vector representing the linear position
    * \param[in] quat Quaternion representing the angular position
    * \param[in] from_frame       Source frame
    * \param[in] to_frame         Target frame
    * \return True on success, false on failure.
    */
    bool transformPose(Eigen::Vector3d *pos, Eigen::Quaterniond *quat,
                      const std::string &from_frame, const std::string &to_frame) const;

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

    std::vector<hardware_interface::JointHandle> joint_handles_; //!< Joint handles for states and commands
    rbdyn_wrapper rbdyn_wrapper_;   //!< Wrapper for RBDyn library for kinematics 
    std::string end_effector_;      //!< End-effector link name
    std::string robot_description_; //!< URDF of the robot
    std::string root_frame_;        //!< Base frame obtained from URDF
    std::string control_frame_;     //!< Frame wrt impedance is referred

    Eigen::VectorXd tau_m_;         //!< Measured joint torques

    ros::Subscriber sub_cart_stiffness_;    //!< Cartesian stiffness subscriber
    ros::Subscriber sub_cart_wrench_;       //!< Cartesian wrench subscriber
    ros::Subscriber sub_damping_factors_;           //!< Damping subscriber
    ros::Subscriber sub_controller_config_;  //!< Controller configuration subscriber
    ros::Subscriber sub_reference_pose_;    //!< Cartesian reference pose subscriber

    tf::TransformListener tf_listener_;     //!< tf transformation listener
    std::string wrench_ee_frame_;           //!< Frame for the application of the commanded wrench 

    // Hard limits. They are enforced on input.
    const double trans_stf_min_{0};     //!< Minimum translational stiffness
    const double trans_stf_max_{1500};  //!< Maximum translational stiffness
    const double rot_stf_min_{0};       //!< Minimum rotational stiffness
    const double rot_stf_max_{100};     //!< Maximum rotational stiffness
    const double ns_min_{0};            //!< Minimum nullspace stiffness
    const double ns_max_{100};          //!< Maximum nullspace stiffness
    const double dmp_factor_min_{0.001};       //!< Minimum damping factor
    const double dmp_factor_max_{2.0};         //!< Maximum damping factor

    // The Jacobian of RBDyn comes with orientation in the first three lines. Needs to be interchanged.
    const Eigen::VectorXi perm_indices_ =
      (Eigen::Matrix<int, 6, 1>() << 3, 4, 5, 0, 1, 2).finished(); //!< Permutation indices to switch position and orientation
    const Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_ =
      Eigen::PermutationMatrix<Eigen::Dynamic, 6>(perm_indices_); //!< Permutation matrix to switch position and orientation entries

    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<cartesian_impedance_controller::stiffnessConfig>>
        dynamic_server_compliance_param_; //!< Dybanic reconfigure server for stiffness
    std::unique_ptr<dynamic_reconfigure::Server<cartesian_impedance_controller::dampingConfig>>
        dynamic_server_damping_param_;    //!< Dynamic reconfigure server for damping
    std::unique_ptr<dynamic_reconfigure::Server<cartesian_impedance_controller::wrenchConfig>>
        dynamic_server_wrench_param_;     //!< Dynamic reconfigure server for commanded wrench

    // Trajectory handling
    ros::Subscriber sub_trajectory_;  //!< Subscriber for a single trajectory
    std::unique_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> traj_as_; //!< Trajectory action server
    boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> traj_as_goal_;  //!< Trajectory action server goal
    trajectory_msgs::JointTrajectory trajectory_; //!< Currently played trajectory
    ros::Time traj_start_;          //!< Time the current trajectory is started 
    ros::Duration traj_duration_;   //!< Duration of the current trajectory
    unsigned int traj_index_{0};    //!< Index of the current trajectory point
    bool traj_running_{false};      //!< True when running a trajectory 

    // Extra output
    bool verbose_print_{false};       //!< Verbose printing enabled
    bool verbose_state_{false};       //!< Verbose state messages enabled
    bool verbose_tf_{false};          //!< Verbose tf pubishing enabled
    tf::TransformBroadcaster tf_br_;  //!< tf transform broadcaster for verbose tf 
    realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> pub_torques_;  //!< Realtime publisher for commanded torques
    realtime_tools::RealtimePublisher<cartesian_impedance_controller::ControllerState> pub_state_;  //!< Realtime publisher for controller state
    tf::Transform tf_br_transform_;   //!< tf transform for publishing
    tf::Vector3 tf_pos_;              //!< tf position for publishing
    tf::Quaternion tf_rot_;           //!< tf orientation for publishing
    ros::Time tf_last_time_ = ros::Time::now(); //!< Last published tf message
  };

  // Declares this controller
  PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceControllerRos,
                         controller_interface::ControllerBase);

} // namespace cartesian_impedance_controller
