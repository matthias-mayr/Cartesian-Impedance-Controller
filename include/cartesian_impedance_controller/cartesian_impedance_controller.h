#pragma once

#include <Eigen/Dense>
#include <vector>

namespace cartesian_impedance_controller
{
  class CartesianImpedanceController
  {
  public:
    CartesianImpedanceController();
    ~CartesianImpedanceController() = default;

    /*! \brief Sets pose without using filtering
    *
    * \param[in] position_d_target Reference positions
    * \param[in] orientation_d_target Reference orientation
    */
    void initDesiredPose(const Eigen::Vector3d &position_d_target,
                         const Eigen::Quaterniond &orientation_d_target);

    /*! \brief Sets the nullspace configuration without using filtering
    *
    * \param[in] q_d_nullspace_target Nullspace joint positions
    */
    void initNullspaceConfig(const Eigen::VectorXd &q_d_nullspace_target);

    /*! \brief Sets the number of joints
    * 
    * \param[in] n_joints Number of joints
    */
    void setNumberOfJoints(size_t n_joints);

    /*! \brief Set the desired diagonal stiffnessess + nullspace stiffness
    * 
    * \param[in] stiffness Stiffnesses: position, orientation, nullspace
    * \param[in] auto_damping Apply automatic damping
    */
    void setStiffness(const Eigen::Matrix<double, 7, 1> &stiffness, bool auto_damping = true);

    /*! \brief Sets the Cartesian and nullspace stiffnesses
    * 
    * \param[in] t_x Translational stiffness x
    * \param[in] t_y Translational stiffness y
    * \param[in] t_z Translational stiffness z
    * \param[in] r_x Rotational stiffness x
    * \param[in] r_y Rotational stiffness y
    * \param[in] r_z Rotational stiffness z
    * \param[in] n   Nullspace stiffness
    * \param[in] auto_damping Apply automatic damping
    */
    void setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, double n, bool auto_damping = true);

    /*! \brief Sets the Cartesian and nullspace stiffnesses
    * 
    * \param[in] t_x Translational stiffness x
    * \param[in] t_y Translational stiffness y
    * \param[in] t_z Translational stiffness z
    * \param[in] r_x Rotational stiffness x
    * \param[in] r_y Rotational stiffness y
    * \param[in] r_z Rotational stiffness z
    * \param[in] auto_damping Apply automatic damping
    */
    void setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, bool auto_damping = true);

    /*! \brief Set the desired damping factors
    * 
    * \param[in] t_x Translational damping x
    * \param[in] t_y Translational damping y
    * \param[in] t_z Translational damping z
    * \param[in] r_x Rotational damping x
    * \param[in] r_y Rotational damping y
    * \param[in] r_z Rotational damping z
    * \param[in] n   Nullspace damping
    */
    void setDampingFactors(double d_x, double d_y, double d_z, double d_a, double d_b, double d_c, double d_n);

    /*! \brief Sets the desired end-effector pose
    *
    * Sets them as a new target, so filtering can be applied on them.
    * \param[in] position_d New reference position
    * \param[in] orientation_d New reference orientation
    */
    void setReferencePose(const Eigen::Vector3d &position_d, const Eigen::Quaterniond &orientation_d);

    /*! \brief Sets a new nullspace joint configuration
    *
    * Sets them as a new target, so filtering can be applied on them.
    * \param[in] q_d_nullspace_target New joint configuration
    */
    void setNullspaceConfig(const Eigen::VectorXd &q_d_nullspace_target);

    /*! \brief Sets filtering on stiffness + end-effector pose.
    *
    * Default inactive && depends on update_frequency
    * \param[in] update_frequency The expected controller update frequency
    * \param[in] filter_params_nullspace_config Filter setting for nullspace config
    * \param[in] filter_params_nullspace_config Filter setting for the stiffness
    * \param[in] filter_params_nullspace_config Filter setting for the pose
    * \param[in] filter_params_nullspace_config Filter setting for the commanded wrenc
    */
    void setFiltering(double update_frequency, double filter_params_nullspace_config, double filter_params_stiffness, double filter_params_pose,
                      double filter_params_wrench);

    /*! \brief Maximum commanded torque change per time step
    * 
    * Prevents too large changes in the commanded torques by using saturation.
    * \param[in] d Torque change per timestep
    */
    void setMaxTorqueDelta(double d);

    /*! \brief Sets maximum commanded torque change per time step and the update frequency
    * 
    * Prevents too large changes in the commanded torques by using saturation.
    * \param[in] d Torque change per timestep
    * \param[in] update_frequency Update frequency
    */
    void setMaxTorqueDelta(double d, double update_frequency);

    /*! \brief Apply a virtual Cartesian wrench in the root frame (often "world")
    * 
    * Prevents too large changes in the commanded torques by using saturation.
    * \param[in] cartesian_wrench Wrench to apply
    */
    void applyWrench(const Eigen::Matrix<double, 6, 1> &cartesian_wrench);

    /*! \brief Returns the commanded torques. Performs a filtering step.
    * 
    * This function assumes that the internal states have already been updates. The it utilizes the control rules to calculate commands.
    * \return Eigen Vector of the commanded torques
    */
    Eigen::VectorXd calculateCommandedTorques();

    /*! \brief Returns the commanded torques. Performs a filtering step and updates internal state.
    * 
    * This function utilizes the control rules.
    * \param[in] q Joint positions
    * \param[in] dq Joint velocities
    * \param[in] position End-effector position
    * \param[in] orientation End-effector orientation
    * \param[in] jacobian Jacobian
    */
    Eigen::VectorXd calculateCommandedTorques(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                              const Eigen::Vector3d &position, Eigen::Quaterniond orientation,
                                              const Eigen::MatrixXd &jacobian, const Eigen::Matrix3d &R_control_root = Eigen::Matrix3d::Identity());

    /*! \brief Get the state of the controller. Updates when "calculateCommandedTorques" is called
    * 
    * \param[out] q Joint positions
    * \param[out] dq Joint velocities
    * \param[out] position End-effector position
    * \param[out] orientation End-effector orientation
    * \param[out] position_d End-effector reference position
    * \param[out] orientation_d End-effector reference orientation
    * \param[out] cartesian_stiffness Cartesian stiffness
    * \param[out] nullspace_stiffness Nullspace stiffness
    * \param[out] q_d_nullspace Nullspace reference position
    * \param[out] cartesian_damping Cartesian damping
    */
    void getState(Eigen::VectorXd *q, Eigen::VectorXd *dq, Eigen::Vector3d *position, Eigen::Quaterniond *orientation,
                  Eigen::Vector3d *position_d, Eigen::Quaterniond *orientation_d,
                  Eigen::Matrix<double, 6, 6> *cartesian_stiffness, double *nullspace_stiffness,
                  Eigen::VectorXd *q_d_nullspace, Eigen::Matrix<double, 6, 6> *cartesian_damping) const;

    /*! \brief Get the state of the controller. Updates when "calculateCommandedTorques" is called
    * 
    * \param[out] position_d End-effector reference position
    * \param[out] orientation_d End-effector reference orientation
    * \param[out] cartesian_stiffness Cartesian stiffness
    * \param[out] nullspace_stiffness Nullspace stiffness
    * \param[out] q_d_nullspace Nullspace reference position
    * \param[out] cartesian_damping Cartesian damping
    */
    void getState(Eigen::Vector3d *position_d, Eigen::Quaterniond *orientation_d,
                  Eigen::Matrix<double, 6, 6> *cartesian_stiffness, double *nullspace_stiffness,
                  Eigen::VectorXd *q_d_nullspace, Eigen::Matrix<double, 6, 6> *cartesian_damping) const;

    /*! \brief Get the currently applied commands
    * 
    * \return Eigen Vector with commands
    */
    Eigen::VectorXd getLastCommands() const;

    /*! \brief Get the currently applied Cartesian wrench
    * 
    * \return Eigen Vector with the applied wrench
    */
    Eigen::Matrix<double, 6, 1> getAppliedWrench() const;

    /*! \brief Get the current pose error
    * 
    * \return Eigen Vector with the pose error for translation and rotation
    */
    Eigen::Matrix<double, 6, 1> getPoseError() const;

  protected:
    size_t n_joints_{7}; //!< Number of joints to control

    Eigen::Matrix<double, 6, 6> cartesian_stiffness_{Eigen::Matrix<double, 6, 6>::Identity()};  //!< Cartesian stiffness matrix
    Eigen::Matrix<double, 6, 6> cartesian_damping_{Eigen::Matrix<double, 6, 6>::Identity()};    //!< Cartesian damping matrix

    Eigen::VectorXd q_d_nullspace_;           //!< Current nullspace reference pose
    Eigen::VectorXd q_d_nullspace_target_;    //!< Nullspace reference target pose
    double nullspace_stiffness_{0.0};         //!< Current nullspace stiffness
    double nullspace_stiffness_target_{0.0};  //!< Nullspace stiffness target
    double nullspace_damping_{0.0};           //!< Current nullspace damping
    double nullspace_damping_target_{0.0};    //!< Nullspace damping target

    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_{Eigen::Matrix<double, 6, 6>::Identity()}; //!< Cartesian stiffness target
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_{Eigen::Matrix<double, 6, 6>::Identity()};   //!< Cartesian damping target
    Eigen::Matrix<double, 7, 1> damping_factors_{Eigen::Matrix<double, 7, 1>::Ones()};                //!< Damping factors

    Eigen::VectorXd q_;   //!< Joint positions
    Eigen::VectorXd dq_;  //!< Joint velocities

    Eigen::MatrixXd jacobian_; //!< Jacobian. Row format: 3 translations, 3 rotation

    Eigen::Matrix<double, 6,6> T_control_w_adj_{Eigen::Matrix<double, 6,6>::Identity()};

    // End Effector
    Eigen::Matrix<double, 6, 1> error_; //!< Calculate pose error
    Eigen::Vector3d position_{Eigen::Vector3d::Zero()};           //!< Current end-effector position
    Eigen::Vector3d position_d_{Eigen::Vector3d::Zero()};         //!< Current end-effector reference position
    Eigen::Vector3d position_d_target_{Eigen::Vector3d::Zero()};  //!< End-effector target position

    Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};          //!< Current end-effector orientation
    Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};        //!< Current end-effector target orientation
    Eigen::Quaterniond orientation_d_target_{Eigen::Quaterniond::Identity()}; //!< End-effector orientation target

    //  External applied forces
    Eigen::Matrix<double, 6, 1> cartesian_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};         //!< Current Cartesian wrench
    Eigen::Matrix<double, 6, 1> cartesian_wrench_target_{Eigen::Matrix<double, 6, 1>::Zero()};  //!< Cartesian wrench target

    Eigen::VectorXd tau_c_; //!< Last commanded torques

    double update_frequency_{1000};               //!< Update frequency in Hz
    double filter_params_nullspace_config_{1.0};  //!< Nullspace filtering
    double filter_params_stiffness_{1.0};         //!< Cartesian stiffness filtering
    double filter_params_pose_{1.0};              //!< Reference pose filtering
    double filter_params_wrench_{1.0};            //!< Commanded wrench filtering

    double delta_tau_max_{1.0};                   //!< Maximum allowed torque change per time step

  private:
    /*! \brief Implements the damping based on a stiffness
    *
    * Damping rule is 2*sqrt(stiffness)
    * \param[in] stiffness Stiffness value
    * \return Damping value
    */
    double dampingRule(double stiffness) const;

    /*! \brief Applies the damping rule with all stiffness values
    */
    void applyDamping();

    /*! Sets the update frequency
    *
    * \param[in] freq Update frequency
    */
    void setUpdateFrequency(double freq);

    /*! \brief Sets the filter value and asserts bounds
    *
    * \param[in] val New value
    * \param[out] saved_val Pointer to the value to be set
    */
    void setFilterValue(double val, double *saved_val);

    /*! \brief Adds a percental filtering effect to the nullspace configuration
    *
    * Gradually moves the nullspace configuration to the target configuration.
    */
    void updateFilteredNullspaceConfig();

    /*! \brief Adds a percental filtering effect to stiffness
    */
    void updateFilteredStiffness();

    /*! \brief Adds a percental filtering effect to the end-effector pose
    */
    void updateFilteredPose();

    /*! \brief Adds a percental filtering effect to the applied Cartesian wrench
    */
    void updateFilteredWrench();
  };

} // namespace cartesian_impedance_controller
