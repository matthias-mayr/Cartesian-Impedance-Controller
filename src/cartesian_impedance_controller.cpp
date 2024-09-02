#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include "pseudo_inversion.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace cartesian_impedance_controller
{
  /*! \brief Calculates the orientation error between two quaternions
   *
   * \param[in] orientation_d Reference orientation
   * \param[in] orientation Current orientation
   * \return Eigen Vector with the error
   */
  Eigen::Vector3d calculateOrientationError(const Eigen::Quaterniond &orientation_d, Eigen::Quaterniond orientation)
  {
    // Orientation error
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
    {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    const Eigen::Quaterniond error_quaternion(orientation * orientation_d.inverse());
    // convert to axis angle
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    return error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
  }

  /*! \brief Calculates a filtered percental update
   *
   * \param[in] target Target value
   * \param[in] current Current value
   * \param[in] filter Percentage of the target value
   * \return Calculated value
   */
  template <typename T>
  inline T filteredUpdate(T target, T current, double filter)
  {
    return (1.0 - filter) * current + filter * target;
  }

  /*! \brief Calculates the filter step
   *
   * \param[in] update_frequency   Update frequency in Hz
   * \param[in] filter_percentage  Filter percentage
   * \return Filter step
   */
  inline double filterStep(const double &update_frequency, const double &filter_percentage)
  {
    const double kappa = -1 / (std::log(1 - std::min(filter_percentage, 0.999999)));
    return 1.0 / (kappa * update_frequency + 1.0);
  }

  /*! \brief Saturate a variable x with the limits x_min and x_max
   *
   * \param[in] x Value
   * \param[in] x_min Minimal value
   * \param[in] x_max Maximum value
   * \return Saturated value
   */
  inline double saturateValue(double x, double x_min, double x_max)
  {
    return std::min(std::max(x, x_min), x_max);
  }

  /*! Saturate the torque rate to not stress the motors
   *
   * \param[in] tau_d_calculated Calculated input torques
   * \param[out] tau_d_saturated Saturated torque values
   * \param[in] delta_tau_max
   */
  inline void saturateTorqueRate(const Eigen::VectorXd &tau_d_calculated, Eigen::VectorXd *tau_d_saturated, double delta_tau_max)
  {
    for (size_t i = 0; i < tau_d_calculated.size(); i++)
    {
      const double difference = tau_d_calculated[i] - tau_d_saturated->operator()(i);
      tau_d_saturated->operator()(i) += saturateValue(difference, -delta_tau_max, delta_tau_max);
    }
  }

  CartesianImpedanceController::CartesianImpedanceController()
  {
    // Default stiffness values
    this->setStiffness(200., 200., 200., 20., 20., 20., 0.);
    this->cartesian_stiffness_ = this->cartesian_stiffness_target_;
    this->cartesian_damping_ = this->cartesian_damping_target_;
  }

  void CartesianImpedanceController::initDesiredPose(const Eigen::Vector3d &position_d_target,
                                                     const Eigen::Quaterniond &orientation_d_target)
  {
    this->setReferencePose(position_d_target, orientation_d_target);
    this->position_d_ = position_d_target;
    this->orientation_d_ = orientation_d_target;
  }

  void CartesianImpedanceController::initNullspaceConfig(const Eigen::VectorXd &q_d_nullspace_target)
  {
    this->setNullspaceConfig(q_d_nullspace_target);
    this->q_d_nullspace_ = this->q_d_nullspace_target_;
  }

  void CartesianImpedanceController::setNumberOfJoints(size_t n_joints)
  {
    if (n_joints < 0)
    {
      throw std::invalid_argument("Number of joints must be positive");
    }
    this->n_joints_ = n_joints;
    this->q_ = Eigen::VectorXd::Zero(this->n_joints_);
    this->dq_ = Eigen::VectorXd::Zero(this->n_joints_);
    this->jacobian_ = Eigen::MatrixXd::Zero(6, this->n_joints_);
    this->q_d_nullspace_ = Eigen::VectorXd::Zero(this->n_joints_);
    this->q_d_nullspace_target_ = this->q_d_nullspace_;
    this->tau_c_ = Eigen::VectorXd::Zero(this->n_joints_);
  }

  void CartesianImpedanceController::setStiffness(const Eigen::Matrix<double, 7, 1> &stiffness, bool auto_damping)
  {
    for (int i = 0; i < 6; i++)
    {
      // Set diagonal values of stiffness matrix
      if (stiffness(i) < 0.0)
      {
        assert(stiffness(i) >= 0 && "Stiffness values need to be positive.");
        this->cartesian_stiffness_target_(i, i) = 0.0;
      }
      else
      {
        this->cartesian_stiffness_target_(i, i) = stiffness(i);
      }
    }
    if (stiffness(6) < 0.0) {
      assert(stiffness(6) >= 0.0 && "Stiffness values need to be positive.");
      this->nullspace_stiffness_target_ = 0.0;
    }
    else
    {
      this->nullspace_stiffness_target_ = stiffness(6);
    }
    if (auto_damping)
    {
      this->applyDamping();
    }
  }

  void CartesianImpedanceController::setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z,
                                                  double n, bool auto_damping)
  {
    Eigen::Matrix<double, 7, 1> stiffness_vector(7);
    stiffness_vector << t_x, t_y, t_z, r_x, r_y, r_z, n;
    this->setStiffness(stiffness_vector, auto_damping);
  }

  void CartesianImpedanceController::setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, bool auto_damping)
  {
    Eigen::Matrix<double, 7, 1> stiffness_vector(7);
    stiffness_vector << t_x, t_y, t_z, r_x, r_y, r_z, this->nullspace_stiffness_target_;
    this->setStiffness(stiffness_vector, auto_damping);
  }

  void CartesianImpedanceController::setDampingFactors(double d_x, double d_y, double d_z, double d_a, double d_b, double d_c,
                                                double d_n)
  {
    Eigen::Matrix<double, 7, 1> damping_new;
    damping_new << d_x, d_y, d_z, d_a, d_b, d_c, d_n;
    for (size_t i = 0; i < damping_new.size(); i++)
    {
      if (damping_new(i) < 0)
      {
        assert(damping_new(i) >= 0 && "Damping factor must not be negative.");
        damping_new(i) = this->damping_factors_(i);
      }     
    }
    this->damping_factors_ = damping_new;
    this->applyDamping();
  }

  void CartesianImpedanceController::applyDamping()
  {
    for (int i = 0; i < 6; i++)
    {
      assert(this->damping_factors_(i) >= 0.0 && "Damping values need to be positive.");
      this->cartesian_damping_target_(i, i) =
          this->damping_factors_(i) * this->dampingRule(this->cartesian_stiffness_target_(i, i));
    }
    assert(this->damping_factors_(6) >= 0.0 && "Damping values need to be positive.");
    this->nullspace_damping_target_ = this->damping_factors_(6) * this->dampingRule(this->nullspace_stiffness_target_);
  }

  void CartesianImpedanceController::setReferencePose(const Eigen::Vector3d &position_d_target,
                                                      const Eigen::Quaterniond &orientation_d_target)
  {
    this->position_d_target_ << position_d_target;
    this->orientation_d_target_.coeffs() << orientation_d_target.coeffs();
    this->orientation_d_target_.normalize();
  }

  void CartesianImpedanceController::setNullspaceConfig(const Eigen::VectorXd &q_d_nullspace_target)
  {
    assert(q_d_nullspace_target.size() == this->n_joints_ && "Nullspace target needs to same size as n_joints_");
    this->q_d_nullspace_target_ << q_d_nullspace_target;
  }

  void CartesianImpedanceController::setFiltering(double update_frequency, double filter_params_nullspace_config, double filter_params_stiffness,
                                                  double filter_params_pose, double filter_params_wrench)
  {
    this->setUpdateFrequency(update_frequency);
    this->setFilterValue(filter_params_nullspace_config, &this->filter_params_nullspace_config_);
    this->setFilterValue(filter_params_stiffness, &this->filter_params_stiffness_);
    this->setFilterValue(filter_params_pose, &this->filter_params_pose_);
    this->setFilterValue(filter_params_wrench, &this->filter_params_wrench_);
  }

  void CartesianImpedanceController::setMaxTorqueDelta(double d)
  {
    assert(d >= 0.0 && "Allowed torque change must be positive");
    this->delta_tau_max_ = d;
  }

  void CartesianImpedanceController::setMaxTorqueDelta(double d, double update_frequency)
  {
    this->setMaxTorqueDelta(d);
    this->setUpdateFrequency(update_frequency);
  }

  void CartesianImpedanceController::applyWrench(const Eigen::Matrix<double, 6, 1> &cartesian_wrench_target)
  {
    this->cartesian_wrench_target_ = cartesian_wrench_target;
  }

  Eigen::VectorXd CartesianImpedanceController::calculateCommandedTorques(const Eigen::VectorXd &q,
                                                                          const Eigen::VectorXd &dq,
                                                                          const Eigen::Vector3d &position,
                                                                          Eigen::Quaterniond orientation,
                                                                          const Eigen::MatrixXd &jacobian,
                                                                          const Eigen::Matrix3d &R_control_root)
  {
    // Update controller to the current robot state
    this->q_ = q;
    this->dq_ = dq;
    this->position_ << position;
    this->orientation_.coeffs() << orientation.coeffs();
    this->jacobian_ << jacobian;
    T_control_w_adj_ << R_control_root, Eigen::Matrix3d::Zero(), 
                        Eigen::Matrix3d::Zero(), R_control_root;

    return this->calculateCommandedTorques();
  }

  Eigen::VectorXd CartesianImpedanceController::calculateCommandedTorques()
  {
    // Perform a filtering step
    updateFilteredNullspaceConfig();
    updateFilteredStiffness();
    updateFilteredPose();
    updateFilteredWrench();

    // Compute error term
    this->error_.head(3) << this->position_ - this->position_d_;
    this->error_.tail(3) << calculateOrientationError(this->orientation_d_, this->orientation_);

    // Kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(this->jacobian_.transpose(), &jacobian_transpose_pinv);

    Eigen::VectorXd tau_task(this->n_joints_), tau_nullspace(this->n_joints_), tau_ext(this->n_joints_);

    // Torque calculated for Cartesian impedance control with respect to a Cartesian pose reference in the end, in the frame of the EE of the robot.
    tau_task << this->jacobian_.transpose() * (T_control_w_adj_ * -this->cartesian_stiffness_ * T_control_w_adj_.transpose() * this->error_ 
                                              - T_control_w_adj_ * this->cartesian_damping_ * T_control_w_adj_.transpose() * (this->jacobian_ * this->dq_));
    // Torque for joint impedance control with respect to a desired configuration and projected in the null-space of the robot's Jacobian, so it should not affect the Cartesian motion of the robot's end-effector.
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - this->jacobian_.transpose() * jacobian_transpose_pinv) *
                         (this->nullspace_stiffness_ * (this->q_d_nullspace_ - this->q_) - this->nullspace_damping_ * this->dq_);
    // Torque to achieve the desired external force command in the frame of the EE of the robot.
    tau_ext = this->jacobian_.transpose() * this->cartesian_wrench_;

    // Torque commanded to the joints of the robot is composed by the superposition of these three joint-torque signals:
    Eigen::VectorXd tau_d = tau_task + tau_nullspace + tau_ext;
    saturateTorqueRate(tau_d, &this->tau_c_, this->delta_tau_max_);
    return this->tau_c_;
  }

  // Get the state of the robot.Updates when "calculateCommandedTorques" is called
  void CartesianImpedanceController::getState(Eigen::VectorXd *q, Eigen::VectorXd *dq, Eigen::Vector3d *position,
                                              Eigen::Quaterniond *orientation, Eigen::Vector3d *position_d,
                                              Eigen::Quaterniond *orientation_d,
                                              Eigen::Matrix<double, 6, 6> *cartesian_stiffness,
                                              double *nullspace_stiffness, Eigen::VectorXd *q_d_nullspace,
                                              Eigen::Matrix<double, 6, 6> *cartesian_damping) const
  {
    *q << this->q_;
    *dq << this->dq_;
    *position << this->position_;
    orientation->coeffs() << this->orientation_.coeffs();
    this->getState(position_d, orientation_d, cartesian_stiffness, nullspace_stiffness, q_d_nullspace, cartesian_damping);
  }

  void CartesianImpedanceController::getState(Eigen::Vector3d *position_d, Eigen::Quaterniond *orientation_d,
                                              Eigen::Matrix<double, 6, 6> *cartesian_stiffness,
                                              double *nullspace_stiffness, Eigen::VectorXd *q_d_nullspace,
                                              Eigen::Matrix<double, 6, 6> *cartesian_damping) const
  {
    *position_d = this->position_d_;
    orientation_d->coeffs() << this->orientation_d_.coeffs();
    *cartesian_stiffness = this->cartesian_stiffness_;
    *nullspace_stiffness = this->nullspace_stiffness_;
    *q_d_nullspace = this->q_d_nullspace_;
    *cartesian_damping << this->cartesian_damping_;
  }

  Eigen::VectorXd CartesianImpedanceController::getLastCommands() const
  {
    return this->tau_c_;
  }

  Eigen::Matrix<double, 6, 1> CartesianImpedanceController::getAppliedWrench() const
  {
    return this->cartesian_wrench_;
  }

  Eigen::Matrix<double, 6, 1> CartesianImpedanceController::getPoseError() const
  {
    return this->error_;
  }

  double CartesianImpedanceController::dampingRule(double stiffness) const
  {
    return 2 * sqrt(stiffness);
  }

  void CartesianImpedanceController::setUpdateFrequency(double freq)
  {
    assert(freq >= 0.0 && "Update frequency needs to be greater or equal to zero");
    this->update_frequency_ = std::max(freq, 0.0);
  }

  void CartesianImpedanceController::setFilterValue(double val, double *saved_val)
  {
    assert(val > 0 && val <= 1.0 && "Filter params need to be between 0 and 1.");
    *saved_val = saturateValue(val, 0.0000001, 1.0);
  }

  void CartesianImpedanceController::updateFilteredNullspaceConfig()
  {
    if (this->filter_params_nullspace_config_ == 1.0)
    {
      this->q_d_nullspace_ = this->q_d_nullspace_target_;
    }
    else
    {
      const double step = filterStep(this->update_frequency_, this->filter_params_nullspace_config_);
      this->q_d_nullspace_ = filteredUpdate(this->q_d_nullspace_target_, this->q_d_nullspace_, step);
    }
  }

  void CartesianImpedanceController::updateFilteredStiffness()
  {
    if (this->filter_params_stiffness_ == 1.0)
    {
      this->cartesian_stiffness_ = this->cartesian_stiffness_target_;
      this->cartesian_damping_ = this->cartesian_damping_target_;
      this->nullspace_stiffness_ = this->nullspace_stiffness_target_;
      this->q_d_nullspace_ = this->q_d_nullspace_target_;
      this->nullspace_damping_ = this->nullspace_damping_target_;
    }
    else
    {
      const double step = filterStep(this->update_frequency_, this->filter_params_stiffness_);

      this->cartesian_stiffness_ = filteredUpdate(this->cartesian_stiffness_target_, this->cartesian_stiffness_, step);
      this->cartesian_damping_ = filteredUpdate(this->cartesian_damping_target_, this->cartesian_damping_, step);
      this->nullspace_stiffness_ = filteredUpdate(this->nullspace_stiffness_target_, this->nullspace_stiffness_, step);
      this->nullspace_damping_ = filteredUpdate(this->nullspace_damping_target_, this->nullspace_damping_, step);
    }
  }

  void CartesianImpedanceController::updateFilteredPose()
  {
    if (this->filter_params_pose_ == 1.0)
    {
      position_d_ << position_d_target_;
      orientation_d_.coeffs() << orientation_d_target_.coeffs();
    }
    else
    {
      const double step = filterStep(this->update_frequency_, this->filter_params_pose_);

      this->position_d_ = filteredUpdate(this->position_d_target_, this->position_d_, step);
      this->orientation_d_ = this->orientation_d_.slerp(step, this->orientation_d_target_);
    }
  }

  void CartesianImpedanceController::updateFilteredWrench()
  {
    if (this->filter_params_wrench_ == 1.0)
    {
      this->cartesian_wrench_ = this->cartesian_wrench_target_;
    }
    else
    {
      const double step = filterStep(this->update_frequency_, this->filter_params_wrench_);
      this->cartesian_wrench_ = filteredUpdate(this->cartesian_wrench_target_, this->cartesian_wrench_, step);
    }
  }

} // namespace cartesian_impedance_controller
