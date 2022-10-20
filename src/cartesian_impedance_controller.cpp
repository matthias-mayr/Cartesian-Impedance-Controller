#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include "pseudo_inversion.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace cartesian_impedance_controller
{
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

  template <typename T>
  T filteredUpdate(T target, T current, double filter)
  {
    return (1.0 - filter) * current + filter * target;
  }

  // Saturate a variable x with the limits x_min and x_max
  double saturateValue(double x, double x_min, double x_max)
  {
    return std::min(std::max(x, x_min), x_max);
  }

  // Saturate the torque rate to not stress the motors
  void saturateTorqueRate(const Eigen::VectorXd &tau_d_calculated, Eigen::VectorXd *tau_d_saturated, double delta_tau_max)
  {
    for (size_t i = 0; i < tau_d_calculated.size(); i++)
    {
      double difference = tau_d_calculated[i] - tau_d_saturated->operator()(i);
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
      assert(stiffness(i) >= 0.0 && "Stiffness values need to be positive.");
      // Set diagonal values of stiffness matrix
      this->cartesian_stiffness_target_(i, i) = stiffness(i);
    }
    assert(stiffness(6) >= 0.0 && "Stiffness values need to be positive.");
    this->nullspace_stiffness_target_ = stiffness(6);
    if (auto_damping)
    {
      this->applyDamping();
    }
  }

  // Set the desired diagonal stiffnessess + nullspace stiffness
  void CartesianImpedanceController::setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z,
                                                  double n, bool auto_damping)
  {
    Eigen::Matrix<double, 7, 1> stiffness_vector(7);
    stiffness_vector << t_x, t_y, t_z, r_x, r_y, r_z, n;
    this->setStiffness(stiffness_vector, auto_damping);
  }

  // Set the desired diagonal stiffnessess
  void CartesianImpedanceController::setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, bool auto_damping)
  {
    Eigen::Matrix<double, 7, 1> stiffness_vector(7);
    stiffness_vector << t_x, t_y, t_z, r_x, r_y, r_z, this->nullspace_stiffness_target_;
    this->setStiffness(stiffness_vector, auto_damping);
  }

  // Set the desired damping factors and applies them
  void CartesianImpedanceController::setDamping(double d_x, double d_y, double d_z, double d_a, double d_b, double d_c,
                                                double d_n)
  {
    Eigen::Matrix<double, 7, 1> damping_new;
    damping_new << d_x, d_y, d_z, d_a, d_b, d_c, d_n;
    for (size_t i = 0; i < damping_new.size(); i++)
    {
      if (damping_new(i) < 0)
      {
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
    this->nullspace_damping_target_ = this->damping_factors_(6) * this->dampingRule(this->nullspace_stiffness_target_);
  }

  // Set the desired end-effector pose
  void CartesianImpedanceController::setReferencePose(const Eigen::Vector3d &position_d_target,
                                                      const Eigen::Quaterniond &orientation_d_target)
  {
    this->position_d_target_ << position_d_target;
    this->orientation_d_target_.coeffs() << orientation_d_target.coeffs();
    this->orientation_d_target_.normalize();
  }

  // Set the desired nullspace configuration
  void CartesianImpedanceController::setNullspaceConfig(const Eigen::VectorXd &q_d_nullspace_target)
  {
    assert(q_d_nullspace_target.size() == this->n_joints_ && "Nullspace target needs to same size as n_joints_");
    this->q_d_nullspace_target_ << q_d_nullspace_target;
  }

  // Apply filtering on stiffness + end-effector pose. Default inactive && depends on update_frequency
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

  // Apply a virtual Cartesian wrench
  void CartesianImpedanceController::applyWrench(const Eigen::Matrix<double, 6, 1> &cartesian_wrench_target)
  {
    this->cartesian_wrench_target_ = cartesian_wrench_target;
  }

  // Updates state; calculates and returns the commanded torques
  Eigen::VectorXd CartesianImpedanceController::calculateCommandedTorques(const Eigen::VectorXd &q,
                                                                          const Eigen::VectorXd &dq,
                                                                          const Eigen::Vector3d &position,
                                                                          Eigen::Quaterniond orientation,
                                                                          const Eigen::MatrixXd &jacobian)
  {
    // Update controller to the current robot state
    this->q_ = q;
    this->dq_ = dq;
    this->position_ << position;
    this->orientation_.coeffs() << orientation.coeffs();
    this->jacobian_ << jacobian;

    return this->calculateCommandedTorques();
  }

  // Calculates and returns the commanded torques
  Eigen::VectorXd CartesianImpedanceController::calculateCommandedTorques()
  {
    // Perform a filtering step
    updateFilteredNullspaceConfig();
    updateFilteredStiffness();
    updateFilteredPose();
    updateFilteredWrench();

    // Compute error term
    error_.head(3) << this->position_ - this->position_d_;
    error_.tail(3) << calculateOrientationError(this->orientation_d_, this->orientation_);

    // Kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(this->jacobian_.transpose(), &jacobian_transpose_pinv);

    Eigen::VectorXd tau_task(this->n_joints_), tau_nullspace(this->n_joints_), tau_ext(this->n_joints_);

    // Cartesian PD control with damping ratio = 1
    tau_task << this->jacobian_.transpose() * (-this->cartesian_stiffness_ * this->error_ - this->cartesian_damping_ * (this->jacobian_ * this->dq_));
    // Nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - this->jacobian_.transpose() * jacobian_transpose_pinv) *
                         (this->nullspace_stiffness_ * (this->q_d_nullspace_ - this->q_) - this->nullspace_damping_ * this->dq_);
    // External wrench update
    tau_ext = this->jacobian_.transpose() * this->cartesian_wrench_;

    // Desired torque
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

  // Get the state of the robot. Updates when "calculateCommandedTorques" is called
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

  // Get the currently applied commands
  Eigen::VectorXd CartesianImpedanceController::getLastCommands() const
  {
    return this->tau_c_;
  }

  // Get the currently applied Cartesian wrench
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
    this->update_frequency_ = freq;
  }

  void CartesianImpedanceController::setFilterValue(double val, double *saved_val)
  {
    assert(val > 0 && val <= 1.0 && "Filter params need to be between 0 and 1.");
    *saved_val = val;
  }

  // Adds a percental filtering effect to the nullspace configuration
  void CartesianImpedanceController::updateFilteredNullspaceConfig()
  {
    const double step = this->filter_params_nullspace_config_ / this->update_frequency_;
    this->q_d_nullspace_ = filteredUpdate(this->q_d_nullspace_target_, this->q_d_nullspace_, step);
  }

  // Adds a percental filtering effect to stiffness
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
      const double step = this->filter_params_stiffness_ / this->update_frequency_;
      this->cartesian_stiffness_ = filteredUpdate(this->cartesian_stiffness_target_, this->cartesian_stiffness_, step);
      this->cartesian_damping_ = filteredUpdate(this->cartesian_damping_target_, this->cartesian_damping_, step);
      this->nullspace_stiffness_ = filteredUpdate(this->nullspace_stiffness_target_, this->nullspace_stiffness_, step);
      this->nullspace_damping_ = filteredUpdate(this->nullspace_damping_target_, this->nullspace_damping_, step);
    }
  }

  // Adds a percental filtering effect to the end-effector pose
  void CartesianImpedanceController::updateFilteredPose()
  {
    if (filter_params_pose_ == 1.0)
    {
      position_d_ << position_d_target_;
      orientation_d_.coeffs() << orientation_d_target_.coeffs();
    }
    else
    {
      const double step = this->filter_params_pose_ / this->update_frequency_;
      this->position_d_ = filteredUpdate(this->position_d_target_, this->position_d_, step);
      this->orientation_d_ = this->orientation_d_.slerp(step, this->orientation_d_target_);
    }
  }

  // Adds a percental filtering effect to the applied Cartesian wrench
  void CartesianImpedanceController::updateFilteredWrench()
  {
    const double step = this->filter_params_wrench_ / this->update_frequency_;
    this->cartesian_wrench_ = filteredUpdate(this->cartesian_wrench_target_, this->cartesian_wrench_, step);
  }

} // namespace cartesian_impedance_controller