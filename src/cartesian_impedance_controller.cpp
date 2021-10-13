#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include "pseudo_inversion.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

CartesianImpedanceController::CartesianImpedanceController(const size_t n_joints) : n_joints_{ n_joints }
{
  // Robot state
  this->q_ = Eigen::VectorXd::Zero(n_joints_);
  this->dq_ = Eigen::VectorXd::Zero(n_joints_);
  this->jacobian_ = Eigen::MatrixXd::Zero(6, n_joints_);

  // End effector pose
  position_.setZero();
  position_d_.setZero();
  orientation_.coeffs() << 1., 0., 0., 0.;
  orientation_d_.coeffs() << 1., 0., 0., 0.;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 1., 0., 0., 0.;

  // Default stiffness values
  set_stiffness(200., 200., 200., 100., 100., 100., 0.);
  cartesian_stiffness_ << cartesian_stiffness_target_;
  nullspace_stiffness_target_ = 0;
  nullspace_stiffness_ = 0;
  q_d_nullspace_target_.setZero();

  // Default damping factors
  set_damping(1., 1., 1., 1., 1., 1., 1.);
  cartesian_damping_ << cartesian_damping_target_;
  nullspace_damping_ = nullspace_damping_target_;

  // Applied "External" forces
  cartesian_wrench_.setZero();
  cartesian_wrench_target_.setZero();
}

void CartesianImpedanceController::set_stiffness(const Eigen::Matrix<double, 7, 1> &stiffness)
{
  for (int i = 0; i < 6; i++)
  {
    cartesian_stiffness_target_(i, i) = stiffness(i);
    // Damping ratio = 1
    cartesian_damping_target_(i, i) = 2 * damping_factors_(i) * sqrt(stiffness(i));
  }
  this->nullspace_stiffness_target_ = stiffness[6];
  this->nullspace_damping_target_ = damping_factors_(6) * 2 * sqrt(this->nullspace_stiffness_target_);
}

// Set the desired diagonal stiffnessess + nullspace stiffness
void CartesianImpedanceController::set_stiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z,
                                                 double n)
{
  Eigen::Matrix<double, 7, 1> stiffness_vector(7);
  stiffness_vector << t_x, t_y, t_z, r_x, r_y, r_z, n;
  this->set_stiffness(stiffness_vector);
}

// Set the desired diagonal stiffnessess
void CartesianImpedanceController::set_stiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z)
{
  Eigen::Matrix<double, 7, 1> stiffness_vector(7);
  stiffness_vector << t_x, t_y, t_z, r_x, r_y, r_z, this->nullspace_stiffness_;
  this->set_stiffness(stiffness_vector);
}

// Set the desired damping factors + (TODO) nullspace damping
void CartesianImpedanceController::set_damping(double d_x, double d_y, double d_z, double d_a, double d_b, double d_c,
                                               double d_n)
{
  this->damping_factors_ << d_x, d_y, d_z, d_a, d_b, d_c, d_n;
  for (int i = 0; i < 6; i++)
  {
    cartesian_damping_target_(i, i) = 2 * damping_factors_(i) * sqrt(cartesian_stiffness_target_(i, i));
  }
  this->nullspace_damping_target_ = d_n * 2 * sqrt(this->nullspace_stiffness_target_);
}

// Set the desired enf-effector pose
void CartesianImpedanceController::set_desired_pose(const Eigen::Vector3d &position_d_target,
                                                    const Eigen::Quaterniond &orientation_d_target)
{
  this->position_d_target_ << position_d_target;
  this->orientation_d_target_.coeffs() << orientation_d_target.coeffs();
}

// Set the desired nullspace configuration
void CartesianImpedanceController::set_nullspace_config(const Eigen::VectorXd &q_d_nullspace_target)
{
  assert(q_d_nullspace_target.size() == this->n_joints_ && "Nullspace target needs to same size as n_joints_");
  this->q_d_nullspace_target_ << q_d_nullspace_target;
}

// Apply filtering on stiffness + end-effector pose. Default inactive && depends on update_frequency
void CartesianImpedanceController::set_filtering(double update_frequency, double filter_params_stiffness,
                                                 double filter_params_pose, double filter_params_wrench)
{
  this->update_frequency_ = update_frequency;
  this->filter_params_stiffness_ = filter_params_stiffness;
  this->filter_params_pose_ = filter_params_pose;
  this->filter_params_wrench_ = filter_params_wrench;
}

void CartesianImpedanceController::set_delta_tau_max(double d)
{
  if (d >= 0.0)
  {
    this->delta_tau_max_ = d;
  }
}

// Apply a virtual Cartesian wrench
void CartesianImpedanceController::apply_wrench(const Eigen::Matrix<double, 6, 1> &cartesian_wrench_target)
{
  this->cartesian_wrench_target_ = cartesian_wrench_target;
}

// Returns the desired control law
Eigen::VectorXd CartesianImpedanceController::get_commanded_torques(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                                                    const Eigen::Vector3d &position,
                                                                    Eigen::Quaterniond orientation,
                                                                    const Eigen::MatrixXd &jacobian)
{
  // Update controller to the current robot state
  update_states(q, dq, jacobian, position, orientation);

  // Updates stiffness with some filter
  update_filtering_stiffness();

  // Updates desired position with some filter
  update_filtering_pose();

  // Updates applied wrench with some filter
  update_filtering_wrench();

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << this->position_ - this->position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
  {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * this->orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  Eigen::VectorXd tau_task(7), tau_nullspace(7);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) - nullspace_damping_ * dq);
  // Desired torque. Used to contain coriolis as well
  this->tau_d_ << tau_task + tau_nullspace + this->tau_ext_;

  return this->saturate_torque_rate(this->tau_d_, this->tau_commanded_);
}

// Get the state of the robot. Updates when "get_commanded_torques" is called
// void CartesianImpedanceController::get_robot_state(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq,
//                                                    Eigen::Vector3d &position, Eigen::Quaterniond &orientation,
//                                                    Eigen::Vector3d &position_d, Eigen::Quaterniond &orientation_d,
//                                                    Eigen::Matrix<double, 6, 6> &cartesian_stiffness,
//                                                    double &nullspace_stiffness,
//                                                    Eigen::Matrix<double, 7, 1> &q_d_nullspace,
//                                                    Eigen::Matrix<double, 6, 6> &cartesian_damping) const
// {
//   q << this->q_;
//   dq << this->dq_;
//   position << this->position_;
//   orientation.coeffs() << this->orientation_.coeffs();
//   position_d << this->position_d_;
//   orientation_d.coeffs() << this->orientation_d_.coeffs();
//   cartesian_stiffness << this->cartesian_stiffness_;
//   nullspace_stiffness = this->nullspace_stiffness_;
//   q_d_nullspace << this->q_d_nullspace_;
//   cartesian_damping << this->cartesian_damping_;
// }

// Get the state of the robot. Updates when "get_commanded_torques" is called
// void CartesianImpedanceController::get_robot_state(Eigen::Vector3d &position_d, Eigen::Quaterniond &orientation_d,
//                                                    Eigen::Matrix<double, 6, 6> &cartesian_stiffness,
//                                                    double &nullspace_stiffness,
//                                                    Eigen::Matrix<double, 7, 1> &q_d_nullspace,
//                                                    Eigen::Matrix<double, 6, 6> &cartesian_damping) const
// {
//   position_d = this->position_d_;
//   orientation_d.coeffs() << this->orientation_d_.coeffs();
//   cartesian_stiffness = this->cartesian_stiffness_;
//   nullspace_stiffness = this->nullspace_stiffness_;
//   q_d_nullspace = this->q_d_nullspace_;
//   cartesian_damping << this->cartesian_damping_;
// }

// Get the currently applied commands
Eigen::VectorXd CartesianImpedanceController::get_last_commands() const
{
  return this->tau_commanded_;
}

// Get the currently applied Cartesian wrench
Eigen::Matrix<double, 6, 1> CartesianImpedanceController::get_applied_wrench() const
{
  return this->cartesian_wrench_;
}

// Saturate the torque rate to not stress the motors
Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturate_torque_rate(const Eigen::VectorXd &tau_d_calculated,
                                                                               Eigen::VectorXd &tau_d_saturated) const
{
  for (size_t i = 0; i < this->n_joints_; i++)
  {
    double difference = tau_d_calculated[i] - tau_d_saturated[i];
    tau_d_saturated[i] += this->saturate(difference, -delta_tau_max_, delta_tau_max_);
  }
  return tau_d_saturated;
}

// Saturate a variable x with the limits x_min and x_max
double CartesianImpedanceController::saturate(double x, double x_min, double x_max) const
{
  return std::min(std::max(x, x_min), x_max);
}

// Update the state of the robot
void CartesianImpedanceController::update_states(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                                 const Eigen::MatrixXd &jacobian, const Eigen::Vector3d &position,
                                                 const Eigen::Quaterniond &orientation)
{
  this->q_ = q;
  this->dq_ = dq;
  this->position_ << position;
  this->orientation_.coeffs() << orientation.coeffs();
  this->jacobian_ << jacobian;
}

// Adds some filtering effect to stiffness
void CartesianImpedanceController::update_filtering_stiffness()
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
    double filter_params_new = this->filter_params_stiffness_ * 100 / this->update_frequency_;
    cartesian_stiffness_ =
        filter_params_new * cartesian_stiffness_target_ + (1.0 - filter_params_new) * cartesian_stiffness_;
    cartesian_damping_ = filter_params_new * cartesian_damping_target_ + (1.0 - filter_params_new) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_new * nullspace_stiffness_target_ + (1.0 - filter_params_new) * nullspace_stiffness_;
    q_d_nullspace_ = filter_params_new * q_d_nullspace_target_ + (1.0 - filter_params_new) * q_d_nullspace_;
    nullspace_damping_ = filter_params_new * nullspace_damping_target_ + (1.0 - filter_params_new) * nullspace_damping_;
  }
}

// Adds some filtering effect to the end-effector pose
void CartesianImpedanceController::update_filtering_pose()
{
  if (filter_params_pose_ == 1)
  {
    position_d_ << position_d_target_;
    orientation_d_.coeffs() << orientation_d_target_.coeffs();
  }
  else
  {
    double filter_params_pose_new_ = filter_params_pose_ * 100 / update_frequency_;
    position_d_ = filter_params_pose_new_ * position_d_target_ + (1.0 - filter_params_pose_new_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_pose_new_, orientation_d_target_);
  }
}
// Adds some filtering effect to the applied Cartesian wrench
void CartesianImpedanceController::update_filtering_wrench()
{
  double filter_params_wrench_new = this->filter_params_wrench_ * 100 / update_frequency_;
  this->cartesian_wrench_ = filter_params_wrench_new * this->cartesian_wrench_target_ +
                            (1 - filter_params_wrench_new) * this->cartesian_wrench_;
  this->tau_ext_ = this->jacobian_.transpose() * this->cartesian_wrench_;
}