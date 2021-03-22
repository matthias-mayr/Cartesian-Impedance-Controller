#include "cartesian_impedance_controller_base.h"
#include "pseudo_inversion.h"



bool CartesianImpedanceController_base::get_states(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Matrix<double, 7, 1> &q_interface, Eigen::Matrix<double, 7, 1> &dq_interface)
{
    //get from DART : q_interface, dq_interface and put them in q resp. dq
    q = q_interface;
    dq = dq_interface;
    return true;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController_base::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
    const Eigen::Matrix<double, 7, 1> &tau_J_d, const double delta_tau_max_)
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

bool CartesianImpedanceController_base::update_control(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq,
                                                      Eigen::Vector3d &position, Eigen::Quaterniond &orientation,
                                                      Eigen::Matrix<double, 6, 7> &jacobian, Eigen::VectorXd &tau_d,Eigen::VectorXd &tau_task,Eigen::VectorXd &tau_nullspace,Eigen::VectorXd &tau_wrench, Eigen::Matrix<double, 6, 1> &error)
{
    // compute error to desired pose
    // position error

    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
    {
        orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
    // convert to axis angle
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    // compute "orientation error"
    error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

    // compute control
    // allocate variables
   // Eigen::VectorXd tau_task(7), tau_nullspace(7);
    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() *
                    (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);

    // Desired torque. Used to contain coriolis as well
    tau_d << tau_task + tau_nullspace+tau_wrench;
    // Saturate torque rate to avoid discontinuities
    //tau_d << saturateTorqueRate(tau_d, tau_J_d_, delta_tau_max_);

    return true;
}

void CartesianImpedanceController_base::update_parameters(double filter_params_, double &nullspace_stiffness_,
                                                          double nullspace_stiffness_target_, const double delta_tau_max_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_,
                                                          Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_damping_,
                                                          Eigen::Matrix<double, 6, 6> cartesian_damping_target_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_,
                                                          Eigen::Matrix<double, 7, 1> q_d_nullspace_target_, Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_,
                                                          Eigen::Vector3d position_d_target_, Eigen::Quaterniond orientation_d_target_)
{
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    q_d_nullspace_ = filter_params_ * q_d_nullspace_target_ + (1.0 - filter_params_) * q_d_nullspace_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

    this->cartesian_stiffness_ = cartesian_stiffness_;
    this->cartesian_damping_ = cartesian_damping_;
    this->nullspace_stiffness_ = nullspace_stiffness_;
    this->position_d_ = position_d_;
    this->q_d_nullspace_ = q_d_nullspace_;
    this->orientation_d_ = orientation_d_;
}
