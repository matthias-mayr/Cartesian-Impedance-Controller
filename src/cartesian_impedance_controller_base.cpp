#include "cartesian_impedance_controller/cartesian_impedance_controller_base.h"
#include "pseudo_inversion.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

//Initialization
bool CartesianImpedanceController_base::initialize()
{
    //Make sure the function can only be called once
    static uint64_t c;
    if (c++ == 0)
    {
        //Default: Filter not active
        filter_params_ = 1;
        update_frequency = 100;

        // initialize pose
        position.setZero();
        position_d_.setZero();
        orientation.coeffs() << 1., 0., 0., 0.;
        orientation_d_.coeffs() << 1., 0., 0., 0.;

        position_d_target_.setZero();
        orientation_d_target_.coeffs() << 1., 0., 0., 0.;
        // default stiffness
        set_stiffness(200., 200., 200., 100., 100., 100., 0.);
        cartesian_damping_ << cartesian_damping_target_;
        damping_factors_ << 1., 1., 1., 1., 1., 1;
        cartesian_stiffness_ << cartesian_stiffness_target_;
        q_d_nullspace_target_ << q;

        // "External" forces
        tau_ext.resize(7);
        tau_ext.setZero();
        cartesian_wrench.setZero();

        return true;
    }
    return false;
}

void CartesianImpedanceController_base::set_stiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, double n)
{
    Eigen::VectorXd stiffness_vector_(6);
    stiffness_vector_ << t_x, t_y, t_z, r_x, r_y, r_z;
    cartesian_stiffness_target_.setIdentity();
    cartesian_damping_target_.setIdentity();
    for (int i = 0; i < 6; i++)
    {
        cartesian_stiffness_target_(i, i) = stiffness_vector_(i);
        // Damping ratio = 1
        cartesian_damping_target_(i, i) = 2 * damping_factors_(i) * sqrt(stiffness_vector_(i));
    }
    nullspace_stiffness_target_ = n;
}
//Set the damping factors >0.1 && < 1. First 6 parameters represent the diagonals of the damping matrix and the last parameter represents the nullspace damping.
void CartesianImpedanceController_base::set_damping(double d_x, double d_y, double d_z, double d_a, double d_b, double d_c, double d_n)
{
    this->damping_factors_ << d_x, d_y, d_z, d_a, d_b, d_c;

    for (int i = 0; i < 6; i++)
    {
        cartesian_damping_target_(i, i) = 2 * damping_factors_(i) * sqrt(cartesian_stiffness_target_(i, i));
    }
}

void CartesianImpedanceController_base::set_desired_pose(Eigen::Vector3d position_d_target_, Eigen::Quaterniond orientation_d_target_)
{
    this->position_d_target_ << position_d_target_;
    this->orientation_d_target_.coeffs() << orientation_d_target_.coeffs();
}

void CartesianImpedanceController_base::set_nullspace_config(Eigen::Matrix<double, 7, 1> q_d_nullspace_target_)
{
    this->q_d_nullspace_target_ << q_d_nullspace_target_;
}

void CartesianImpedanceController_base::set_filtering(double update_frequency, double filter_params_)
{
    this->update_frequency = update_frequency;
    this->filter_params_ = filter_params_;
}

void CartesianImpedanceController_base::get_robot_state(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Vector3d &position, Eigen::Quaterniond &orientation, Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_, double &nullspace_stiffness_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_, Eigen::Matrix<double, 6, 6> &cartesian_damping_)
{
    q << this->q;
    dq << this->dq;
    position << this->position;
    orientation.coeffs() << this->orientation.coeffs();
    position_d_ << this->position_d_;
    orientation_d_.coeffs() << this->orientation_d_.coeffs();
    cartesian_stiffness_ << this->cartesian_stiffness_;
    nullspace_stiffness_ = this->nullspace_stiffness_;
    q_d_nullspace_ << this->q_d_nullspace_;
    cartesian_damping_ << this->cartesian_damping_;
}

void CartesianImpedanceController_base::get_robot_state(Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_, double &nullspace_stiffness_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_, Eigen::Matrix<double, 6, 6> &cartesian_damping_)
{
    position_d_ = this->position_d_;
    orientation_d_.coeffs() << this->orientation_d_.coeffs();
    cartesian_stiffness_ = this->cartesian_stiffness_;
    nullspace_stiffness_ = this->nullspace_stiffness_;
    q_d_nullspace_ = this->q_d_nullspace_;
    cartesian_damping_ << this->cartesian_damping_;
}

//returns tau_desired
Eigen::VectorXd CartesianImpedanceController_base::get_commanded_torques(Eigen::Matrix<double, 7, 1> q, Eigen::Matrix<double, 7, 1> dq, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Matrix<double, 6, 7> jacobian)
{
    // Update controller to the current robot state
    update_states(q, dq, jacobian, position, orientation, position_d_target_, orientation_d_target_);

    //Updates stiffness
    update_filtering();

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
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
    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    Eigen::VectorXd tau_d(7), tau_task(7), tau_nullspace(7);

    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() *
                    (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);
    // Desired torque. Used to contain coriolis as well
    tau_d << tau_task + tau_nullspace + tau_ext;
    return tau_d;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController_base::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
    Eigen::Matrix<double, 7, 1> &tau_J_d, double delta_tau_max_)
{ // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++)
    {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] =
            tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    // saves last desired torque.
    for (size_t i = 0; i < 7; i++)
    {
        tau_J_d[i] = tau_d_saturated[i];
    }
    return tau_d_saturated;
}

void CartesianImpedanceController_base::apply_wrench(Eigen::Matrix<double, 6, 1> cartesian_wrench)
{
    tau_ext = jacobian.transpose() * cartesian_wrench;
    this->cartesian_wrench=cartesian_wrench;
}

Eigen::Matrix<double,6,1> CartesianImpedanceController_base::get_applied_wrench()
{
    return cartesian_wrench;
}

bool CartesianImpedanceController_base::update_control(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq,
                                                       Eigen::Vector3d &position, Eigen::Quaterniond &orientation,
                                                       Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_,
                                                       Eigen::Matrix<double, 6, 7> &jacobian, Eigen::VectorXd &tau_d,
                                                       Eigen::VectorXd &tau_task, Eigen::VectorXd &tau_nullspace)
{
    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
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
    tau_d << tau_task + tau_nullspace;
    return true;
}

void CartesianImpedanceController_base::update_parameters(double update_frequency, double filter_params_, double &nullspace_stiffness_,
                                                          double nullspace_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_,
                                                          Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_damping_,
                                                          Eigen::Matrix<double, 6, 6> cartesian_damping_target_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_,
                                                          Eigen::Matrix<double, 7, 1> q_d_nullspace_target_, Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_,
                                                          Eigen::Vector3d position_d_target_, Eigen::Quaterniond orientation_d_target_)
{
    filter_params_ = filter_params_ * 100 / update_frequency;
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;

    q_d_nullspace_ = filter_params_ * q_d_nullspace_target_ + (1.0 - filter_params_) * q_d_nullspace_;

    filter_params_ = filter_params_ * update_frequency / 100;
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

void CartesianImpedanceController_base::update_compliance(Eigen::Vector3d translational_stiffness_target_, Eigen::Vector3d rotational_stiffness_target_, double nullspace_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_damping_target_, Eigen::Matrix<double, 6, 1> &damping_factors_)
{
    cartesian_stiffness_target_.setIdentity();
    Eigen::Matrix3d K_t = translational_stiffness_target_.asDiagonal();
    Eigen::Matrix3d K_r = rotational_stiffness_target_.asDiagonal();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << K_t;
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << K_r;

    cartesian_damping_target_.setIdentity();
    cartesian_damping_target_.topLeftCorner(3, 3)
        << 2 * K_t.cwiseSqrt();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 2 * K_r.cwiseSqrt();

    for (size_t i = 0; i < 6; i++)
    {
        cartesian_damping_target_(i, i) = cartesian_damping_target_(i, i) * damping_factors_(i);
    }
}

void CartesianImpedanceController_base::rpy_to_quaternion(Eigen::Vector3d &rpy, Eigen::Quaterniond &q)
{
    q =
        Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
}

void CartesianImpedanceController_base::quaternion_to_rpy(Eigen::Quaterniond &q, Eigen::Vector3d &rpy)
{

    rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
}

double CartesianImpedanceController_base::saturate(double x, double x_min, double x_max)
{

    if (x > x_max)
    {
        x = x_max;
    }
    if (x < x_min)
    {
        x = x_min;
    }
    return x;
}
//----------------------------------------------------
