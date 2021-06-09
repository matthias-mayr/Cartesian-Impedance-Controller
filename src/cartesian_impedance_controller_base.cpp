#include "cartesian_impedance_controller/cartesian_impedance_controller_base.h"
#include "pseudo_inversion.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

// Initialization
bool CartesianImpedanceController_base::initialize()
{
    //Make sure that the function can only be called once
    static uint64_t c;
    if (c++ == 0)
    {
        // Robot state
        q.setZero();
        dq.setZero();
        jacobian.setZero();

        // End effector pose
        position.setZero();
        position_d_.setZero();
        orientation.coeffs() << 1., 0., 0., 0.;
        orientation_d_.coeffs() << 1., 0., 0., 0.;
        position_d_target_.setZero();
        orientation_d_target_.coeffs() << 1., 0., 0., 0.;

        // Default stiffness values
        set_stiffness(200., 200., 200., 100., 100., 100., 0.);
        cartesian_stiffness_ << cartesian_stiffness_target_;
        nullspace_stiffness_target_=0;
        nullspace_stiffness_=0;
        q_d_nullspace_target_.setZero();

        // Default damping factors
        set_damping(1.,1.,1.,1.,1.,1.,1.);
        cartesian_damping_ << cartesian_damping_target_;
        nullspace_damping_=nullspace_damping_target_;

        // Applied "External" forces
        tau_ext.resize(7);
        tau_ext.setZero();
        cartesian_wrench.setZero();
        cartesian_wrench_target_.setZero();

        return true;
    }
    return false;
}

// Set the desired diagonal stiffnessess + nullspace stiffness
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
    nullspace_damping_target_=damping_factors_(6)*2*sqrt(nullspace_stiffness_target_);
}

// Set the desired damping factors + (TODO) nullspace damping
void CartesianImpedanceController_base::set_damping(double d_x, double d_y, double d_z, double d_a, double d_b, double d_c, double d_n)
{
    this->damping_factors_ << d_x, d_y, d_z, d_a, d_b, d_c,d_n;
    for (int i = 0; i < 6; i++)
    {
        cartesian_damping_target_(i, i) = 2 * damping_factors_(i) * sqrt(cartesian_stiffness_target_(i, i));
    }
    nullspace_damping_target_=d_n*2*sqrt(nullspace_stiffness_target_);
}

// Set the desired enf-effector pose
void CartesianImpedanceController_base::set_desired_pose(Eigen::Vector3d position_d_target_, Eigen::Quaterniond orientation_d_target_)
{
    this->position_d_target_ << position_d_target_;
    this->orientation_d_target_.coeffs() << orientation_d_target_.coeffs();
}

// Set the desired nullspace configuration
void CartesianImpedanceController_base::set_nullspace_config(Eigen::Matrix<double, 7, 1> q_d_nullspace_target_)
{
    this->q_d_nullspace_target_ << q_d_nullspace_target_;
}

// Apply filtering on stiffness + end-effector pose. Default inactive && depends on update_frequency
void CartesianImpedanceController_base::set_filtering(double update_frequency, double filter_params_stiffness, double filter_params_pose, double filter_params_wrench)
{
    this->update_frequency = update_frequency;
    this->filter_params_stiffness = filter_params_stiffness;
    this->filter_params_pose=filter_params_pose;
    this->filter_params_wrench=filter_params_wrench;
}

// Returns the desired control law
Eigen::VectorXd CartesianImpedanceController_base::get_commanded_torques(Eigen::Matrix<double, 7, 1> q, Eigen::Matrix<double, 7, 1> dq, Eigen::Vector3d position, Eigen::Quaterniond orientation, Eigen::Matrix<double, 6, 7> jacobian)
{
    // Update controller to the current robot state
    update_states(q, dq, jacobian, position, orientation, position_d_target_, orientation_d_target_);

    // Updates stiffness with some filter
    update_filtering_stiffness();

    // Updates desired position with some filter
    update_filtering_pose();

    // Updates applied wrench with some filter
    update_filtering_wrench();

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

    tau_d.resize(7);
    Eigen::VectorXd  tau_task(7), tau_nullspace(7);

    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() *
                    (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    // nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                          nullspace_damping_ * dq);
    // Desired torque. Used to contain coriolis as well
    tau_d << tau_task + tau_nullspace + tau_ext;
    return tau_d;
}

// Get the state of the robot. Updates when "get_commanded_torques" is called
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

// Get the state of the robot. Updates when "get_commanded_torques" is called
void CartesianImpedanceController_base::get_robot_state(Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_, double &nullspace_stiffness_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_, Eigen::Matrix<double, 6, 6> &cartesian_damping_)
{
    position_d_ = this->position_d_;
    orientation_d_.coeffs() << this->orientation_d_.coeffs();
    cartesian_stiffness_ = this->cartesian_stiffness_;
    nullspace_stiffness_ = this->nullspace_stiffness_;
    q_d_nullspace_ = this->q_d_nullspace_;
    cartesian_damping_ << this->cartesian_damping_;
}

// Get the currently applied commands
Eigen::VectorXd CartesianImpedanceController_base::get_commands()
{
    tau_d.resize(7);
    return tau_d;
}

// Apply a virtual Cartesian wrench
void CartesianImpedanceController_base::apply_wrench(Eigen::Matrix<double, 6, 1> cartesian_wrench_target_)
{
    this->cartesian_wrench_target_ = cartesian_wrench_target_;
}

// Get the currently applied Cartesian wrench
Eigen::Matrix<double, 6, 1> CartesianImpedanceController_base::get_applied_wrench()
{
    return cartesian_wrench;
}

// Saturate the torque rate of the control law
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

// Saturate a variable x with the limits x_min and x_max
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
