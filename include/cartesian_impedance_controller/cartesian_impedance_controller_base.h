
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#pragma once
class CartesianImpedanceController_base
{

public:


    bool initialize();
    void set_stiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, double n);
    void set_desired_pose(Eigen::Vector3d position_d_, Eigen::Quaterniond orientation_d_);
    void set_nullspace_config(Eigen::Matrix<double, 7, 1> q_d_nullspace_target_);
    void set_filtering(double update_frequency, double filter_params_);

    //Returns the desired commands
    Eigen::VectorXd get_commanded_torques(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Vector3d &position, Eigen::Quaterniond &orientation, Eigen::Matrix<double, 6, 7> &jacobian);

    void get_robot_state(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Vector3d &position, Eigen::Quaterniond &orientation, Eigen::Vector3d &position_d_, Eigen::Quaterniond orientation_d_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_, double &nullspace_stiffness_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_);
    void get_robot_state(Eigen::Vector3d &position_d_, Eigen::Quaterniond orientation_d_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_, double &nullspace_stiffness_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_);

    void get_stiffness(Eigen::Matrix<double, 6, 6> &cartesian_stiffness_, double nullspace_stiffness_);
    //-------------------------------------------------------------------------------------------------
        //-------------------------------------------------------------------------------------------------
            //-------------------------------------------------------------------------------------------------
    bool update_control(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq,
                        Eigen::Vector3d &position, Eigen::Quaterniond &orientation,
                        Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_,
                        Eigen::Matrix<double, 6, 7> &jacobian, Eigen::VectorXd &tau_d,
                        Eigen::VectorXd &tau_task, Eigen::VectorXd &tau_nullspace);
    void update_parameters(double update_frequency, double filter_params_, double &nullspace_stiffness_,
                           double nullspace_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_,
                           Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_damping_,
                           Eigen::Matrix<double, 6, 6> cartesian_damping_target_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_,
                           Eigen::Matrix<double, 7, 1> q_d_nullspace_target_, Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_,
                           Eigen::Vector3d position_d_target_, Eigen::Quaterniond orientation_d_target_);
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        Eigen::Matrix<double, 7, 1> &tau_J_d, double delta_tau_max_);

    void update_compliance(Eigen::Vector3d translational_stiffness_target_, Eigen::Vector3d rotational_stiffness_target_, double nullspace_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_damping_target_, Eigen::Matrix<double, 6, 1> &damping_factors_);

    void rpy_to_quaternion(Eigen::Vector3d &rpy, Eigen::Quaterniond &q);
    void quaternion_to_rpy(Eigen::Quaterniond &q, Eigen::Vector3d &rpy);

    double saturate(double x, double x_min, double x_max);

private:
    // Robot pose and state variables
    Eigen::Vector3d position;
    Eigen::Vector3d position_d_;
    Eigen::Vector3d position_d_target_;

    Eigen::Quaterniond orientation;
    Eigen::Quaterniond orientation_d_;
    Eigen::Quaterniond orientation_d_target_;

    Eigen::Matrix<double, 7, 1> q;
    Eigen::Matrix<double, 7, 1> dq;

    // Filtering parameters
    double filter_params_{1};
    double update_frequency{100};

    // Stiffness parameters
    double nullspace_stiffness_;
    double nullspace_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_target_;

    // Rate limiter
    Eigen::Matrix<double, 7, 1> tau_J_d_;

    // Private functions-----

    // Update the state of the robot
    void update_states(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Vector3d &position, Eigen::Quaterniond &orientation,Eigen::Vector3d &position_d_target_, Eigen::Quaterniond &orientation_d_target_)
    {
        this->q = q;
        this->dq = dq;
        this->position << position;
        this->orientation.coeffs() << orientation.coeffs();
        this->position_d_target_ << position_d_target_;
        this->orientation_d_target_.coeffs() << orientation_d_target_.coeffs();

    }

    // Update impedance with some slope
    void update_filtering()
    {
        double filter_params_new_ = filter_params_ * 100 / update_frequency;
        cartesian_stiffness_ =
            filter_params_new_ * cartesian_stiffness_target_ + (1.0 - filter_params_new_) * cartesian_stiffness_;
        cartesian_damping_ =
            filter_params_new_ * cartesian_damping_target_ + (1.0 - filter_params_new_) * cartesian_damping_;
        nullspace_stiffness_ =
            filter_params_new_ * nullspace_stiffness_target_ + (1.0 - filter_params_new_) * nullspace_stiffness_;
        q_d_nullspace_ = filter_params_new_ * q_d_nullspace_target_ + (1.0 - filter_params_new_) * q_d_nullspace_;
        position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
        orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
    }
};