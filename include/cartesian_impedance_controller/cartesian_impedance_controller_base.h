
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#pragma once
class CartesianImpedanceController_base
{

public:

    bool get_states(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Matrix<double, 7, 1> &q_interface, Eigen::Matrix<double, 7, 1> &dq_interface);
    bool update_control(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq,
                                                      Eigen::Vector3d &position, Eigen::Quaterniond &orientation,
                                                      Eigen::Matrix<double, 6, 7> &jacobian, Eigen::VectorXd &tau_d,
                                                      Eigen::VectorXd &tau_task,Eigen::VectorXd &tau_nullspace,Eigen::VectorXd &tau_wrench, Eigen::Matrix<double, 6, 1> &error,Eigen::Matrix<double, 6, 1> &cartesian_wrench);
    void update_parameters(double filter_params_, double &nullspace_stiffness_,
                           double nullspace_stiffness_target_, const double delta_tau_max_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_,
                           Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_damping_,
                           Eigen::Matrix<double, 6, 6> cartesian_damping_target_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_,
                           Eigen::Matrix<double, 7, 1> q_d_nullspace_target_, Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_,
                           Eigen::Vector3d position_d_target_, Eigen::Quaterniond orientation_d_target_);
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d, const double delta_tau_max_);

    void update_compliance(double &translational_stiffness, double &rotational_stiffness, double &nullspace_stiffness, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_damping_target_,double &nullspace_stiffness_target_);
private:
    // Saturation
 // NOLINT (readability-identifier-naming)

    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;


    //Parameters
    double damping_factor_{5};
    double filter_params_;
    double nullspace_stiffness_;
    double nullspace_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_target_;
    Eigen::Matrix<double, 7, 1> tau_J_d_;
};