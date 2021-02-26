//#include <ros/ros.h> //for debugging
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <iiwa_tools/iiwa_tools.h>
#pragma once
class CartesianImpedanceController_base
{

public:
    void update_parameters();
    bool getStates(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Matrix<double, 7, 1> &q_interface, Eigen::Matrix<double, 7, 1> &dq_interface);
    bool updateControl(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Matrix<double, 7, 1> &q_interface, Eigen::Matrix<double, 7, 1> &dq_interaface, Eigen::VectorXd &tau_d, Eigen::Matrix<double, 6, 1> &error);
    bool get_fk(const Eigen::Matrix<double, 7, 1> &q, Eigen::Vector3d &translation, Eigen::Quaterniond &rotation);
    bool get_jacobian(const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq, Eigen::Matrix<double, 6, 7> &jacobian);

private:
    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    void publish();

    double filter_params_{0.005};
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{5.0};
    const double delta_tau_max_{1.0};
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_target_;
    Eigen::Matrix<double, 7, 1> tau_J_d_;

    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;

    // IIWA Tools - this is GPLv3
    iiwa_tools::IiwaTools _tools;
    std::string end_effector_;
    std::string robot_description_;
    unsigned int n_joints_;

    // The Jacobian of RBDyn comes with orientation in the first three lines. Needs to be interchanged.
    Eigen::VectorXi perm_indices_;
    Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_;
};