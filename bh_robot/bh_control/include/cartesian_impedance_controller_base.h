
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#pragma once
class CartesianImpedanceController_base
{

public:
    CartesianImpedanceController_base(){};
    void initialize_parameters(Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_,
                               Eigen::Vector3d &position_d_target_, Eigen::Quaterniond &orientation_d_target_,
                               Eigen::Matrix<double, 6, 6> &cartesian_stiffness_, Eigen::Matrix<double, 6, 6> &cartesian_damping_);
    
    void initialize_parameters(Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_,
                               Eigen::Vector3d &position_d_target_, Eigen::Quaterniond &orientation_d_target_,
                               Eigen::Matrix<double, 7, 1> q_d_nullspace_, Eigen::Matrix<double, 7, 1> q_d_nullspace_target_);
    /*
     position_d_ = tmp_pos;
    orientation_d_ = tmp_quat;

    position_d_target_ = position_d_;
    orientation_d_target_ = orientation_d_;

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;
    q_d_nullspace_target_ = q_d_nullspace_;
    */
    bool getStates(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Matrix<double, 7, 1> &q_interface, Eigen::Matrix<double, 7, 1> &dq_interface);
    bool updateControl(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq,
                       Eigen::Vector3d &position, Eigen::Quaterniond &orientation, Eigen::Matrix<double, 6, 7> &jacobian,
                       Eigen::VectorXd &tau_d, Eigen::Matrix<double, 6, 1> &error);
    void update_parameters(double filter_params_, double &nullspace_stiffness_,
                           double nullspace_stiffness_target_, const double delta_tau_max_, Eigen::Matrix<double, 6, 6> &cartesian_stiffness_,
                           Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_, Eigen::Matrix<double, 6, 6> &cartesian_damping_,
                           Eigen::Matrix<double, 6, 6> cartesian_damping_target_, Eigen::Matrix<double, 7, 1> &q_d_nullspace_,
                           Eigen::Matrix<double, 7, 1> q_d_nullspace_target_, Eigen::Vector3d &position_d_, Eigen::Quaterniond &orientation_d_,
                           Eigen::Vector3d position_d_target_, Eigen::Quaterniond orientation_d_target_);

private:
    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
        const Eigen::Matrix<double, 7, 1> &tau_J_d); // NOLINT (readability-identifier-naming)

    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;

    std::string end_effector_;
    std::string robot_description_;
    unsigned int n_joints_;

    // The Jacobian of RBDyn comes with orientation in the first three lines. Needs to be interchanged.
    Eigen::VectorXi perm_indices_;
    Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_;

    //Parameters
    double filter_params_;
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{5.0};
    /*const*/ double delta_tau_max_;

    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_target_;
    Eigen::Matrix<double, 7, 1> tau_J_d_;
};