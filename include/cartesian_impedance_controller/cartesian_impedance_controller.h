#pragma once

#include <Eigen/Dense>
#include <vector>

class CartesianImpedanceController
{
public:
  CartesianImpedanceController(const size_t n_joints = 7);
  ~CartesianImpedanceController() = default;

  // Set the desired diagonal stiffnessess + nullspace stiffness
  void setStiffness(const Eigen::Matrix<double, 7, 1> &stiffness);

  // Set the desired diagonal stiffnessess + nullspace stiffness
  void setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z, double n);

  // Set the desired diagonal stiffnessess
  void setStiffness(double t_x, double t_y, double t_z, double r_x, double r_y, double r_z);

  // Set the desired damping factors
  void setDamping(double d_x, double d_y, double d_z, double d_a, double d_b, double d_c, double d_n);

  // Set the desired end-effector pose
  void setDesiredPose(const Eigen::Vector3d &position_d, const Eigen::Quaterniond &orientation_d);

  // Set the desired nullspace configuration
  void setNullspaceConfig(const Eigen::VectorXd &q_d_nullspace_target);

  // Apply filtering on stiffness + end-effector pose. Default inactive && depends on update_frequency
  void setFiltering(double update_frequency, double filter_params_stiffness, double filter_params_pose,
                    double filter_params_wrench);

  // Maximum commanded torque change per time step
  void setMaxTorqueDelta(double d);

  // Apply a virtual Cartesian wrench in the world frame
  void applyWrench(const Eigen::Matrix<double, 6, 1> &cartesian_wrench);

  // Returns the desired control law
  Eigen::VectorXd calculateCommandedTorques(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                            const Eigen::Vector3d &position, Eigen::Quaterniond orientation,
                                            const Eigen::MatrixXd &jacobian);

  // Get the state of the robot. Updates when "calculateCommandedTorques" is called
  // void get_robot_state(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &dq, Eigen::Vector3d &position,
  //                      Eigen::Quaterniond &orientation, Eigen::Vector3d &position_d, Eigen::Quaterniond
  //                      &orientation_d, Eigen::Matrix<double, 6, 6> &cartesian_stiffness, double &nullspace_stiffness,
  //                      Eigen::Matrix<double, 7, 1> &q_d_nullspace,
  //                      Eigen::Matrix<double, 6, 6> &cartesian_damping) const;

  // Get the state of the robot. Updates when "calculateCommandedTorques" is called
  // void get_robot_state(Eigen::Vector3d &position_d, Eigen::Quaterniond &orientation_d,
  //                      Eigen::Matrix<double, 6, 6> &cartesian_stiffness, double &nullspace_stiffness,
  //                      Eigen::Matrix<double, 7, 1> &q_d_nullspace,
  //                      Eigen::Matrix<double, 6, 6> &cartesian_damping) const;

  // Get the currently applied commands
  Eigen::VectorXd getLastCommands() const;

  // Get the currently applied Cartesian wrench
  Eigen::Matrix<double, 6, 1> getAppliedWrench() const;

private:
  // Saturate the torque rate of the control law
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::VectorXd &tau_d_calculated,
                                                 Eigen::VectorXd &tau_J_d) const;

  // Saturate a variable x with the limits x_min and x_max
  double saturateValue(double x, double x_min, double x_max) const;

  // Update the robot state of the controller
  void updateStates(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::MatrixXd &jacobian,
                    const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

  // Adds some filtering effect to stiffness
  void updateFilteredStiffness();

  // Adds some filtering effect to the end-effector pose
  void updateFilteredPose();

  // Adds some filtering effect to the applied Cartesian wrench
  void updateFilteredWrench();

  // Robot variables
  const size_t n_joints_{ 7 };
  // End Effector
  Eigen::Vector3d position_;
  Eigen::Vector3d position_d_;
  Eigen::Vector3d position_d_target_;

  Eigen::Quaterniond orientation_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Quaterniond orientation_d_target_;

  // Joint State
  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  Eigen::MatrixXd jacobian_;

  // Stiffness parameters
  double nullspace_stiffness_{ 0.0 };
  double nullspace_stiffness_target_{ 0.0 };
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_{ Eigen::Matrix<double, 6, 6>::Identity() };
  Eigen::Matrix<double, 6, 6> cartesian_damping_{ Eigen::Matrix<double, 6, 6>::Identity() };
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_{ Eigen::Matrix<double, 6, 6>::Identity() };
  Eigen::Matrix<double, 7, 1> damping_factors_{ Eigen::Matrix<double, 7, 1>::Ones() };
  Eigen::VectorXd q_d_nullspace_;
  Eigen::VectorXd q_d_nullspace_target_;
  double nullspace_damping_;
  double nullspace_damping_target_;

  Eigen::VectorXd tau_d_{ Eigen::VectorXd::Zero(7) };

  // Last commanded torques
  Eigen::VectorXd tau_commanded_{ Eigen::VectorXd::Zero(7) };
  double delta_tau_max_{ 1.0 };

  // Filtering parameters
  double update_frequency_{ 100 };
  double filter_params_stiffness_{ 1 };
  double filter_params_pose_{ 1 };
  double filter_params_wrench_{ 1 };

  //"External" applied forces
  Eigen::VectorXd tau_ext_{ Eigen::VectorXd::Zero(7) };
  Eigen::Matrix<double, 6, 1> cartesian_wrench_target_;
  Eigen::Matrix<double, 6, 1> cartesian_wrench_;
};