#pragma once

#include <Eigen/Dense>
#include <vector>

namespace cartesian_impedance_controller
{
  class CartesianImpedanceController
  {
  public:
    CartesianImpedanceController(const size_t n_joints = 7);
    ~CartesianImpedanceController() = default;

    // Sets pose without using filtering
    void initDesiredPose(const Eigen::Vector3d &position_d_target,
                         const Eigen::Quaterniond &orientation_d_target);

    // Sets nullspace configuration without using filtering
    void initNullspaceConfig(const Eigen::VectorXd &q_d_nullspace_target);

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
    void setFiltering(double update_frequency, double filter_params_nullspace_config, double filter_params_stiffness, double filter_params_pose,
                      double filter_params_wrench);

    // Maximum commanded torque change per time step
    void setMaxTorqueDelta(double d);

    // Maximum commanded torque change per time step
    void setMaxTorqueDelta(double d, double update_frequency);

    // Apply a virtual Cartesian wrench in the root frame (often "world")
    void applyWrench(const Eigen::Matrix<double, 6, 1> &cartesian_wrench);

    // Returns the commanded torques. Performs a filtering step and updates internal state.
    Eigen::VectorXd calculateCommandedTorques(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                              const Eigen::Vector3d &position, Eigen::Quaterniond orientation,
                                              const Eigen::MatrixXd &jacobian);

    // Get the state of the controller. Updates when "calculateCommandedTorques" is called
    void getState(Eigen::VectorXd *q, Eigen::VectorXd *dq, Eigen::Vector3d *position, Eigen::Quaterniond *orientation,
                  Eigen::Vector3d *position_d, Eigen::Quaterniond *orientation_d,
                  Eigen::Matrix<double, 6, 6> *cartesian_stiffness, double *nullspace_stiffness,
                  Eigen::VectorXd *q_d_nullspace, Eigen::Matrix<double, 6, 6> *cartesian_damping) const;

    // Get the state of the controller. Updates when "calculateCommandedTorques" is called
    void getState(Eigen::Vector3d *position_d, Eigen::Quaterniond *orientation_d,
                  Eigen::Matrix<double, 6, 6> *cartesian_stiffness, double *nullspace_stiffness,
                  Eigen::VectorXd *q_d_nullspace, Eigen::Matrix<double, 6, 6> *cartesian_damping) const;

    // Get the currently applied commands
    Eigen::VectorXd getLastCommands() const;

    // Get the currently applied Cartesian wrench
    Eigen::Matrix<double, 6, 1> getAppliedWrench() const;

    Eigen::Matrix<double, 6, 1> getPoseError() const;

  private:
    // Implements the damping based on a stiffness
    double dampingRule(double stiffness) const;

    // Applies the stiffness values to damping
    void applyDamping();

    void setUpdateFrequency(double freq);

    void setFilterValue(double val, double *saved_val);

    // Update the robot state of the controller
    void updateStates(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::MatrixXd &jacobian,
                      const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

    // Adds a percental filtering effect to the nullspace configuration
    void updateFilteredNullspaceConfig();

    // Adds a percental filtering effect to stiffness
    void updateFilteredStiffness();

    // Adds a percental filtering effect to the end-effector pose
    void updateFilteredPose();

    // Adds a percental filtering effect to the applied Cartesian wrench
    void updateFilteredWrench();

    // Robot variables
    const size_t n_joints_{7};
    // End Effector
    Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d position_d_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d position_d_target_{Eigen::Vector3d::Zero()};

    Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond orientation_d_target_{Eigen::Quaterniond::Identity()};

    // Joint state
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    // Jacobian. Row format: 3 translations, 3 rotation
    Eigen::MatrixXd jacobian_;
    Eigen::Matrix<double, 6, 1> error_;

    // End effector control parameters
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 6, 6> cartesian_damping_{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_{Eigen::Matrix<double, 6, 6>::Identity()};
    Eigen::Matrix<double, 7, 1> damping_factors_{Eigen::Matrix<double, 7, 1>::Ones()};

    // Nullspace control
    Eigen::VectorXd q_d_nullspace_;
    Eigen::VectorXd q_d_nullspace_target_;
    double nullspace_stiffness_{0.0};
    double nullspace_stiffness_target_{0.0};
    double nullspace_damping_{0.0};
    double nullspace_damping_target_{0.0};

    // Last commanded torques
    Eigen::VectorXd tau_commanded_;
    double delta_tau_max_{1.0};

    // Filtering parameters
    double update_frequency_{1000};
    double filter_params_nullspace_config_{1.0};
    double filter_params_stiffness_{1.0};
    double filter_params_pose_{1.0};
    double filter_params_wrench_{1.0};

    //  External applied forces
    Eigen::Matrix<double, 6, 1> cartesian_wrench_target_{Eigen::Matrix<double, 6, 1>::Zero()};
    Eigen::Matrix<double, 6, 1> cartesian_wrench_{Eigen::Matrix<double, 6, 1>::Zero()};
  };

} // namespace cartesian_impedance_controller