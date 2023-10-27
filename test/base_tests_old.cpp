#include "cartesian_impedance_controller/cartesian_impedance_controller.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>


using namespace cartesian_impedance_controller;



CartesianImpedanceController cont; // Create an instance of the controller


TEST(ControllerTests, initializationTests){
	Eigen::Vector3d position_d_target(1.0, 2.0, 3.0);
    	Eigen::Quaterniond orientation_d_target(1.0, 0.0, 0.0, 0.0);
 
	EXPECT_NO_THROW(cont.initDesiredPose(position_d_target, orientation_d_target)); 
	
	
	Eigen::VectorXd q_d_nullspace_target(7);
    	q_d_nullspace_target << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;
	int n = 7;
	cont.setNumberOfJoints(n);
	EXPECT_NO_THROW(cont.initNullspaceConfig(q_d_nullspace_target)); //Should not fail since we have the same number of joints as q_d_nullspace_target elements
	
	cont.setNumberOfJoints(n+1);
	EXPECT_DEBUG_DEATH(cont.initNullspaceConfig(q_d_nullspace_target), "Nullspace target needs to same size as n_joints_"); //Should fail since we have differences between the number of joints and the number of q_d_nullspace_target elements
	
}


TEST(ControllerTests, setjointnumberTests){
	int n = 7; 
	EXPECT_NO_THROW(cont.setNumberOfJoints(n)); //This value of n shouldn't throw an exception.
	EXPECT_ANY_THROW(cont.setNumberOfJoints(-n)); //For a negative number of joints, we expect to catch the error.
}

TEST(ControllerTests, dampingTests){
	double d = 3.2;
	EXPECT_NO_THROW(cont.setDampingFactors(d,d,d,d,d,d,d)); //This should work.
	EXPECT_ANY_THROW(cont.setDampingFactors(-d,d,d,d,d,d,d)); //This should not work since damping should be positive.
}

TEST(ControllerTests, stiffnessTests) {
    Eigen::Matrix<double, 7, 1> stiffness_good, stiffness_bad;
    stiffness_good << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    stiffness_bad << -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    double t_x = 1.0, t_y = 1.0, t_z = 1.0, r_x = 1.0, r_y = 1.0, r_z = 1.0, n = 1.0;
    
    EXPECT_NO_THROW(cont.setStiffness(stiffness_good, false));
    EXPECT_DEBUG_DEATH(cont.setStiffness(stiffness_bad, false), "Stiffness values need to be positive.");
    EXPECT_NO_THROW(cont.setStiffness(t_x, t_y, t_z, r_x, r_y, r_z, false));
    EXPECT_NO_THROW(cont.setStiffness(t_x, t_y, t_z, r_x, r_y, r_z, n, false));
    
    EXPECT_NO_THROW(cont.setStiffness(stiffness_good, true));
    EXPECT_DEBUG_DEATH(cont.setStiffness(stiffness_bad, true), "Stiffness values need to be positive.");
    EXPECT_NO_THROW(cont.setStiffness(t_x, t_y, t_z, r_x, r_y, r_z, true));
    EXPECT_NO_THROW(cont.setStiffness(t_x, t_y, t_z, r_x, r_y, r_z, n, true));
}


TEST(ControllerTests, filteringTests){
    EXPECT_NO_THROW(cont.setFiltering(100, 0.5, 0.6, 0.7, 0.8));
    EXPECT_DEBUG_DEATH(cont.setFiltering(-100, 0.5, 0.6, 0.7, 0.8), "Update frequency needs to be greater or equal to zero");
    EXPECT_DEBUG_DEATH(cont.setFiltering(-100, 0.5, 0.6, 1.7, 0.8), "Filter params need to be between 0 and 1.");
}

TEST(ControllerTests, saturationTests){
    EXPECT_NO_THROW(cont.setMaxTorqueDelta(2.0));
    EXPECT_NO_THROW(cont.setMaxTorqueDelta(2.0,100.0));
    EXPECT_DEBUG_DEATH(cont.setMaxTorqueDelta(-2.0), "Allowed torque change must be positive");
    EXPECT_DEBUG_DEATH(cont.setMaxTorqueDelta(-2.0,100.0), "Allowed torque change must be positive");
    EXPECT_DEBUG_DEATH(cont.setMaxTorqueDelta(2.0,-100.0), "Update frequency needs to be greater or equal to zero");
}

TEST(ControllerTests, wrenchTest){
    Eigen::Matrix<double, 6, 1> wrench;
    wrench << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    EXPECT_NO_THROW(cont.applyWrench(wrench));
}

TEST(ControllerTests, getterTests){
    Eigen::VectorXd q(7);
    q << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Eigen::VectorXd dq(7);
    dq << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Eigen::Vector3d position(1.0, 2.0, 3.0);
    Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d position_d(2.0, 2.0, 3.0);
    Eigen::Quaterniond orientation_d(1.0, 0.0, 0.0, 0.0);
    Eigen::Matrix<double, 6, 6> cartesian_stiffness;
    double nullspace_stiffness = 1.0;
    Eigen::VectorXd q_d_nullspace(7);
    q_d_nullspace << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Eigen::Matrix<double, 6, 6> cartesian_damping;
    
    EXPECT_NO_THROW(cont.getState(&q, &dq, &position, &orientation, &position_d, &orientation_d,
                  &cartesian_stiffness, &nullspace_stiffness, &q_d_nullspace, &cartesian_damping));
    EXPECT_NO_THROW(cont.getAppliedWrench());
    EXPECT_NO_THROW(cont.getLastCommands());
    EXPECT_NO_THROW(cont.getPoseError());

    cont.setNumberOfJoints(7);
    Eigen::MatrixXd jacobian(6,7);
    jacobian.setZero();
    EXPECT_NO_THROW(cont.calculateCommandedTorques(q, dq, position, orientation, jacobian));

    
    Eigen::Matrix<double, 6, 1> error_getter, error_state;
    error_getter << cont.getPoseError();
    error_state.head(3) << position - position_d;
    
    EXPECT_DOUBLE_EQ(error_getter(0),error_state(0));
    EXPECT_DOUBLE_EQ(error_getter(1),error_state(1));
    EXPECT_DOUBLE_EQ(error_getter(2),error_state(2));
    
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

