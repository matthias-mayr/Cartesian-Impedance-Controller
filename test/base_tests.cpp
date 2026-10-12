#include "cartesian_impedance_controller/cartesian_impedance_controller.hpp"
#include "cartesian_impedance_controller/transform_wrench.hpp"
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace cartesian_impedance_controller;



CartesianImpedanceController cont; // Create an instance of the controller


TEST(ControllerTests, initializationTests){
	const Eigen::Vector3d position_d_target(1.0, 2.0, 3.0);
    const Eigen::Quaterniond orientation_d_target(1.0, 0.0, 0.0, 0.0);

	EXPECT_NO_THROW(cont.initDesiredPose(position_d_target, orientation_d_target));
}

TEST(ControllerTests, nullspaceConfigTests){
	Eigen::VectorXd q_d_nullspace_target(7);
    q_d_nullspace_target << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;
	const int n = 7;
	cont.setNumberOfJoints(n);
	EXPECT_NO_THROW(cont.initNullspaceConfig(q_d_nullspace_target)); //Should not fail since we have the same number of joints as q_d_nullspace_target elements

	cont.setNumberOfJoints(n+1);
	EXPECT_DEBUG_DEATH(cont.initNullspaceConfig(q_d_nullspace_target), "Nullspace target needs to same size as n_joints_"); //Should fail since we have differences between the number of joints and the number of q_d_nullspace_target elements
}

TEST(ControllerTests, setjointnumberTests){
	const int n = 7;
	EXPECT_NO_THROW(cont.setNumberOfJoints(n)); //This value of n shouldn't throw an exception.
	EXPECT_ANY_THROW(cont.setNumberOfJoints(-n)); //For a negative number of joints, we expect to catch the error.
}

TEST(ControllerTests, dampingTests){
	const double d = 3.2;
	EXPECT_NO_THROW(cont.setDampingFactors(d,d,d,d,d,d,d)); //This should work.
	EXPECT_NO_THROW(cont.setDampingFactors(-d,d,d,d,d,d,d)); //Negative damping factors are ignored outside debug builds

    EXPECT_DEBUG_DEATH(cont.setDampingFactors(-d,d,d,d,d,d,d), "Negative damping factors are tested by asserts"); //This should not work since damping should be positive
}

TEST(ControllerTests, stiffnessTests) {
    Eigen::Matrix<double, 7, 1> stiffness_good, stiffness_bad;
    stiffness_good << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    stiffness_bad << -1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    double t_x = 1.0, t_y = 1.0, t_z = 1.0, r_x = 1.0, r_y = 1.0, r_z = 1.0, n = 1.0;

    // Without auto-damping
    EXPECT_NO_THROW(cont.setStiffness(stiffness_good, false));
    EXPECT_DEBUG_DEATH(cont.setStiffness(stiffness_bad, false), "Stiffness values need to be positive.");
    EXPECT_NO_THROW(cont.setStiffness(t_x, t_y, t_z, r_x, r_y, r_z, false));
    EXPECT_NO_THROW(cont.setStiffness(t_x, t_y, t_z, r_x, r_y, r_z, n, false));
    // With auto-damping
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

TEST(TransformWrenchTests, ZeroWrenchInTranslatedFrameRemainsZero)
{
  geometry_msgs::msg::WrenchStamped wrench;
  wrench.header.frame_id = "ee_frame";
  wrench.wrench.force.x = 0.0;
  wrench.wrench.force.y = 0.0;
  wrench.wrench.force.z = 0.0;
  wrench.wrench.torque.x = 0.0;
  wrench.wrench.torque.y = 0.0;
  wrench.wrench.torque.z = 0.0;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "ee_frame";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;

  auto out = transformWrench(wrench, transform);

  EXPECT_EQ(out.header.frame_id, "world");
  EXPECT_DOUBLE_EQ(out.wrench.force.x, 0.0);
  EXPECT_DOUBLE_EQ(out.wrench.force.y, 0.0);
  EXPECT_DOUBLE_EQ(out.wrench.force.z, 0.0);
  EXPECT_DOUBLE_EQ(out.wrench.torque.x, 0.0);
  EXPECT_DOUBLE_EQ(out.wrench.torque.y, 0.0);
  EXPECT_DOUBLE_EQ(out.wrench.torque.z, 0.0);
}

TEST(TransformWrenchTests, NonZeroForceWithTranslationAppliesOnlyRotation)
{
  // Input wrench in ee_frame with non-zero force and torque
  geometry_msgs::msg::WrenchStamped wrench;
  wrench.header.frame_id = "ee_frame";
  wrench.wrench.force.x = 10.0;
  wrench.wrench.force.y = -5.0;
  wrench.wrench.force.z = 3.0;
  wrench.wrench.torque.x = 1.0;
  wrench.wrench.torque.y = 2.0;
  wrench.wrench.torque.z = -1.0;

  // Transform with non-zero translation and a 90-degree rotation about the Z axis.
  // Under +90 deg rotation around Z: (x, y, z) -> (-y, x, z).
  // Force (10, -5, 3) rotated by +90 deg around Z becomes (5, 10, 3).
  // Torque (1, 2, -1) rotated by +90 deg around Z becomes (-2, 1, -1).
  // The non-zero translation (5.0, 10.0, -15.0) MUST NOT contribute to the force vector.
  // With the old buggy implementation (tf * force_in), translation was added to force,
  // resulting in force = (5 + 5, 10 + 10, 3 - 15) = (10, 20, -12).
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "ee_frame";
  transform.transform.translation.x = 5.0;
  transform.transform.translation.y = 10.0;
  transform.transform.translation.z = -15.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI_2);
  transform.transform.rotation = tf2::toMsg(q);

  auto out = transformWrench(wrench, transform);

  EXPECT_EQ(out.header.frame_id, "world");
  EXPECT_NEAR(out.wrench.force.x, 5.0, 1e-6);
  EXPECT_NEAR(out.wrench.force.y, 10.0, 1e-6);
  EXPECT_NEAR(out.wrench.force.z, 3.0, 1e-6);
  EXPECT_NEAR(out.wrench.torque.x, -2.0, 1e-6);
  EXPECT_NEAR(out.wrench.torque.y, 1.0, 1e-6);
  EXPECT_NEAR(out.wrench.torque.z, -1.0, 1e-6);
}

TEST(TransformWrenchTests, PureTranslationDoesNotAlterForceVector)
{
  geometry_msgs::msg::WrenchStamped wrench;
  wrench.header.frame_id = "ee_frame";
  wrench.wrench.force.x = 7.5;
  wrench.wrench.force.y = -3.2;
  wrench.wrench.force.z = 4.1;
  wrench.wrench.torque.x = 0.5;
  wrench.wrench.torque.y = -0.8;
  wrench.wrench.torque.z = 1.2;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "ee_frame";
  transform.transform.translation.x = 100.0;
  transform.transform.translation.y = -200.0;
  transform.transform.translation.z = 300.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;

  auto out = transformWrench(wrench, transform);

  EXPECT_DOUBLE_EQ(out.wrench.force.x, 7.5);
  EXPECT_DOUBLE_EQ(out.wrench.force.y, -3.2);
  EXPECT_DOUBLE_EQ(out.wrench.force.z, 4.1);
  EXPECT_DOUBLE_EQ(out.wrench.torque.x, 0.5);
  EXPECT_DOUBLE_EQ(out.wrench.torque.y, -0.8);
  EXPECT_DOUBLE_EQ(out.wrench.torque.z, 1.2);
}

TEST(TransformWrenchTests, MatrixOverloadNonZeroForceWithTranslationAppliesOnlyRotation)
{
  Eigen::Matrix<double, 6, 1> wrench;
  wrench << 10.0, -5.0, 3.0, 1.0, 2.0, -1.0;

  geometry_msgs::msg::Transform transform;
  transform.translation.x = 5.0;
  transform.translation.y = 10.0;
  transform.translation.z = -15.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI_2);
  transform.rotation = tf2::toMsg(q);

  transformWrench(&wrench, transform);

  EXPECT_NEAR(wrench(0), 5.0, 1e-6);
  EXPECT_NEAR(wrench(1), 10.0, 1e-6);
  EXPECT_NEAR(wrench(2), 3.0, 1e-6);
  EXPECT_NEAR(wrench(3), -2.0, 1e-6);
  EXPECT_NEAR(wrench(4), 1.0, 1e-6);
  EXPECT_NEAR(wrench(5), -1.0, 1e-6);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

