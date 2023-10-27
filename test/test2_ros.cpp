#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/Config.h>


//ROSBaseTests tested that key topics in the library are being written.
TEST(ROSBaseTests, referenceposeTests)
{
    ros::NodeHandle n;
    ros::Duration timeout(10);
  for (int i = 0; i < 5; i++) {
    auto msg_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
        "/CartesianImpedance_trajectory_controller/reference_pose", n, timeout);

    ASSERT_TRUE(msg_pose != nullptr) << "Did not receive message on iteration " << i + 1;

    EXPECT_LE(std::abs(msg_pose->pose.orientation.x), 11);
  }

}

TEST(ROSBaseTests, commandedtorqueTests)
{
    ros::NodeHandle n;
    ros::Duration timeout(5);
  for (int i = 0; i < 5; i++) {
    auto msg_torque = ros::topic::waitForMessage<std_msgs::Float64MultiArray>(
        "/CartesianImpedance_trajectory_controller/commanded_torques", n, timeout);

    ASSERT_TRUE(msg_torque != nullptr) << "Did not receive message on iteration " << i + 1;

    EXPECT_LE(std::abs(msg_torque->data[0]), 1000.0);
  }

}

//ROSReconfigureTests tested that we are able to modify external wrench, virtual stiffness, and damping online.
TEST(ROSReconfigureTests, wrenchTest) {
    ros::NodeHandle n;
    ros::Duration timeout(10);
  for (int i = 0; i < 5; i++) {
    auto msg_wup = ros::topic::waitForMessage<dynamic_reconfigure::Config>(
        "/CartesianImpedance_trajectory_controller/cartesian_wrench_reconfigure/parameter_updates", n, timeout);

    ASSERT_TRUE(msg_wup != nullptr) << "Did not receive message on iteration " << i + 1;

    for(const dynamic_reconfigure::DoubleParameter& double_param : msg_wup->doubles) {
        if(double_param.name == "f_x") {
            EXPECT_EQ(double_param.value, 5.0);
            break; 
        }
    }
  }   
}

TEST(ROSReconfigureTests, stiffnessTest) {
    ros::NodeHandle n;
    ros::Duration timeout(5);
  for (int i = 0; i < 5; i++) {
    auto msg_stup = ros::topic::waitForMessage<dynamic_reconfigure::Config>(
        "/CartesianImpedance_trajectory_controller/stiffness_reconfigure/parameter_updates", n, timeout);

    ASSERT_TRUE(msg_stup != nullptr) << "Did not receive message on iteration " << i + 1;

    for(const dynamic_reconfigure::DoubleParameter& double_param : msg_stup->doubles) {
        if(double_param.name == "translation_x") {
            EXPECT_EQ(double_param.value, 100.0);
            break; 
        }
    }
  }   
}

TEST(ROSReconfigureTests, dampingTest) {
    ros::NodeHandle n;
    ros::Duration timeout(5);
  for (int i = 0; i < 5; i++) {
    auto msg_dup = ros::topic::waitForMessage<dynamic_reconfigure::Config>(
        "/CartesianImpedance_trajectory_controller/damping_factors_reconfigure/parameter_updates", n, timeout);

    ASSERT_TRUE(msg_dup != nullptr) << "Did not receive message on iteration " << i + 1;

    for(const dynamic_reconfigure::DoubleParameter& double_param : msg_dup->doubles) {
        if(double_param.name == "translation_x") {
            EXPECT_EQ(double_param.value, 0.75);
            break; 
        }
    }
  }   
}

//ROSParameterTests make sure that all the necessary ROS parameters have been created.
TEST(ROSParameterTests, paramTest) {
    ros::NodeHandle n;
    ros::Duration timeout(5);
    std::vector<std::string> params_to_check = {
	"/CartesianImpedance_trajectory_controller/arm_id",
	"/CartesianImpedance_trajectory_controller/cartesian_wrench_reconfigure/apply_wrench",
	"/CartesianImpedance_trajectory_controller/cartesian_wrench_reconfigure/f_x",
	"/CartesianImpedance_trajectory_controller/cartesian_wrench_reconfigure/f_y",
	"/CartesianImpedance_trajectory_controller/cartesian_wrench_reconfigure/f_z",
	"/CartesianImpedance_trajectory_controller/cartesian_wrench_reconfigure/tau_x",
	"/CartesianImpedance_trajectory_controller/cartesian_wrench_reconfigure/tau_y",
	"/CartesianImpedance_trajectory_controller/cartesian_wrench_reconfigure/tau_z",
	"/CartesianImpedance_trajectory_controller/damping_factors_reconfigure/nullspace_damping",
	"/CartesianImpedance_trajectory_controller/damping_factors_reconfigure/rotation_x",
	"/CartesianImpedance_trajectory_controller/damping_factors_reconfigure/rotation_y",
	"/CartesianImpedance_trajectory_controller/damping_factors_reconfigure/rotation_z",
	"/CartesianImpedance_trajectory_controller/damping_factors_reconfigure/translation_x",
	"/CartesianImpedance_trajectory_controller/damping_factors_reconfigure/translation_y",
	"/CartesianImpedance_trajectory_controller/damping_factors_reconfigure/translation_z",
	"/CartesianImpedance_trajectory_controller/damping_factors_reconfigure/update_damping_factors",
	"/CartesianImpedance_trajectory_controller/delta_tau_max",
	"/CartesianImpedance_trajectory_controller/dynamic_reconfigure",
	"/CartesianImpedance_trajectory_controller/end_effector",
	"/CartesianImpedance_trajectory_controller/filtering/nullspace_config",
	"/CartesianImpedance_trajectory_controller/filtering/pose",
	"/CartesianImpedance_trajectory_controller/filtering/stiffness",
	"/CartesianImpedance_trajectory_controller/filtering/wrench",
	"/CartesianImpedance_trajectory_controller/handle_trajectories",
	"/CartesianImpedance_trajectory_controller/joints",
	"/CartesianImpedance_trajectory_controller/root_frame",
	"/CartesianImpedance_trajectory_controller/stiffness_reconfigure/nullspace_stiffness",
	"/CartesianImpedance_trajectory_controller/stiffness_reconfigure/rotation_x",
	"/CartesianImpedance_trajectory_controller/stiffness_reconfigure/rotation_y",
	"/CartesianImpedance_trajectory_controller/stiffness_reconfigure/rotation_z",
	"/CartesianImpedance_trajectory_controller/stiffness_reconfigure/translation_x",
	"/CartesianImpedance_trajectory_controller/stiffness_reconfigure/translation_y",
	"/CartesianImpedance_trajectory_controller/stiffness_reconfigure/translation_z",
	"/CartesianImpedance_trajectory_controller/stiffness_reconfigure/update_stiffness",
	"/CartesianImpedance_trajectory_controller/type",
	"/CartesianImpedance_trajectory_controller/update_frequency",
	"/CartesianImpedance_trajectory_controller/verbosity/state_msgs",
	"/CartesianImpedance_trajectory_controller/verbosity/tf_frames",
	"/CartesianImpedance_trajectory_controller/verbosity/verbose_print",
	"/CartesianImpedance_trajectory_controller/wrench_ee_frame"
    };

    for (const auto &param : params_to_check) {
        EXPECT_TRUE(ros::param::has(param)) << "Parameter [" << param << "] does NOT exist.";
    }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ros");
  return RUN_ALL_TESTS();
}










