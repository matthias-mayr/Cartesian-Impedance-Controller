#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/Config.h>

const std::string ctrl_name = "/CartesianImpedance_trajectory_controller";

//ROSBaseTests tested that key topics in the library are being written.
TEST(ROSBaseTests, referenceposeTests)
{
  ros::NodeHandle n;
  ros::Duration timeout(1);
  for (int i = 0; i < 5; i++) {
    auto msg_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
        ctrl_name + "/reference_pose", n, timeout);

    ASSERT_TRUE(msg_pose != nullptr) << "Did not receive message on iteration " << i + 1;

    EXPECT_LE(std::abs(msg_pose->pose.orientation.x), 11);
  }

}

TEST(ROSBaseTests, commandedtorqueTests)
{
  ros::NodeHandle n;
  ros::Duration timeout(5);
  const std::string torque_topic {ctrl_name + "/commanded_torques"};
  for (int i = 0; i < 5; i++) {
    auto msg_torque = ros::topic::waitForMessage<std_msgs::Float64MultiArray>(torque_topic, n, timeout);

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
        ctrl_name + "/cartesian_wrench_reconfigure/parameter_updates", n, timeout);

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
        ctrl_name + "/stiffness_reconfigure/parameter_updates", n, timeout);

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
        ctrl_name + "/damping_factors_reconfigure/parameter_updates", n, timeout);

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
  const std::vector<std::string> params_to_check = {
    "cartesian_wrench_reconfigure/apply_wrench",
    "cartesian_wrench_reconfigure/f_x",
    "cartesian_wrench_reconfigure/f_y",
    "cartesian_wrench_reconfigure/f_z",
    "cartesian_wrench_reconfigure/tau_x",
    "cartesian_wrench_reconfigure/tau_y",
    "cartesian_wrench_reconfigure/tau_z",
    "damping_factors_reconfigure/nullspace_damping",
    "damping_factors_reconfigure/rotation_x",
    "damping_factors_reconfigure/rotation_y",
    "damping_factors_reconfigure/rotation_z",
    "damping_factors_reconfigure/translation_x",
    "damping_factors_reconfigure/translation_y",
    "damping_factors_reconfigure/translation_z",
    "damping_factors_reconfigure/update_damping_factors",
    "delta_tau_max",
    "dynamic_reconfigure",
    "end_effector",
    "filtering/nullspace_config",
    "filtering/pose",
    "filtering/stiffness",
    "filtering/wrench",
    "handle_trajectories",
    "joints",
    "root_frame",
    "stiffness_reconfigure/nullspace_stiffness",
    "stiffness_reconfigure/rotation_x",
    "stiffness_reconfigure/rotation_y",
    "stiffness_reconfigure/rotation_z",
    "stiffness_reconfigure/translation_x",
    "stiffness_reconfigure/translation_y",
    "stiffness_reconfigure/translation_z",
    "stiffness_reconfigure/update_stiffness",
    "type",
    "update_frequency",
    "verbosity/state_msgs",
    "verbosity/tf_frames",
    "verbosity/verbose_print",
    "wrench_ee_frame"
    };

    for (const auto &param : params_to_check) {
      EXPECT_TRUE(ros::param::has("/" + ctrl_name + "/" + param)) << "Parameter [" << param << "] does NOT exist.";
    }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ros");
  return RUN_ALL_TESTS();
}
