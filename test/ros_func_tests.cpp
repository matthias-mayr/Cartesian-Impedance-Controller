#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/Config.h>
#include <cartesian_impedance_controller/ControllerState.h>

const std::string ctrl_name = "/CartesianImpedance_trajectory_controller";

//ROSFunctionalityTests tested the controller functionality

TEST(ROSFunctionalityTests, forceTests) // Tested that a desired external force is being applied.
{
  ros::NodeHandle n;
  ros::Duration timeout(5);
  ros::Publisher wrench_publisher = n.advertise<geometry_msgs::WrenchStamped>(ctrl_name +"/set_cartesian_wrench", 500);

  const std::string controller_state {ctrl_name + "/controller_state"};
  auto msg_controller_up = ros::topic::waitForMessage<cartesian_impedance_controller::ControllerState>(controller_state, n, timeout);
  ASSERT_TRUE(msg_controller_up != nullptr) << "Controller is not up.";
  EXPECT_DOUBLE_EQ(std::abs(msg_controller_up->commanded_wrench.force.x), 0.0);

  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.frame_id = "world";
  wrench_msg.header.stamp = ros::Time::now();
  wrench_msg.wrench.force.x = 5.0;
  wrench_msg.wrench.force.y = 0.0;
  wrench_msg.wrench.force.z = 0.0;
  wrench_msg.wrench.torque.x = 0.0;
  wrench_msg.wrench.torque.y = 0.0;
  wrench_msg.wrench.torque.z = 0.0;
  wrench_publisher.publish(wrench_msg);
  ros::Duration(1).sleep();

  for (int i = 0; i < 5; i++) {
    auto msg_state = ros::topic::waitForMessage<cartesian_impedance_controller::ControllerState>(controller_state, n, timeout);

    ASSERT_TRUE(msg_state != nullptr) << "Did not receive message on iteration " << i + 1;
    EXPECT_GE(std::abs(msg_state->commanded_wrench.force.x), 1.0);
  }

  // Cancel the wrench
  wrench_msg.header.stamp = ros::Time::now();
  wrench_msg.wrench.force.x = 0.0;
  wrench_publisher.publish(wrench_msg);
  ros::Duration(1).sleep();
}

TEST(ROSFunctionalityTests, motionTests) // Tested that a new desired reference pose is sought
{
  ros::NodeHandle n;
  ros::Duration timeout(5);
  ros::Publisher pose_publisher = n.advertise<geometry_msgs::PoseStamped>(ctrl_name +"/reference_pose", 500);
  ros::Duration(0.5).sleep(); // C++ publisher need some time to start up

  const std::string controller_state {ctrl_name + "/controller_state"};
  auto msg_controller_up = ros::topic::waitForMessage<cartesian_impedance_controller::ControllerState>(controller_state, n, timeout);
  ASSERT_TRUE(msg_controller_up != nullptr) << "Controller is not up.";
  EXPECT_NEAR(std::abs(msg_controller_up->current_pose.position.y), 0.3, 0.05);

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.pose.position.x = 0.4;
  pose_msg.pose.position.y = 0.0;
  pose_msg.pose.position.z = 0.14;
  pose_msg.pose.orientation.x = 0.0;
  pose_msg.pose.orientation.y = 1.0;
  pose_msg.pose.orientation.z = 0.0;
  pose_msg.pose.orientation.w = 0.0;
  pose_publisher.publish(pose_msg);
  ros::Duration(5).sleep();

  for (int i = 0; i < 5; i++) {
    auto msg_state = ros::topic::waitForMessage<cartesian_impedance_controller::ControllerState>(controller_state, n, timeout);
    ASSERT_TRUE(msg_state != nullptr) << "Did not receive message on iteration " << i + 1;
    EXPECT_NEAR(std::abs(msg_state->reference_pose.position.y), 0.0, 0.11);
    EXPECT_NEAR(std::abs(msg_state->current_pose.position.y), 0.0, 0.15);
      ros::Duration(0.1).sleep();
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ros");
  return RUN_ALL_TESTS();
}

