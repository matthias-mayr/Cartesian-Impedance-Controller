#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cartesian_impedance_controller/msg/controller_state.hpp>

// Controller node name as set in the controller manager configuration.
// Adjust to match your deployment if it differs.
const std::string ctrl_name = "CartesianImpedance_trajectory_controller";

// Node shared across all tests in this translation unit.
rclcpp::Node::SharedPtr test_node;

// Wait until the controller's publisher is discovered by the local DDS participant.
// Without this, wait_for_message may time out before DDS discovery completes.
static void wait_for_publisher(
  const std::string & topic,
  std::chrono::seconds timeout = std::chrono::seconds(15))
{
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (test_node->count_publishers(topic) == 0) {
    rclcpp::spin_some(test_node);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    if (std::chrono::steady_clock::now() > deadline) { break; }
  }
}

// ROSFunctionalityTests verify the controller's runtime behaviour.

TEST(ROSFunctionalityTests, forceTests)  // Tests that a desired external force is applied.
{
  const std::string controller_state_topic{"/" + ctrl_name + "/controller_state"};
  const std::string wrench_topic{"/" + ctrl_name + "/set_cartesian_wrench"};

  auto wrench_publisher =
      test_node->create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic, 10);

  // Wait for the publisher to be discovered before sending any messages.
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Wait for DDS publisher discovery, then check the controller is up.
  wait_for_publisher(controller_state_topic);
  cartesian_impedance_controller::msg::ControllerState initial_state;
  bool up =
      rclcpp::wait_for_message(initial_state, test_node, controller_state_topic,
                               std::chrono::seconds(5));
  ASSERT_TRUE(up) << "Controller is not publishing on " << controller_state_topic;
  EXPECT_DOUBLE_EQ(std::abs(initial_state.commanded_wrench.force.x), 0.0);

  // Command a 5 N force along the x-axis in the world frame.
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header.frame_id = "world";
  wrench_msg.header.stamp = test_node->now();
  wrench_msg.wrench.force.x = 5.0;
  wrench_publisher->publish(wrench_msg);
  rclcpp::sleep_for(std::chrono::seconds(3));

  for (int i = 0; i < 5; i++) {
    cartesian_impedance_controller::msg::ControllerState state;
    bool received =
        rclcpp::wait_for_message(state, test_node, controller_state_topic,
                                 std::chrono::seconds(5));
    ASSERT_TRUE(received) << "Did not receive controller state on iteration " << i + 1;
    EXPECT_GE(std::abs(state.commanded_wrench.force.x), 1.0);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    // check commanded torques are non-zero to verify the wrench is being applied to the robot
    EXPECT_GE(std::abs(state.commanded_torques[0]), 0.3);
  }

  // Cancel the wrench so subsequent tests start from a clean state.
  wrench_msg.header.stamp = test_node->now();
  wrench_msg.wrench.force.x = 0.0;
  wrench_publisher->publish(wrench_msg);
  rclcpp::sleep_for(std::chrono::seconds(1));
}

TEST(ROSFunctionalityTests, motionTests)  // Tests that a new reference pose is accepted.
{
  const std::string controller_state_topic{"/" + ctrl_name + "/controller_state"};
  const std::string pose_topic{"/" + ctrl_name + "/reference_pose"};

  auto pose_publisher =
      test_node->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

  // Wait for the publisher to be discovered before sending any messages.
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Wait for DDS publisher discovery, then verify the controller is up.
  wait_for_publisher(controller_state_topic);
  cartesian_impedance_controller::msg::ControllerState initial_state;
  bool up =
      rclcpp::wait_for_message(initial_state, test_node, controller_state_topic,
                               std::chrono::seconds(5));
  ASSERT_TRUE(up) << "Controller is not publishing on " << controller_state_topic;
  EXPECT_NEAR(std::abs(initial_state.current_pose.position.y), 0.3, 0.05);

  // Command a new reference pose with y = 0.
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp = test_node->now();
  pose_msg.pose.position.x = 0.4;
  pose_msg.pose.position.y = 0.0;
  pose_msg.pose.position.z = 0.14;
  pose_msg.pose.orientation.x = 0.0;
  pose_msg.pose.orientation.y = 1.0;
  pose_msg.pose.orientation.z = 0.0;
  pose_msg.pose.orientation.w = 0.0;
  pose_publisher->publish(pose_msg);
  rclcpp::sleep_for(std::chrono::seconds(2));

  for (int i = 0; i < 5; i++) {
    cartesian_impedance_controller::msg::ControllerState state;
    bool received =
        rclcpp::wait_for_message(state, test_node, controller_state_topic,
                                 std::chrono::seconds(5));
    ASSERT_TRUE(received) << "Did not receive controller state on iteration " << i + 1;
    // Verify the controller accepted and stored the new reference pose.
    EXPECT_NEAR(std::abs(state.reference_pose.position.y), 0.0, 0.11);
    // Note: current_pose does not change with mock hardware because the mock
    // backend mirrors effort commands to effort states but does not integrate
    // torques to joint positions.
    EXPECT_GE(std::abs(state.commanded_torques[0]), 0.3);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  test_node = rclcpp::Node::make_shared("test_ros_func_node");
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  test_node.reset();  // destroy node before shutdown to avoid use-after-free on DDS context
  rclcpp::shutdown();
  return result;
}

