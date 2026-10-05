#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include "gtest/gtest.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

class PiperHardwareTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<rclcpp::Node>("piper_hardware_test");
    js = node->create_publisher<sensor_msgs::msg::JointState>("/test/feedback", 1);
    health = node->create_publisher<std_msgs::msg::Bool>("/test/ready", 1);
    loader = std::make_unique<pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
      "hardware_interface", "hardware_interface::SystemInterface");
    hw = loader->createSharedInstance("piper_hardware/PiperSystem");
    const std::string xml = R"(
<robot name="test"><link name="base"/><link name="tip"/>
<joint name="joint1" type="revolute"><parent link="base"/><child link="tip"/><limit lower="-3" upper="3" effort="10" velocity="3"/></joint>
<ros2_control name="TestPiper" type="system"><hardware><plugin>piper_hardware/PiperSystem</plugin>
<param name="feedback_topic">/test/feedback</param><param name="ready_topic">/test/ready</param>
<param name="command_topic">/test/command</param><param name="stop_topic">/test/stop</param>
<param name="feedback_timeout">0.15</param><param name="startup_timeout">0.6</param></hardware>
<joint name="joint1"><command_interface name="position"/><state_interface name="position"/><state_interface name="velocity"/></joint>
</ros2_control></robot>)";
    ASSERT_EQ(hw->on_init(hardware_interface::parse_control_resources_from_urdf(xml).at(0)), CallbackReturn::SUCCESS);
    states = hw->export_state_interfaces(); commands = hw->export_command_interfaces();
    running = true;
    thread = std::thread([this]() {
      while (running) {
        if (publish) {
          sensor_msgs::msg::JointState m;
          m.header.stamp = node->now() - rclcpp::Duration::from_seconds(old_stamp ? 1.0 : 0.0);
          m.name = {invalid ? "wrong_joint" : "joint1"}; m.position = {0.4}; m.velocity = {0.0};
          js->publish(m);
          std_msgs::msg::Bool r; r.data = ready; health->publish(r);
        }
        std::this_thread::sleep_for(5ms);
      }
    });
    std::this_thread::sleep_for(150ms);
  }
  void TearDown() override
  {
    running = false;
    if (thread.joinable()) {thread.join();}
    hw.reset(); loader.reset(); node.reset();
  }
  return_type read() {return hw->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005));}
  return_type write() {return hw->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005));}
  std::atomic<bool> running{false}, publish{true}, ready{true}, invalid{false}, old_stamp{false};
  std::thread thread;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health;
  std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> loader;
  std::shared_ptr<hardware_interface::SystemInterface> hw;
  std::vector<hardware_interface::StateInterface> states;
  std::vector<hardware_interface::CommandInterface> commands;
};

TEST_F(PiperHardwareTest, SeedsMeasuredPoseAndDoesNotMirrorCommands)
{
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(commands[0].get_value(), 0.4);
  commands[0].set_value(0.9);
  EXPECT_EQ(write(), return_type::OK);
  EXPECT_EQ(read(), return_type::OK);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 0.4);
}
TEST_F(PiperHardwareTest, LostFeedbackFailsClosed)
{
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  publish = false; std::this_thread::sleep_for(220ms);
  EXPECT_EQ(read(), return_type::ERROR);
  EXPECT_EQ(write(), return_type::ERROR);
}
TEST_F(PiperHardwareTest, ReplayedOldStampAndMissingJointAreRejected)
{
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  old_stamp = true; std::this_thread::sleep_for(60ms);
  EXPECT_EQ(read(), return_type::ERROR);
  old_stamp = false; invalid = true;
  EXPECT_EQ(hw->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}
TEST_F(PiperHardwareTest, DriverFaultFailsClosed)
{
  ASSERT_EQ(hw->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ready = false; std::this_thread::sleep_for(60ms);
  EXPECT_EQ(read(), return_type::ERROR);
}
