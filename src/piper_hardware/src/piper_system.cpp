// Copyright 2026 Crobotic Solutions
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

namespace piper_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using Clock = std::chrono::steady_clock;

class PiperSystem : public hardware_interface::SystemInterface
{
public:
  ~PiperSystem() override
  {
    if (executor_) {executor_->cancel();}
    if (thread_.joinable()) {thread_.join();}
  }

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    const auto parameter = [&](const std::string & name, const std::string & fallback) {
        auto it = info_.hardware_parameters.find(name);
        return it == info_.hardware_parameters.end() ? fallback : it->second;
      };
    try {
      timeout_ = std::stod(parameter("feedback_timeout", "0.25"));
      startup_timeout_ = std::stod(parameter("startup_timeout", "5.0"));
    } catch (...) {return CallbackReturn::ERROR;}
    if (!std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(startup_timeout_) ||
      startup_timeout_ <= 0) {return CallbackReturn::ERROR;}
    for (const auto & joint : info_.joints) {
      names_.push_back(joint.name);
      bool has_position = false;
      for (const auto & state : joint.state_interfaces) {
        if (state.name == "position") {has_position = true;}
        else if (state.name != "velocity") {return CallbackReturn::ERROR;}
      }
      if (!has_position || joint.command_interfaces.size() > 1) {return CallbackReturn::ERROR;}
      if (!joint.command_interfaces.empty() && joint.command_interfaces[0].name != "position") {
        return CallbackReturn::ERROR;
      }
    }
    if (names_.empty()) {return CallbackReturn::ERROR;}
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    position_.assign(names_.size(), nan);
    velocity_.assign(names_.size(), nan);
    command_.assign(names_.size(), nan);
    measured_position_ = position_;
    measured_velocity_ = velocity_;
    node_ = std::make_shared<rclcpp::Node>(
      "piper_hardware_" + info_.name, rclcpp::NodeOptions().use_global_arguments(false));
    const auto qos = rclcpp::QoS(1).reliable();
    command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      parameter("command_topic", "/piper/joint_commands"), qos);
    stop_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      parameter("stop_topic", "/piper/stop"), qos);
    feedback_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      parameter("feedback_topic", "/joint_states"), qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->position.size() != msg->name.size() || msg->velocity.size() != msg->name.size()) {
          valid_feedback_ = false; return;
        }
        auto p = measured_position_;
        auto v = measured_velocity_;
        for (size_t i = 0; i < names_.size(); ++i) {
          auto it = std::find(msg->name.begin(), msg->name.end(), names_[i]);
          if (it == msg->name.end() || std::count(msg->name.begin(), msg->name.end(), names_[i]) != 1) {
            valid_feedback_ = false; return;
          }
          size_t j = std::distance(msg->name.begin(), it);
          if (!std::isfinite(msg->position[j]) || !std::isfinite(msg->velocity[j])) {
            valid_feedback_ = false; return;
          }
          p[i] = msg->position[j]; v[i] = msg->velocity[j];
        }
        double stamp = rclcpp::Time(msg->header.stamp).seconds();
        double age = node_->now().seconds() - stamp;
        if (stamp <= 0 || age < -0.01 || age > timeout_ || stamp < measurement_stamp_) {
          valid_feedback_ = false; return;
        }
        measured_position_ = p; measured_velocity_ = v;
        measurement_stamp_ = stamp; feedback_received_ = Clock::now(); valid_feedback_ = true;
      });
    ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      parameter("ready_topic", "/piper/hardware_ready"), qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        ready_ = msg->data; ready_received_ = Clock::now();
      });
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    thread_ = std::thread([this]() {executor_->spin();});
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> states;
    for (size_t i = 0; i < names_.size(); ++i) {
      for (const auto & s : info_.joints[i].state_interfaces) {
        states.emplace_back(names_[i], s.name, s.name == "position" ? &position_[i] : &velocity_[i]);
      }
    }
    return states;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> commands;
    for (size_t i = 0; i < names_.size(); ++i) {
      if (!info_.joints[i].command_interfaces.empty()) {
        commands.emplace_back(names_[i], "position", &command_[i]);
      }
    }
    return commands;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    const auto deadline = Clock::now() + std::chrono::duration<double>(startup_timeout_);
    while (rclcpp::ok() && Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (healthy()) {
          position_ = measured_position_; velocity_ = measured_velocity_;
          command_ = measured_position_;  // Never use YAML zero or a previous command.
          active_ = true; faulted_ = false;
          return CallbackReturn::SUCCESS;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    RCLCPP_ERROR(node_->get_logger(), "Activation requires complete fresh CAN feedback and healthy drivers");
    return CallbackReturn::ERROR;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    stop(); active_ = false;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    stop(); active_ = false;
    // Explicit reconfigure/activate is required; no automatic motion recovery.
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ && (!healthy() || faulted_)) {
      stop(); return return_type::ERROR;
    }
    if (valid_feedback_) {
      position_ = measured_position_; velocity_ = measured_velocity_;
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {return return_type::OK;}
    if (!healthy() || faulted_) {stop(); return return_type::ERROR;}
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    for (size_t i = 0; i < names_.size(); ++i) {
      if (info_.joints[i].command_interfaces.empty()) {continue;}
      if (!std::isfinite(command_[i])) {stop(); return return_type::ERROR;}
      msg.name.push_back(names_[i]); msg.position.push_back(command_[i]);
    }
    command_pub_->publish(msg);
    return return_type::OK;
  }

private:
  bool healthy() const  // Called with mutex held. Receipt AND source age are required.
  {
    auto elapsed = [](const auto & t) {return std::chrono::duration<double>(Clock::now() - t).count();};
    double age = node_->now().seconds() - measurement_stamp_;
    return ready_ && valid_feedback_ && elapsed(ready_received_) <= timeout_ &&
      elapsed(feedback_received_) <= timeout_ && age >= -0.01 && age <= timeout_;
  }
  void stop()
  {
    if (active_ && !faulted_) {
      faulted_ = true;
      stop_pub_->publish(std_msgs::msg::Empty());
      RCLCPP_ERROR(node_->get_logger(), "PiPER hardware stopped: deactivation, stale feedback or fault");
    }
  }
  std::vector<std::string> names_;
  std::vector<double> position_, velocity_, command_, measured_position_, measured_velocity_;
  std::mutex mutex_;
  double timeout_{0.25}, startup_timeout_{5.0}, measurement_stamp_{0};
  bool valid_feedback_{false}, ready_{false}, active_{false}, faulted_{false};
  Clock::time_point feedback_received_{}, ready_received_{};
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread thread_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
};
}  // namespace piper_hardware
PLUGINLIB_EXPORT_CLASS(piper_hardware::PiperSystem, hardware_interface::SystemInterface)
