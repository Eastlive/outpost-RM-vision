// Copyright (c) 2023 Chen Tingxu
// Licensed under the MIT License.

#ifndef VIRTUAL_SERIAL_DRIVER__VIRTUAL_SERIAL_DRIVER_HPP_
#define VIRTUAL_SERIAL_DRIVER__VIRTUAL_SERIAL_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/float64.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "auto_aim_interfaces/msg/target_outpost.hpp"

namespace virtual_serial_driver
{

class VirtualSerialDriver : public rclcpp::Node
{
private:
  int count_;
  char enemy_color_;
  double bullet_speed_;
  double pitch_joint_;
  double yaw_joint_;
  // auto_aim_interfaces::msg::TargetOutpost target_msg_;

  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr enemy_color_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bullet_speed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  // rclcpp::Subscription<auto_aim_interfaces::msg::TargetOutpost>::SharedPtr target_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();

public:
  explicit VirtualSerialDriver(const rclcpp::NodeOptions & options);
  ~VirtualSerialDriver() {}
};

} // virtual_serial_driver

#endif  // VIRTUAL_SERIAL_DRIVER__VIRTUAL_SERIAL_DRIVER_HPP_
