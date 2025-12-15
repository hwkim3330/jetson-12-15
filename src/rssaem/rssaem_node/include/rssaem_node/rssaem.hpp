// Copyright 2025 Jetsonai CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef RSSAEM_NODE__RSSAEM_HPP_
#define RSSAEM_NODE__RSSAEM_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
//IMU #include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rssaem_msgs/msg/sensor_state.hpp>

#include "rssaem_node/control_table.hpp"
#include "rssaem_node/devices/devices.hpp"
#include "rssaem_node/devices/motor_power.hpp"
#include "rssaem_node/devices/reset.hpp"
#include "rssaem_node/devices/sound.hpp"
#include "rssaem_node/dynamixel_sdk_wrapper.hpp"
#include "rssaem_node/odometry.hpp"
#include "rssaem_node/sensors/battery_state.hpp"
//IMU #include "rssaem_node/sensors/imu.hpp"
#include "rssaem_node/sensors/joint_state.hpp"
#include "rssaem_node/sensors/sensor_state.hpp"
#include "rssaem_node/sensors/sensors.hpp"
#include "rssaem_node/twist_subscriber.hpp"

namespace jetsonai
{
namespace rssaem
{
extern const ControlTable extern_control_table;
class RSsaem : public rclcpp::Node
{
public:
  typedef struct
  {
    float separation;
    float radius;
  } Wheels;

  typedef struct
  {
    float profile_acceleration_constant;
    float profile_acceleration;
  } Motors;

  explicit RSsaem(const std::string & usb_port);
  virtual ~RSsaem() {}

  Wheels * get_wheels();
  Motors * get_motors();

private:
  void init_dynamixel_sdk_wrapper(const std::string & usb_port);
  void check_device_status();

  void add_sensors();
  void add_devices();
  void add_motors();
  void add_wheels();

  void run();

  void publish_timer(const std::chrono::milliseconds timeout);
  void heartbeat_timer(const std::chrono::milliseconds timeout);

  void cmd_vel_callback();
  void parameter_event_callback();

  Wheels wheels_;
  Motors motors_;

  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;

  std::list<sensors::Sensors *> sensors_;
  std::map<std::string, devices::Devices *> devices_;

  std::unique_ptr<Odometry> odom_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::unique_ptr<TwistSubscriber> cmd_vel_sub_;

  rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};
}  // namespace rssaem
}  // namespace jetsonai
#endif  // RSSAEM_NODE__RSSAEM_HPP_
