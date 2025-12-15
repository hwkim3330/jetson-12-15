// Copyright 2025 Jetsonai CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#ifndef RSSAEM_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define RSSAEM_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rssaem_node/odometry.hpp"

namespace jetsonai
{
namespace rssaem
{
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  // namespace rssaem
}  // namespace jetsonai
#endif  // RSSAEM_NODE__DIFF_DRIVE_CONTROLLER_HPP_
