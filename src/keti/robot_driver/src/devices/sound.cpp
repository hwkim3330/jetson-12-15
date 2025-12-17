// Copyright 2025 Jetsonai CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Authors:Kate Kim

#include "rssaem_node/devices/sound.hpp"

#include <memory>
#include <string>

using jetsonai::rssaem::devices::Sound;

Sound::Sound(
  std::shared_ptr<rclcpp::Node> & nh,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper,
  const std::string & server_name)
: Devices(nh, dxl_sdk_wrapper)
{
  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create sound server");
  srv_ = nh_->create_service<rssaem_msgs::srv::Sound>(
    server_name,
    [this](
      const std::shared_ptr<rssaem_msgs::srv::Sound::Request> request,
      std::shared_ptr<rssaem_msgs::srv::Sound::Response> response) -> void
    {
      this->command(static_cast<void *>(request.get()), static_cast<void *>(response.get()));
    }
  );
}

void Sound::command(const void * request, void * response)
{
  rssaem_msgs::srv::Sound::Request req = *(rssaem_msgs::srv::Sound::Request *)request;
  rssaem_msgs::srv::Sound::Response * res = (rssaem_msgs::srv::Sound::Response *)response;

  res->success = dxl_sdk_wrapper_->set_data_to_device(
    extern_control_table.sound.addr,
    extern_control_table.sound.length,
    reinterpret_cast<uint8_t *>(&req.value),
    &res->message);
}

void Sound::request(
  rclcpp::Client<rssaem_msgs::srv::Sound>::SharedPtr client,
  rssaem_msgs::srv::Sound::Request req)
{
  auto request = std::make_shared<rssaem_msgs::srv::Sound::Request>(req);
  auto result = client->async_send_request(request);
}
