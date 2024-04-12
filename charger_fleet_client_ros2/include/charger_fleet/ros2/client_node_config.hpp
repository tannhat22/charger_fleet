/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef CHARGER_FLEET__ROS2__CLIENTNODECONFIG_HPP
#define CHARGER_FLEET__ROS2__CLIENTNODECONFIG_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <charger_fleet/ClientConfig.hpp>

namespace charger_fleet
{
namespace ros2
{

struct ClientNodeConfig
{

  std::string charger_name = "charger_name";
  std::string fleet_name = "fleet_name";

  std::string charger_state_topic = "/charger_state";
  std::string charging_trigger_server_name = "charger_server";

  int dds_domain = 52;
  std::string dds_state_topic = "charger_state";
  std::string dds_charger_request_topic = "charger_charger_request";

  double wait_timeout = 10.0;
  double update_frequency = 10.0;
  double publish_frequency = 1.0;

  void print_config() const;

  ClientConfig get_client_config() const;
};

} // namespace ros2
} // namespace charger_fleet

#endif // CHARGER_FLEET__ROS2__CLIENTNODECONFIG_HPP