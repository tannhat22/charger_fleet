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

#ifndef CHARGER_FLEET_SERVER_ROS2__SRC__SERVERNODECONFIG_HPP
#define CHARGER_FLEET_SERVER_ROS2__SRC__SERVERNODECONFIG_HPP

#include <string>

namespace charger_fleet
{
namespace ros2
{

struct ServerNodeConfig
{
  
  std::string fleet_name = "fleet_name";

  std::string fleet_state_topic = "fleet_charger_state";
  std::string charger_request_topic = "charger_request";

  int dds_domain = 52;
  std::string dds_charger_state_topic = "fleet_charger_state";
  std::string dds_charger_request_topic = "charger_request";

  double update_state_frequency = 10.0;
  double publish_state_frequency = 1.0;

  void print_config() const;

  ServerConfig get_server_config() const;

  static ServerNodeConfig make();

};

} // namespace ros2
} // namespace charger_fleet

#endif // CHARGER_FLEET_SERVER_ROS2__SRC__SERVERNODECONFIG_HPP
