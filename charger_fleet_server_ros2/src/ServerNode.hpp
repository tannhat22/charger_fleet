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

#ifndef CHARGER_FLEET_SERVER_ROS2__SRC__SERVERNODE_HPP
#define CHARGER_FLEET_SERVER_ROS2__SRC__SERVERNODE_HPP

#include <mutex>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>

#include <charger_fleet_msgs/msg/charger_state.hpp>
#include <charger_fleet_msgs/msg/fleet_charger_state.hpp>
#include <charger_fleet_msgs/msg/mode_request.hpp>

#include <charger_fleet/Server.hpp>
#include <charger_fleet/messages/ChargerState.hpp>

#include "ServerNodeConfig.hpp"

namespace charger_fleet
{
namespace ros2
{

class ServerNode : public rclcpp::Node
{
public:

  using SharedPtr = std::shared_ptr<ServerNode>;
  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  static SharedPtr make(
      const ServerNodeConfig& config,
      const rclcpp::NodeOptions& options =
          rclcpp::NodeOptions()
              .allow_undeclared_parameters(true)
              .automatically_declare_parameters_from_overrides(true));

  ~ServerNode();

  struct Fields
  {
    // Free fleet server
    Server::SharedPtr server;
  };

  void print_config();

private:

  bool is_request_valid(
      const std::string& fleet_name, const std::string& charger_name);

  // --------------------------------------------------------------------------

  rclcpp::Subscription<charger_fleet_msgs::msg::ModeRequest>::SharedPtr
      mode_request_sub;

  void handle_mode_request(charger_fleet_msgs::msg::ModeRequest::UniquePtr msg);

  // --------------------------------------------------------------------------

  rclcpp::CallbackGroup::SharedPtr update_state_callback_group;

  rclcpp::TimerBase::SharedPtr update_state_timer;

  std::mutex charger_states_mutex;

  std::unordered_map<std::string, charger_fleet_msgs::msg::ChargerState>
      charger_states;

  void update_state_callback();

  // --------------------------------------------------------------------------

  rclcpp::CallbackGroup::SharedPtr
      fleet_state_pub_callback_group;

  rclcpp::TimerBase::SharedPtr fleet_state_pub_timer;

  rclcpp::Publisher<charger_fleet_msgs::msg::FleetChargerState>::SharedPtr
      fleet_state_pub;

  void publish_fleet_state();

  // --------------------------------------------------------------------------

  ServerNodeConfig server_node_config;

  void setup_config();

  bool is_ready();

  Fields fields;

  ServerNode(
      const ServerNodeConfig& config, const rclcpp::NodeOptions& options);

  void start(Fields fields);

};

} // namespace ros2
} // namespace charger_fleet

#endif // CHARGER_FLEET_SERVER_ROS2__SRC__SERVERNODE_HPP
