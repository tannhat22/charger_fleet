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

#ifndef CHARGER_FLEET__ROS2__CLIENTNODE_HPP
#define CHARGER_FLEET__ROS2__CLIENTNODE_HPP

#include <deque>
#include <shared_mutex>
#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <charger_fleet_msgs/srv/charger.hpp>
#include <charger_fleet_msgs/msg/charger_state.hpp>

#include <charger_fleet/Client.hpp>
#include <charger_fleet/ClientConfig.hpp>
#include <charger_fleet/messages/ChargerState.hpp>
#include <charger_fleet/messages/ChargerRequest.hpp>

#include <charger_fleet/Client.hpp>

#include "charger_fleet/ros2/client_node_config.hpp"

namespace charger_fleet
{
namespace ros2
{

class ClientNode : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<ClientNode>;
  using Mutex = std::mutex;
  using ReadLock = std::unique_lock<Mutex>;
  using WriteLock = std::unique_lock<Mutex>;

  explicit ClientNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ClientNode() override;

  struct Fields
  {
    /// Charger fleet client
    Client::SharedPtr client;

    // Charger server client
    rclcpp::Client<charger_fleet_msgs::srv::Charger>::SharedPtr charging_trigger_client;
  };

  void print_config();

private:
  // --------------------------------------------------------------------------
  // Chargermode handling

  rclcpp::Subscription<charger_fleet_msgs::msg::ChargerState>::SharedPtr  charger_state_sub;
  Mutex charger_state_mutex;
  charger_fleet_msgs::msg::ChargerState current_charger_state;
  void charger_state_callback_fn(const charger_fleet_msgs::msg::ChargerState::SharedPtr msg);

  // --------------------------------------------------------------------------
  // Mode handling

  // TODO: conditions to trigger emergency, however this is most likely for
  // indicating emergency within the fleet and not in RMF
  // TODO: figure out a better way to handle multiple triggered modes
  std::atomic<bool> request_error;

  messages::ChargerState get_charger_state();
  bool read_charger_request();

  // Request handling

  bool is_valid_request(
      const std::string& request_fleet_name,
      const std::string& request_charger_name,
      const std::string& request_request_id);

  Mutex request_id_mutex;
  std::string current_request_id;

  Mutex request_robot_mutex;
  std::string current_request_robot_name;


  void read_requests();
  void handle_requests();
  void publish_charger_state();

  // --------------------------------------------------------------------------
  // publish and update functions and timers

  std::shared_ptr<rclcpp::TimerBase> update_timer;
  std::shared_ptr<rclcpp::TimerBase> publish_timer;
  void update_fn();
  void publish_fn();

  // --------------------------------------------------------------------------

  ClientNodeConfig client_node_config;
  Fields fields;

  void start(Fields fields);
};

} // namespace ros2
} // namespace charger_fleet

#endif // CHARGER_FLEET__ROS2__CLIENTNODE_HPP