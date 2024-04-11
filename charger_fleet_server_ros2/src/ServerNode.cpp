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

#include <chrono>

#include <Eigen/Geometry>

#include <charger_fleet/Server.hpp>
#include <charger_fleet/ServerConfig.hpp>

#include <charger_fleet/messages/ModeRequest.hpp>

#include "utilities.hpp"
#include "ServerNode.hpp"

namespace charger_fleet
{
namespace ros2
{

ServerNode::SharedPtr ServerNode::make(
    const ServerNodeConfig& _config, const rclcpp::NodeOptions& _node_options)
{
  // Starting the charger fleet server node
  SharedPtr server_node(new ServerNode(_config, _node_options));

  auto start_time = std::chrono::steady_clock::now();
  auto end_time = std::chrono::steady_clock::now();
  while (
      std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time)
          .count() < 10)
  {
    rclcpp::spin_some(server_node);

    server_node->setup_config();
    if (server_node->is_ready())
      break;
    RCLCPP_INFO(
        server_node->get_logger(), "waiting for configuration parameters.");

    end_time = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  if (!server_node->is_ready())
  {
    RCLCPP_ERROR(
        server_node->get_logger(), "unable to initialize parameters.");
    return nullptr;
  }
  server_node->print_config();

  // Starting the charger fleet server
  ServerConfig server_config =
      server_node->server_node_config.get_server_config();
  Server::SharedPtr server = Server::make(server_config);
  if (!server)
    return nullptr;

  server_node->start(Fields{
    std::move(server)
  });

  return server_node;
}

ServerNode::~ServerNode()
{}

ServerNode::ServerNode(
    const ServerNodeConfig& _config,
    const rclcpp::NodeOptions& _node_options) :
  Node(_config.fleet_name + "_node", _node_options),
  server_node_config(_config)
{}

void ServerNode::print_config()
{
  server_node_config.print_config();
}

void ServerNode::setup_config()
{
  get_parameter("fleet_name", server_node_config.fleet_name);
  get_parameter("fleet_charger_state_topic", server_node_config.fleet_state_topic);
  get_parameter("mode_request_topic", server_node_config.mode_request_topic);
  // get_parameter("charge_request_topic", server_node_config.charge_request_topic);
  // get_parameter("cancel_request_topic", server_node_config.cancel_request_topic);
  get_parameter("dds_domain", server_node_config.dds_domain);
  get_parameter("dds_charger_state_topic", server_node_config.dds_charger_state_topic);
  get_parameter("dds_mode_request_topic", server_node_config.dds_mode_request_topic);
  // get_parameter("dds_charge_request_topic", server_node_config.dds_charge_request_topic);
  // get_parameter("dds_cancel_request_topic", server_node_config.dds_cancel_request_topic);
  get_parameter("update_state_frequency", server_node_config.update_state_frequency);
  get_parameter("publish_state_frequency", server_node_config.publish_state_frequency);
}

bool ServerNode::is_ready()
{
  if (server_node_config.fleet_name == "fleet_name")
    return false;
  return true;
}

void ServerNode::start(Fields _fields)
{
  fields = std::move(_fields);

  {
    WriteLock charger_states_lock(charger_states_mutex);
    charger_states.clear();
  }

  using namespace std::chrono_literals;

  // --------------------------------------------------------------------------
  // First callback group that handles getting updates from all the clients
  // available

  update_state_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  update_state_timer = create_wall_timer(
      100ms, std::bind(&ServerNode::update_state_callback, this),
      update_state_callback_group);

  // --------------------------------------------------------------------------
  // Second callback group that handles publishing fleet states to RMF, and
  // handling requests from RMF to be sent down to the clients

  fleet_state_pub_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  fleet_state_pub =
      create_publisher<charger_fleet_msgs::msg::FleetChargerState>(
          server_node_config.fleet_state_topic, 10);

  fleet_state_pub_timer = create_wall_timer(
      std::chrono::seconds(1) / server_node_config.publish_state_frequency,
      std::bind(&ServerNode::publish_fleet_state, this),
      fleet_state_pub_callback_group);

  // --------------------------------------------------------------------------
  // Mode request handling

  auto mode_request_sub_opt = rclcpp::SubscriptionOptions();

  mode_request_sub_opt.callback_group = fleet_state_pub_callback_group;

  mode_request_sub = create_subscription<charger_fleet_msgs::msg::ModeRequest>(
      server_node_config.mode_request_topic, rclcpp::QoS(10),
      [&](charger_fleet_msgs::msg::ModeRequest::UniquePtr msg)
      {
        handle_mode_request(std::move(msg));
      },
      mode_request_sub_opt);
}

bool ServerNode::is_request_valid(
    const std::string& _fleet_name, const std::string& _charger_name)
{
  if (_fleet_name != server_node_config.fleet_name)
    return false;

  ReadLock charger_states_lock(charger_states_mutex);
  auto it = charger_states.find(_charger_name);
  if (it == charger_states.end())
    return false;
  return true;
}

void ServerNode::handle_mode_request(
    charger_fleet_msgs::msg::ModeRequest::UniquePtr _msg)
{
  messages::ModeRequest cf_msg;
  to_cf_message(*(_msg.get()), cf_msg);
  fields.server->send_mode_request(cf_msg);
}

void ServerNode::update_state_callback()
{
  std::vector<messages::ChargerState> new_charger_states;
  fields.server->read_charger_states(new_charger_states);

  for (const messages::ChargerState& cf_cs : new_charger_states)
  {
    charger_fleet_msgs::msg::ChargerState ros_cs;
    to_ros_message(cf_cs, ros_cs);

    WriteLock charger_states_lock(charger_states_mutex);
    auto it = charger_states.find(ros_cs.name);
    if (it == charger_states.end())
      RCLCPP_INFO(
          get_logger(),
          "registered a new charger: [%s]",
          ros_cs.name.c_str());

    charger_states[ros_cs.name] = ros_cs;
  }
}

void ServerNode::publish_fleet_state()
{
  charger_fleet_msgs::msg::FleetChargerState fleet_state;
  fleet_state.name = server_node_config.fleet_name;
  fleet_state.chargers.clear();

  ReadLock charger_states_lock(charger_states_mutex);
  for (const auto it : charger_states)
  {
    const auto fleet_frame_rs = it.second;
    charger_fleet_msgs::msg::ChargerState rmf_frame_cs;

    rmf_frame_cs.name = fleet_frame_rs.name;
    rmf_frame_cs.model = fleet_frame_rs.model;
    rmf_frame_cs.task_id = fleet_frame_rs.task_id;
    rmf_frame_cs.mode = fleet_frame_rs.mode;

    fleet_state.chargers.push_back(rmf_frame_cs);
  }
  fleet_state_pub->publish(fleet_state);
}

} // namespace ros2
} // namespace charger_fleet
