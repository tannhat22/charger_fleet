/*
 * Copyright (C) 2019 Open Source chargerics Foundation
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

#include <iostream>
#include <exception>
#include <thread>

#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <charger_fleet_msgs/srv/charger.hpp>

#include "charger_fleet/ros2/client_node.hpp"
#include "charger_fleet/ros2/client_node_config.hpp"

namespace charger_fleet
{
namespace ros2
{
ClientNode::ClientNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("charger_fleet_client_ros2", options)
{
  /// Starting the charger fleet client
  RCLCPP_INFO(get_logger(), "Greetings from %s", get_name());

  // parameter declarations
  declare_parameter("fleet_name", client_node_config.fleet_name);
  declare_parameter("charger_name", client_node_config.charger_name);
  declare_parameter("charger_model", client_node_config.charger_model);
  // defaults declared in header
  declare_parameter("charger_mode_topic", client_node_config.charger_mode_topic);
  declare_parameter("charging_trigger_server_name", client_node_config.charging_trigger_server_name);
  declare_parameter("dds_domain", client_node_config.dds_domain);
  declare_parameter("dds_mode_request_topic", client_node_config.dds_mode_request_topic);
  declare_parameter("wait_timeout", client_node_config.wait_timeout);
  declare_parameter("update_frequency", client_node_config.update_frequency);
  declare_parameter("publish_frequency", client_node_config.publish_frequency);

  // getting new values for parameters or keep defaults
  get_parameter("fleet_name", client_node_config.fleet_name);
  get_parameter("charger_name", client_node_config.charger_name);
  get_parameter("charger_model", client_node_config.charger_model);
  get_parameter("charger_mode_topic", client_node_config.charger_mode_topic);
  get_parameter("charging_trigger_server_name", client_node_config.charging_trigger_server_name);
  get_parameter("dds_domain", client_node_config.dds_domain);
  get_parameter("dds_mode_request_topic", client_node_config.dds_mode_request_topic);
  get_parameter("wait_timeout", client_node_config.wait_timeout);
  get_parameter("update_frequency", client_node_config.update_frequency);
  get_parameter("publish_frequency", client_node_config.publish_frequency);
  print_config();

  ClientConfig client_config = client_node_config.get_client_config();
  Client::SharedPtr client = Client::make(client_config);
  if (!client) {
    throw std::runtime_error("Unable to create charger_fleet Client from config.");
  }

  /// Setting up the charging server client, if required, wait for server
  rclcpp::Client<charger_fleet_msgs::srv::Charger>::SharedPtr charging_trigger_client = nullptr;
  if (client_node_config.charging_trigger_server_name != "") {
    charging_trigger_client = create_client<charger_fleet_msgs::srv::Charger>(
      client_node_config.charging_trigger_server_name);
    RCLCPP_INFO(
      get_logger(), "waiting for connection with trigger server: %s",
      client_node_config.charging_trigger_server_name.c_str());
    while (!charging_trigger_client->wait_for_service(
        std::chrono::duration<double>(client_node_config.wait_timeout)))
    {
      RCLCPP_ERROR(
        get_logger(), "timed out waiting for charging trigger server: %s",
        client_node_config.charging_trigger_server_name.c_str());
      if (!rclcpp::ok()) {
        throw std::runtime_error("exited rclcpp while constructing client_node");
      }
    }
  }

  start(
    Fields{
        std::move(client),
        std::move(charging_trigger_client)
      });
}

ClientNode::~ClientNode()
{
}

void ClientNode::start(Fields _fields)
{
  fields = std::move(_fields);

  charger_mode_sub = create_subscription<charger_fleet_msgs::msg::ChargerMode>(
    client_node_config.charger_mode_topic, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ClientNode::charger_state_callback_fn, this, std::placeholders::_1));

  request_error = false;

  RCLCPP_INFO(get_logger(), "starting update timer.");
  std::chrono::duration<double> update_period =
    std::chrono::duration<double>(1.0 / client_node_config.update_frequency);
  update_timer = create_wall_timer(update_period, std::bind(&ClientNode::update_fn, this));

  RCLCPP_INFO(get_logger(), "starting publish timer.");
  std::chrono::duration<double> publish_period =
    std::chrono::duration<double>(1.0 / client_node_config.publish_frequency);
  publish_timer = create_wall_timer(publish_period, std::bind(&ClientNode::publish_fn, this));
}

void ClientNode::print_config()
{
  client_node_config.print_config();
}

void ClientNode::charger_state_callback_fn(
  const charger_fleet_msgs::msg::ChargerMode::SharedPtr _msg)
{
  WriteLock charger_state_lock(charger_state_mutex);
  current_charger_state = *_msg;
}


messages::ChargerMode ClientNode::get_charger_mode()
{
  /// Checks if charger has just received a request that causes an adapter error
  if (request_error) {
    return messages::ChargerMode{messages::ChargerMode::MODE_REQUEST_ERROR};
  }

  /// Checks if charger is charging
  {
    ReadLock charger_state_lock(charger_state_mutex);

    if (current_charger_state.mode ==
      current_charger_state.MODE_IDLE)
    {
      return messages::ChargerMode{messages::ChargerMode::MODE_IDLE};
    } else if (current_charger_state.mode ==
             current_charger_state.MODE_CHARGING)
    {
      return messages::ChargerMode{messages::ChargerMode::MODE_CHARGING};
    }  else if (current_charger_state.mode ==
             current_charger_state.MODE_CHARGEFULL)
    {
      return messages::ChargerMode{messages::ChargerMode::MODE_CHARGEFULL};
    }  else if (current_charger_state.mode ==
             current_charger_state.MODE_ERROR)
    {
      return messages::ChargerMode{messages::ChargerMode::MODE_ERROR};
    }  else if (current_charger_state.mode ==
             current_charger_state.MODE_REQUEST_ERROR)
    {
      return messages::ChargerMode{messages::ChargerMode::MODE_REQUEST_ERROR};
    }
    
  }

  return messages::ChargerMode{messages::ChargerMode::MODE_IDLE};
}

void ClientNode::publish_charger_state()
{
  messages::ChargerState new_charger_state;
  new_charger_state.name = client_node_config.charger_name;
  new_charger_state.model = client_node_config.charger_model;

  {
    ReadLock task_id_lock(task_id_mutex);
    new_charger_state.task_id = current_task_id;
  }

  new_charger_state.mode = get_charger_mode();

  if (!fields.client->send_charger_state(new_charger_state)) {
    RCLCPP_WARN(get_logger(), "failed to send charger state");
  }
}

bool ClientNode::is_valid_request(
  const std::string & _request_fleet_name,
  const std::string & _request_charger_name,
  const std::string & _request_task_id)
{
  ReadLock task_id_lock(task_id_mutex);
  if (current_task_id == _request_task_id ||
    client_node_config.charger_name != _request_charger_name ||
    client_node_config.fleet_name != _request_fleet_name)
  {
    return false;
  }
  return true;
}

bool ClientNode::read_mode_request()
{
  messages::ModeRequest mode_request;
  if (fields.client->read_mode_request(mode_request) &&
      is_valid_request(
          mode_request.fleet_name, mode_request.charger_name,
          mode_request.task_id))
  {
    if ((mode_request.mode.mode == messages::ChargeMode::MODE_CHARGE) ||
        (mode_request.mode.mode == messages::ChargeMode::MODE_UNCHARGE) )
    {
      if (mode_request.mode.mode == messages::ChargeMode::MODE_CHARGE) {
        RCLCPP_INFO(get_logger(), "received a CHARGE command.");
      } else {
        RCLCPP_INFO(get_logger(), "received a UNCHARGE command.");
      }
      
      if (fields.charging_trigger_client &&
        fields.charging_trigger_client->service_is_ready())
      {
        using ServiceResponseFuture =
          rclcpp::Client<charger_fleet_msgs::srv::Charger>::SharedFuture;
        auto response_received_callback = [&](ServiceResponseFuture future) {
          auto response = future.get();
          if (!response->success)
          {
            RCLCPP_ERROR(get_logger(), "Failed to request mode, message: %s!",
              response->message.c_str());
            request_error = true;
          } else {
            request_error = false;
          }
        };
        auto charge_srv = std::make_shared<charger_fleet_msgs::srv::Charger::Request>();
        charge_srv->mode = mode_request.mode.mode;

        // sync call would block indefinelty as we are in a spinning node
        fields.charging_trigger_client->async_send_request(charge_srv, response_received_callback);
      }

    } else {
      RCLCPP_ERROR(get_logger(), "received an INVALID/UNSUPPORTED command: %d.",
              mode_request.mode.mode);
      request_error = true;
    }
    
    WriteLock task_id_lock(task_id_mutex);
    current_task_id = mode_request.task_id;

    return true;
  }
  return false;
}

void ClientNode::read_requests()
{
  if (read_mode_request())
  {
    return;
  }
}

void ClientNode::handle_requests()
{
}

void ClientNode::update_fn()
{
  read_requests();
}

void ClientNode::publish_fn()
{
  publish_charger_state();
}

} // namespace ros2
} // namespace charger_fleet
