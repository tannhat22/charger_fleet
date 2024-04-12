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

#include "utilities.hpp"

namespace charger_fleet
{
namespace ros2
{

void to_cf_message(
    const charger_fleet_msgs::msg::ChargerRequest& _in_msg, 
    messages::ChargerRequest& _out_msg)
{
  _out_msg.charger_name = _in_msg.charger_name;
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.robot_name = _in_msg.robot_name;
  _out_msg.request_id = _in_msg.request_id;
  _out_msg.charger_mode.mode = _in_msg.charger_mode.mode;
}

void to_ros_message(
    const messages::ChargerState& _in_msg,
    charger_fleet_msgs::msg::ChargerState& _out_msg)
{
  _out_msg.state = _in_msg.state;
  _out_msg.charger_name = _in_msg.charger_name;
  _out_msg.error_message = _in_msg.error_message;
  _out_msg.request_id = _in_msg.request_id;
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.robot_name = _in_msg.robot_name;
}

} // namespace ros2
} // namespace charger_fleet
