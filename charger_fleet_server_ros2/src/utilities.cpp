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
    const charger_fleet_msgs::msg::ModeRequest& _in_msg, 
    messages::ModeRequest& _out_msg)
{
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.charger_name = _in_msg.charger_name;
  _out_msg.mode.mode = _in_msg.mode.mode;
  _out_msg.task_id = _in_msg.task_id;
}

void to_ros_message(
    const messages::ChargerState& _in_msg,
    charger_fleet_msgs::msg::ChargerState& _out_msg)
{
  _out_msg.name = _in_msg.name;
  _out_msg.model = _in_msg.model;
  _out_msg.task_id = _in_msg.task_id;
  _out_msg.mode.mode = _in_msg.mode.mode;
}

} // namespace ros2
} // namespace charger_fleet
