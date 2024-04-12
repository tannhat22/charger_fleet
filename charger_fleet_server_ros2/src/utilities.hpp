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

#ifndef CHARGER_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP
#define CHARGER_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP

#include <charger_fleet_msgs/msg/charger_state.hpp>
#include <charger_fleet_msgs/msg/charger_request.hpp>

#include <charger_fleet/messages/ChargerState.hpp>
#include <charger_fleet/messages/ChargerRequest.hpp>

namespace charger_fleet
{
namespace ros2
{

void to_cf_message(
    const charger_fleet_msgs::msg::ChargerRequest& in_msg, 
    messages::ChargerRequest& out_msg);

// ----------------------------------------------------------------------------

void to_ros_message(
    const messages::ChargerState& in_msg,
    charger_fleet_msgs::msg::ChargerState& out_msg);

} // namespace ros2
} // namespace charger_fleet


#endif // CHARGER_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP
