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

#include <dds/dds.h>

#include "../dds_utils/common.hpp"

#include "message_utils.hpp"

namespace charger_fleet {
namespace messages {

void convert(const ChargerMode& _input, ChargerFleetData_ChargerMode& _output)
{
  // Consequently, charger fleet charger modes need to be ordered similarly as 
  // charger modes.
  _output.mode = _input.mode;
}

void convert(const ChargerFleetData_ChargerMode& _input, ChargerMode& _output)
{
  // Consequently, charger fleet charger modes need to be ordered similarly as 
  // charger modes.
  _output.mode = _input.mode;
}

void convert(const ChargerRequest& _input, ChargerFleetData_ChargerRequest& _output)
{
  _output.charger_name = common::dds_string_alloc_and_copy(_input.charger_name);
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.robot_name = common::dds_string_alloc_and_copy(_input.robot_name);
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
  convert(_input.charger_mode, _output.charger_mode);
}

void convert(const ChargerFleetData_ChargerRequest& _input, ChargerRequest& _output)
{
  _output.charger_name =  std::string(_input.charger_name);
  _output.fleet_name = std::string(_input.fleet_name);
  _output.robot_name = std::string(_input.robot_name);
  _output.request_id = std::string(_input.request_id);
  convert(_input.charger_mode, _output.charger_mode);
}

void convert(const ChargerState& _input, ChargerFleetData_ChargerState& _output)
{
  _output.state = _input.state;
  _output.charger_name = common::dds_string_alloc_and_copy(_input.charger_name);
  _output.error_message = common::dds_string_alloc_and_copy(_input.error_message);
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.robot_name = common::dds_string_alloc_and_copy(_input.robot_name);
}

void convert(const ChargerFleetData_ChargerState& _input, ChargerState& _output)
{
  _output.state = _input.state;
  _output.charger_name = std::string(_input.charger_name);
  _output.error_message = std::string(_input.error_message);
  _output.request_id = std::string(_input.request_id);
  _output.fleet_name = std::string(_input.fleet_name);
  _output.robot_name = std::string(_input.robot_name);
}


} // namespace messages
} // namespace charger_fleet
