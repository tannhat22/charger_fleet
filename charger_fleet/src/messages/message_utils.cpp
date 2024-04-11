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

void convert(const ChargerState& _input, ChargerFleetData_ChargerState& _output)
{
  _output.name = common::dds_string_alloc_and_copy(_input.name);
  _output.model = common::dds_string_alloc_and_copy(_input.model);
  _output.task_id = common::dds_string_alloc_and_copy(_input.task_id);
  convert(_input.mode, _output.mode);
}

void convert(const ChargerFleetData_ChargerState& _input, ChargerState& _output)
{
  _output.name = std::string(_input.name);
  _output.model = std::string(_input.model);
  _output.task_id = std::string(_input.task_id);
  convert(_input.mode, _output.mode);
}

void convert(const ChargeMode& _input, ChargerFleetData_ChargeMode& _output)
{
  _output.mode = _input.mode;
}

void convert(const ChargerFleetData_ChargeMode& _input, ChargeMode& _output)
{
  _output.mode = _input.mode;
}

void convert(const ModeRequest& _input, ChargerFleetData_ModeRequest& _output)
{
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.charger_name = common::dds_string_alloc_and_copy(_input.charger_name);
  convert(_input.mode, _output.mode);
  _output.task_id = common::dds_string_alloc_and_copy(_input.task_id);
}

void convert(const ChargerFleetData_ModeRequest& _input, ModeRequest& _output)
{
  _output.fleet_name = std::string(_input.fleet_name);
  _output.charger_name = std::string(_input.charger_name);
  convert(_input.mode, _output.mode);
  _output.task_id = std::string(_input.task_id);
}

} // namespace messages
} // namespace charger_fleet
