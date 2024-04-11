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

#ifndef CHARGER_FLEET__SRC__MESSAGES__MESSAGE_UTILS_HPP
#define CHARGER_FLEET__SRC__MESSAGES__MESSAGE_UTILS_HPP

#include <charger_fleet/messages/ChargerMode.hpp>
#include <charger_fleet/messages/ChargerState.hpp>
#include <charger_fleet/messages/ChargeMode.hpp>
#include <charger_fleet/messages/ModeRequest.hpp>

#include "FleetMessages.h"

namespace charger_fleet {
namespace messages {

void convert(const ChargerMode& _input, ChargerFleetData_ChargerMode& _output);

void convert(const ChargerFleetData_ChargerMode& _input, ChargerMode& _output);

void convert(const ChargerState& _input, ChargerFleetData_ChargerState& _output);

void convert(const ChargerFleetData_ChargerState& _input, ChargerState& _output);

void convert(const ChargeMode& _input, ChargerFleetData_ChargeMode& _output);

void convert(const ChargerFleetData_ChargeMode& _input, ChargeMode& _output);

void convert(const ModeRequest& _input, ChargerFleetData_ModeRequest& _output);

void convert(const ChargerFleetData_ModeRequest& _input, ModeRequest& _output);

} // namespace 
} // namespace charger_fleet

#endif // CHARGER_FLEET__SRC__MESSAGES__MESSAGE_UTILS_HPP
