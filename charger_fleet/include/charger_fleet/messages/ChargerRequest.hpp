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

#ifndef CHARGER_FLEET__INCLUDE__CHARGER_FLEET__MESSAGES__CHARGERREQUEST_HPP
#define CHARGER_FLEET__INCLUDE__CHARGER_FLEET__MESSAGES__CHARGERREQUEST_HPP

#include <string>
#include "ChargerMode.hpp"

namespace charger_fleet {
namespace messages {

struct ChargerRequest
{
  std::string charger_name;
  std::string fleet_name;
  std::string robot_name;
  std::string request_id;
  ChargerMode charger_mode;
};

} // namespace messages
} // namespace charger_fleet

#endif // CHARGER_FLEET__INCLUDE__CHARGER_FLEET__MESSAGES__CHARGERREQUEST_HPP
