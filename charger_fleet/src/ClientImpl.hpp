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

#ifndef CHARGER_FLEET__SRC__CLIENTIMPL_HPP
#define CHARGER_FLEET__SRC__CLIENTIMPL_HPP

#include <charger_fleet/messages/ChargerState.hpp>
#include <charger_fleet/messages/ChargerRequest.hpp>
#include <charger_fleet/Client.hpp>
#include <charger_fleet/ClientConfig.hpp>

#include <dds/dds.h>

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace charger_fleet {

class Client::ClientImpl
{
public:

  /// DDS related fields required for the client to operate
  struct Fields
  {
    /// DDS participant that is tied to the configured dds_domain_id
    dds_entity_t participant;

    /// DDS publisher that handles sending out current charger states to the 
    /// server
    dds::DDSPublishHandler<ChargerFleetData_ChargerState>::SharedPtr
        state_pub;

    /// DDS subscriber for charger requests coming from the server
    dds::DDSSubscribeHandler<ChargerFleetData_ChargerRequest>::SharedPtr 
        charger_request_sub;
  };

  ClientImpl(const ClientConfig& config);

  ~ClientImpl();

  void start(Fields fields);

  bool send_charger_state(const messages::ChargerState& new_charger_state);

  bool read_charger_request(messages::ChargerRequest& charger_request);

private:

  Fields fields;

  ClientConfig client_config;

};

} // namespace charger_fleet

#endif // CHARGER_FLEET__SRC__CLIENTIMPL_HPP
