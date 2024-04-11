/*
 * Copyright (C) 2019 Open Source Chargerics Foundation
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

#ifndef CHARGER_FLEET__SRC__SERVERIMPL_HPP
#define CHARGER_FLEET__SRC__SERVERIMPL_HPP

#include <charger_fleet/messages/ChargerState.hpp>
#include <charger_fleet/messages/ModeRequest.hpp>
#include <charger_fleet/Server.hpp>
#include <charger_fleet/ServerConfig.hpp>

#include <dds/dds.h>

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace charger_fleet {

class Server::ServerImpl
{
public:

  /// DDS related fields required for the server to operate
  struct Fields
  {
    /// DDS participant that is tied to the configured dds_domain_id
    dds_entity_t participant;

    /// DDS subscribers for new incoming charger states from clients
    dds::DDSSubscribeHandler<ChargerFleetData_ChargerState, 10>::SharedPtr 
        charger_state_sub;

    /// DDS publisher for mode requests to be sent to clients
    dds::DDSPublishHandler<ChargerFleetData_ModeRequest>::SharedPtr
        mode_request_pub;
  };

  ServerImpl(const ServerConfig& config);

  ~ServerImpl();

  void start(Fields fields);

  bool read_charger_states(std::vector<messages::ChargerState>& new_charger_states);

  bool send_mode_request(const messages::ModeRequest& mode_request);

private:

  Fields fields;

  ServerConfig server_config;

};

} // namespace charger_fleet

#endif // CHARGER_FLEET__SRC__SERVERIMPL_HPP
