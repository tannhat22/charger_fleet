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

#include <charger_fleet/Server.hpp>

#include "ServerImpl.hpp"

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace charger_fleet {

Server::SharedPtr Server::make(const ServerConfig& _config)
{
  SharedPtr server = SharedPtr(new Server(_config));

  dds_entity_t participant = dds_create_participant(
      static_cast<dds_domainid_t>(_config.dds_domain), NULL, NULL);
  if (participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  dds::DDSSubscribeHandler<ChargerFleetData_ChargerState, 10>::SharedPtr state_sub(
      new dds::DDSSubscribeHandler<ChargerFleetData_ChargerState, 10>(
          participant, &ChargerFleetData_ChargerState_desc,
          _config.dds_charger_state_topic));

  dds::DDSPublishHandler<ChargerFleetData_ChargerRequest>::SharedPtr 
      charger_request_pub(
          new dds::DDSPublishHandler<ChargerFleetData_ChargerRequest>(
              participant, &ChargerFleetData_ChargerRequest_desc,
              _config.dds_charger_request_topic));

  if (!state_sub->is_ready() ||
      !charger_request_pub->is_ready())
    return nullptr;

  server->impl->start(ServerImpl::Fields{
      std::move(participant),
      std::move(state_sub),
      std::move(charger_request_pub)});
  return server;
}

Server::Server(const ServerConfig& _config)
{
  impl.reset(new ServerImpl(_config));
}

Server::~Server()
{}

bool Server::read_charger_states(
    std::vector<messages::ChargerState>& _new_charger_states)
{
  return impl->read_charger_states(_new_charger_states);
}

bool Server::send_charger_request(const messages::ChargerRequest& _charger_request)
{
  return impl->send_charger_request(_charger_request);
}

} // namespace charger_fleet
