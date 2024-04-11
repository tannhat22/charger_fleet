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

#include <charger_fleet/Client.hpp>

#include "ClientImpl.hpp"

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace charger_fleet {

Client::SharedPtr Client::make(const ClientConfig& _config)
{
  SharedPtr client = SharedPtr(new Client(_config));

  dds_entity_t participant = dds_create_participant(
      static_cast<dds_domainid_t>(_config.dds_domain), NULL, NULL);
  if (participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  dds::DDSPublishHandler<ChargerFleetData_ChargerState>::SharedPtr state_pub(
      new dds::DDSPublishHandler<ChargerFleetData_ChargerState>(
          participant, &ChargerFleetData_ChargerState_desc,
          _config.dds_state_topic));

  dds::DDSSubscribeHandler<ChargerFleetData_ModeRequest>::SharedPtr 
      mode_request_sub(
          new dds::DDSSubscribeHandler<ChargerFleetData_ModeRequest>(
              participant, &ChargerFleetData_ModeRequest_desc,
              _config.dds_mode_request_topic));

  if (!state_pub->is_ready() ||
      !mode_request_sub->is_ready())
    return nullptr;

  client->impl->start(ClientImpl::Fields{
      std::move(participant),
      std::move(state_pub),
      std::move(mode_request_sub)});
  return client;
}

Client::Client(const ClientConfig& _config)
{
  impl.reset(new ClientImpl(_config));
}

Client::~Client()
{}

bool Client::send_charger_state(const messages::ChargerState& _new_charger_state)
{
  return impl->send_charger_state(_new_charger_state);
}

bool Client::read_mode_request(messages::ModeRequest& _mode_request)
{
  return impl->read_mode_request(_mode_request);
}

} // namespace charger_fleet
