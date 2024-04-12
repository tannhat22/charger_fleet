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

#include "ClientImpl.hpp"
#include "messages/message_utils.hpp"

namespace charger_fleet {

Client::ClientImpl::ClientImpl(const ClientConfig& _config) :
  client_config(_config)
{}

Client::ClientImpl::~ClientImpl()
{
  dds_return_t return_code = dds_delete(fields.participant);
  if (return_code != DDS_RETCODE_OK)
  {
    DDS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
}

void Client::ClientImpl::start(Fields _fields)
{
  fields = std::move(_fields);
}

bool Client::ClientImpl::send_charger_state(
    const messages::ChargerState& _new_charger_state)
{
  ChargerFleetData_ChargerState* new_rs = ChargerFleetData_ChargerState__alloc();
  convert(_new_charger_state, *new_rs);
  bool sent = fields.state_pub->write(new_rs);
  ChargerFleetData_ChargerState_free(new_rs, DDS_FREE_ALL);
  return sent;
}

bool Client::ClientImpl::read_charger_request
    (messages::ChargerRequest& _charger_request)
{
  auto charger_requests = fields.charger_request_sub->read();
  if (!charger_requests.empty())
  {
    convert(*(charger_requests[0]), _charger_request);
    return true;
  }
  return false;
}

} // namespace charger_fleet
