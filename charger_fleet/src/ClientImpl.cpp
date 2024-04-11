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

bool Client::ClientImpl::read_mode_request
    (messages::ModeRequest& _mode_request)
{
  auto mode_requests = fields.mode_request_sub->read();
  if (!mode_requests.empty())
  {
    convert(*(mode_requests[0]), _mode_request);
    return true;
  }
  return false;
}

} // namespace charger_fleet
