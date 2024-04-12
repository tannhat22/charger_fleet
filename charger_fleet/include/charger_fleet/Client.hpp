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

#ifndef CHARGER_FLEET__INCLUDE__CHARGER_FLEET__CLIENT_HPP
#define CHARGER_FLEET__INCLUDE__CHARGER_FLEET__CLIENT_HPP

#include <memory>

#include <charger_fleet/ClientConfig.hpp>

#include <charger_fleet/messages/ChargerState.hpp>
#include <charger_fleet/messages/ChargerRequest.hpp>

namespace charger_fleet {

class Client
{
public:

  using SharedPtr = std::shared_ptr<Client>;

  /// Factory function that creates an instance of the Charger Fleet DDS Client.
  ///
  /// \param[in] config
  ///   Configuration that sets up the client to communicate with the server.
  /// \return
  ///   Shared pointer to a charger fleet client.
  static SharedPtr make(const ClientConfig& config);

  /// Attempts to send a new charger state to the charger fleet server, to be 
  /// registered by the fleet management system.
  ///
  /// \param[in] new_charger_state
  ///   Current charger state to be sent to the charger fleet server to update the
  ///   fleet management system.
  /// \return
  ///   True if charger state was successfully sent, false otherwise.
  bool send_charger_state(const messages::ChargerState& new_charger_state);

  /// Attempts to read and receive a new charger request from the charger fleet
  /// server, for commanding the charger client.
  ///
  /// \param[out] charger_request
  ///   Newly received charger charger request from the charger fleet server, to be
  ///   handled by the charger client.
  /// \return
  ///   True if a new charger request was received, false otherwise.
  bool read_charger_request(messages::ChargerRequest& charger_request);

  /// Destructor
  ~Client();

private:

  /// Forward declaration and unique implementation
  class ClientImpl;

  std::unique_ptr<ClientImpl> impl;

  Client(const ClientConfig& config);

};

} // namespace charger_fleet

#endif // CHARGER_FLEET__INCLUDE__CHARGER_FLEET__CLIENT_HPP
