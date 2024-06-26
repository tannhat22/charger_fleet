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

#include <iostream>

#include <dds/dds.h>

#include "../messages/FleetMessages.h"

int main(int argc, char** argv)
{
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t reader;

  ChargerFleetData_ChargerState* msg;
  void* samples[1];
  dds_sample_info_t infos[1];

  dds_return_t rc;
  dds_qos_t* qos;

  (void)argc;
  (void)argv;

  /* Create a participant */
  participant = dds_create_participant(52, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic (
    participant, &ChargerFleetData_ChargerState_desc, 
    "robot_state", NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  /* Create a best effort Reader (UDP) */
  qos = dds_create_qos ();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  reader = dds_create_reader (participant, topic, qos, NULL);
  if (reader < 0)
    DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
  dds_delete_qos(qos);

  printf ("\n=== [Subscriber] Waiting for a sample ...\n");
  fflush (stdout);

  /* Initialize sample buffer, by pointing the void pointer within
   * the buffer array to a valid sample memory location. */
  samples[0] = ChargerFleetData_ChargerState__alloc();

  /* Poll until data has been read. */
  while (true)
  {
    /* Do the actual read.
     * The return value contains the number of read samples. */
    rc = dds_take(reader, samples, infos, 1, 1);
    if (rc < 0)
      DDS_FATAL("dds_read: %s\n", dds_strretcode(-rc));

    /* Check if we read some data and it is valid. */
    if ((rc > 0) && (infos[0].valid_data))
    {
      /* Print Message. */
      msg = (ChargerFleetData_ChargerState*)samples[0];
      std::cout << "=== [Subscriber] Received : " << std::endl;
      std::cout << "charger_name: " << msg->charger_name << std::endl;
      std::cout << "error_message: " << msg->error_message << std::endl;
      std::string request_id(msg->request_id);
      std::cout << "request_id: " << request_id << std::endl;
      std::cout << "fleet_name: " << msg->fleet_name << std::endl;
      std::cout << "robot_name: " << msg->robot_name << std::endl;
      
      std::cout << "state: ";
      if (msg->state == ChargerFleetData_ChargerState_Constants_CHARGER_IDLE)
        std::cout << "IDLE" << std::endl;
      else if (msg->state == ChargerFleetData_ChargerState_Constants_CHARGER_ASSIGNED)
        std::cout << "ASSIGNED" << std::endl;
      else if (msg->state == ChargerFleetData_ChargerState_Constants_CHARGER_CHARGING)
        std::cout << "CHARGING" << std::endl;
      else if (msg->state == ChargerFleetData_ChargerState_Constants_CHARGER_RELEASED)
        std::cout << "RELEASED" << std::endl;
      else if (msg->state == ChargerFleetData_ChargerState_Constants_CHARGER_ERROR)
        std::cout << "ERROR" << std::endl;
      //break;
    }
    else
    {
      /* Polling sleep. */
      dds_sleepfor (DDS_MSECS (20));
    }
  }

  /* Charger the data location. */
  ChargerFleetData_ChargerState_free (samples[0], DDS_FREE_ALL);

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete (participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  return EXIT_SUCCESS;
}
