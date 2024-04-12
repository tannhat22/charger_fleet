/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: FleetMessages.h
  Source: FleetMessages.idl
  Cyclone DDS: V0.7.0

*****************************************************************/

#include "dds/ddsc/dds_public_impl.h"

#ifndef _DDSL_FLEETMESSAGES_H_
#define _DDSL_FLEETMESSAGES_H_


#ifdef __cplusplus
extern "C" {
#endif

#define ChargerFleetData_ChargerMode_Constants_MODE_CHARGE 1
#define ChargerFleetData_ChargerMode_Constants_MODE_UNCHARGE 2
#define ChargerFleetData_ChargerState_Constants_CHARGER_IDLE 1
#define ChargerFleetData_ChargerState_Constants_CHARGER_ASSIGNED 2
#define ChargerFleetData_ChargerState_Constants_CHARGER_CHARGING 3
#define ChargerFleetData_ChargerState_Constants_CHARGER_RELEASED 4
#define ChargerFleetData_ChargerState_Constants_CHARGER_ERROR 200


typedef struct ChargerFleetData_ChargerMode
{
  uint32_t mode;
} ChargerFleetData_ChargerMode;

extern const dds_topic_descriptor_t ChargerFleetData_ChargerMode_desc;

#define ChargerFleetData_ChargerMode__alloc() \
((ChargerFleetData_ChargerMode*) dds_alloc (sizeof (ChargerFleetData_ChargerMode)));

#define ChargerFleetData_ChargerMode_free(d,o) \
dds_sample_free ((d), &ChargerFleetData_ChargerMode_desc, (o))


typedef struct ChargerFleetData_ChargerRequest
{
  char * charger_name;
  char * fleet_name;
  char * robot_name;
  char * request_id;
  ChargerFleetData_ChargerMode charger_mode;
} ChargerFleetData_ChargerRequest;

extern const dds_topic_descriptor_t ChargerFleetData_ChargerRequest_desc;

#define ChargerFleetData_ChargerRequest__alloc() \
((ChargerFleetData_ChargerRequest*) dds_alloc (sizeof (ChargerFleetData_ChargerRequest)));

#define ChargerFleetData_ChargerRequest_free(d,o) \
dds_sample_free ((d), &ChargerFleetData_ChargerRequest_desc, (o))


typedef struct ChargerFleetData_ChargerState
{
  uint32_t state;
  char * charger_name;
  char * error_message;
  char * request_id;
  char * fleet_name;
  char * robot_name;
} ChargerFleetData_ChargerState;

extern const dds_topic_descriptor_t ChargerFleetData_ChargerState_desc;

#define ChargerFleetData_ChargerState__alloc() \
((ChargerFleetData_ChargerState*) dds_alloc (sizeof (ChargerFleetData_ChargerState)));

#define ChargerFleetData_ChargerState_free(d,o) \
dds_sample_free ((d), &ChargerFleetData_ChargerState_desc, (o))

#ifdef __cplusplus
}
#endif
#endif /* _DDSL_FLEETMESSAGES_H_ */
