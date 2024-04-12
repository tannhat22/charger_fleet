/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: FleetMessages.c
  Source: FleetMessages.idl
  Cyclone DDS: V0.7.0

*****************************************************************/
#include "FleetMessages.h"


static const uint32_t ChargerFleetData_ChargerMode_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (ChargerFleetData_ChargerMode, mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t ChargerFleetData_ChargerMode_desc =
{
  sizeof (ChargerFleetData_ChargerMode),
  4u,
  0u,
  0u,
  "ChargerFleetData::ChargerMode",
  NULL,
  2,
  ChargerFleetData_ChargerMode_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"ChargerFleetData\"><Struct name=\"ChargerMode\"><Member name=\"mode\"><ULong/></Member></Struct></Module></MetaData>"
};


static const uint32_t ChargerFleetData_ChargerRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (ChargerFleetData_ChargerRequest, charger_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (ChargerFleetData_ChargerRequest, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (ChargerFleetData_ChargerRequest, robot_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (ChargerFleetData_ChargerRequest, request_id),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (ChargerFleetData_ChargerRequest, charger_mode.mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t ChargerFleetData_ChargerRequest_desc =
{
  sizeof (ChargerFleetData_ChargerRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "ChargerFleetData::ChargerRequest",
  NULL,
  6,
  ChargerFleetData_ChargerRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"ChargerFleetData\"><Struct name=\"ChargerMode\"><Member name=\"mode\"><ULong/></Member></Struct><Struct name=\"ChargerRequest\"><Member name=\"charger_name\"><String/></Member><Member name=\"fleet_name\"><String/></Member><Member name=\"robot_name\"><String/></Member><Member name=\"request_id\"><String/></Member><Member name=\"charger_mode\"><Type name=\"ChargerMode\"/></Member></Struct></Module></MetaData>"
};


static const uint32_t ChargerFleetData_ChargerState_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (ChargerFleetData_ChargerState, state),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (ChargerFleetData_ChargerState, charger_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (ChargerFleetData_ChargerState, error_message),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (ChargerFleetData_ChargerState, request_id),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (ChargerFleetData_ChargerState, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (ChargerFleetData_ChargerState, robot_name),
  DDS_OP_RTS
};

const dds_topic_descriptor_t ChargerFleetData_ChargerState_desc =
{
  sizeof (ChargerFleetData_ChargerState),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "ChargerFleetData::ChargerState",
  NULL,
  7,
  ChargerFleetData_ChargerState_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"ChargerFleetData\"><Struct name=\"ChargerState\"><Member name=\"state\"><ULong/></Member><Member name=\"charger_name\"><String/></Member><Member name=\"error_message\"><String/></Member><Member name=\"request_id\"><String/></Member><Member name=\"fleet_name\"><String/></Member><Member name=\"robot_name\"><String/></Member></Struct></Module></MetaData>"
};
