uint32 CHARGER_IDLE = 1      # Charger is not occupied
uint32 CHARGER_ASSIGNED = 2  # Charger has been assigned a robot
uint32 CHARGER_CHARGING = 3  # Charger is charging
uint32 CHARGER_RELEASED = 4  # Charger has been disconnected from a robot
uint32 CHARGER_ERROR = 200   # Error state, see error_message for info

uint32 state  # One of the previously enumerated states

# The charger name should be unique in the RMF system and
# should match a charger name appearing in the traffic map
string charger_name

# The error_message field should be blank unless state is CHARGER_ERROR
string error_message

# The request_id field will be populated with the ID that started the
# charging cycle if state is anything other than CHARGER_IDLE
string request_id

# The robot that is currently assigned to this charger (if any)
string fleet_name
string robot_name

