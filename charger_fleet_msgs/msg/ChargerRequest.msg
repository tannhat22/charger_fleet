# The name of the charger that should process this message
string charger_name

# The robot that wishes to charge
string fleet_name
string robot_name

# A unique ID for each request. It is advised that you prefix this
# with the sender's node name. This is used for error tracking
# later on
string request_id

# Mode charge
ChargerMode charger_mode