string name
string model

# task_id is copied in from the most recent Request message,
# such as ModeRequest, ChargeRequest
string task_id

# The sequence number of this message. Every new message should increment the
# sequence number by 1.
uint64 seq

ChargerMode mode
