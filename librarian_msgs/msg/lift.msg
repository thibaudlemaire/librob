# Types :
int8 LIFT_TRIGGER = 1      # In this case, payload is a dict with key "state" (boolean) to start/stop listening
int8 CURRENT_FLOOR = 10    # In this case, payload is a dict with key "floor" (int) corresponding to the current floor

# Fields
Header header
int8 type       # Need a Type constant
string payload  # Data, JSON formated
