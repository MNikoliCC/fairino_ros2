# Audit record for every operator-, recovery- or safety-relevant action.
# Published on /fairino_robot/events (hardware) and /fairino_servo/events (servo).

builtin_interfaces/Time stamp
string source        # node that performed the action
string category      # recovery | speed | safety_config | controller | state | fault | operator
string action        # e.g. ResetAllError, SetSpeedInstant, ACTIVE->HOLD
string old_value
string new_value
int32 result_code    # SDK return code, 0 when not applicable
bool success
string message
