# Status of fairino_cartesian_servo, published on /fairino_servo/status.

std_msgs/Header header

string state                 # DISABLED SYNCING ACTIVE HOLD RECOVERING FAULT LOCKOUT
string reason
string mode                  # plan | servo | transition | unknown
string active_position_controller   # fairino20_controller | fairino_servo_controller | none | unknown
bool robot_status_required   # false with mock hardware

# Fault classification of the most relevant active fault.
string fault_source          # none | controller | sdk | signal | servo
int32 main_error_code
int32 sub_error_code
int32 sdk_return_code
string fault_description
string fault_severity        # none | info | warning | error | critical
string fault_policy          # none | auto_recover | operator_reset | lockout
string suggested_action

# Recovery bookkeeping.
bool auto_recovery_enabled
uint32 recovery_attempts
uint32 max_recovery_attempts
string last_recovery_result
float64 last_recovery_duration_s

# Feedback and input freshness.
float64 joint_state_age_s
float64 joint_state_rate_hz
float64 target_age_s
bool target_received
float64 tracking_error_rad   # max |q_command - q_measured|

# Loop configuration and timing.
float64 solver_frequency_hz
float64 command_frequency_hz
uint32 self_collision_pairs
bool realtime_scheduling     # SCHED_FIFO obtained for the servo thread
TimingStats[] timing

# Limits actually applied by the servo (software limits, not FAIRINO safety limits).
float64[] joint_velocity_limit_rad_s
float64[] joint_acceleration_limit_rad_s2
float64 max_linear_speed_m_s
float64 max_angular_speed_rad_s
float64 filter_tau_s
