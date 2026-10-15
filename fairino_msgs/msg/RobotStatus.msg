# FAIRINO controller status published by the ros2_control hardware plugin
# (fairino_hardware/FairinoHardwareInterface), which is the only owner of the
# FAIRINO SDK connection. Values come from the SDK realtime state package
# (robot port 20004) unless noted otherwise.

std_msgs/Header header

# --- SDK connection --------------------------------------------------------
bool connected                  # GetSDKComState() == 0 and realtime state is fresh
int32 sdk_com_state             # 0 normal, 1 disconnected, 2 reconnecting
int32 sdk_com_state_return_code # return code of GetSDKComState()
int32 realtime_state_return_code
float64 realtime_state_age_s    # age of the last successful realtime-state snapshot
string robot_model
string controller_version
string web_version

# --- Robot state -----------------------------------------------------------
bool robot_enabled              # rbtEnableState
uint8 robot_mode                # 0 automatic, 1 manual
uint8 motion_state              # 1 stopped, 2 running, 3 paused, 4 drag
uint8 program_state             # 1 stopped, 2 running, 3 paused
int32 motion_done               # 1 in position, 0 moving

# --- Faults (controller main/sub codes, see FAIRINO manual Appendix 1) -----
int32 main_error_code
int32 sub_error_code

# --- Safety-related signals ------------------------------------------------
bool emergency_stop             # EmergencyStop
bool button_box_stop            # btnBoxStopSignal
bool safety_stop0               # SafetyStop0State (SI0)
bool safety_stop1               # SafetyStop1State (SI1)
bool collision                  # collisionState
bool safety_door_alarm          # safetyDoorAlarm
bool safety_plane_alarm         # safetyPlaneAlarm
bool safety_data_abnormal       # safetyDataState
uint8 driver_alarm_axis         # driverAlarm: axis number, 0 = none
uint8 driver_error_axis         # driverError: drive communication fault axis, 0 = none
uint8[6] driver_com_error       # driverComError
bool out_soft_limit_error       # outSoftLimitError
bool abnormal_stop              # abnomalStop
uint8 alarm_reboot_robot        # 1 E-stop released -> power cycle, 2 joint comm -> power cycle
bool alarm_check_estop_button   # alarmCheckEmergStopBtn
bool singular_pose              # strangePosFlag
uint8 motion_alarm              # motonAlarm
int32 ctrl_box_error            # ctrlBoxError
int32 udp_cmd_state             # udpCmdState, robot port 20007 (ServoJ UDP)

# --- Speed related ---------------------------------------------------------
float64 speed_scale_manual      # global speed in percent, manual mode (realtime value x 100)
float64 speed_scale_auto        # global speed in percent, automatic mode (realtime value x 100)
uint8[6] collision_level        # controller collision level per joint (read only here)

# --- Joint data (degrees, from the realtime state package) -----------------
float64[6] joint_position_deg
float64[6] joint_velocity_deg_s
float64[6] target_joint_velocity_deg_s
int32 servoj_cmd_num            # servoJCmdNum reported by the controller

# --- Cartesian state as the FAIRINO WebApp shows it (base frame) -----------
# x, y, z [mm], rx, ry, rz [deg] (rotation about the fixed axes X, Y, Z).
int32 tool_number               # active tool coordinate system (0 = flange)
int32 user_frame_number         # active workpiece coordinate system
float64[6] flange_pose          # flange_cur_pos
float64[6] tcp_pose             # tl_cur_pos: TCP of the active tool

# --- ros2_control / ServoJ streaming (measured inside read()/write()) ------
uint8 STREAMING_DISARMED = 0    # no ServoJ is sent; requires recover/arm service
uint8 STREAMING_WAIT_SYNC = 1   # armed once the command matches the measured position
uint8 STREAMING_ARMED = 2       # ServoJ sent every write() cycle
uint8 streaming_state
string streaming_reason
float64 servoj_cmd_period_s
int32 last_servoj_return_code
uint64 servoj_calls
uint64 servoj_errors
# ServoJ flow control: the controller executes ServoJ points from a FIFO. On
# SimMachine 3.9.9 it consumed ~928 points/s while 1000/s were sent, so the
# queue (and the motion delay) grew by ~72 ms per second. write() therefore
# estimates the queue depth (sent - executed, from servoJCmdNum) and skips a
# cycle when the estimate exceeds servoj_max_queue.
int32 servoj_queue_estimate
int32 servoj_max_queue
uint64 servoj_flow_skips
uint64 servoj_flow_rebaselines
uint64 servoj_flow_probes             # pauses that tested for an empty FIFO
uint64 servoj_flow_drift_corrections  # probes that found the estimate too high
# Points whose step was limited to point_velocity_factor x v_max x cmdT (the
# robot lags briefly after a skipped cycle or a controller stall).
uint64 servoj_point_limited
uint64 jump_guard_trips
int32 last_read_return_code
uint64 read_errors
uint32 consecutive_read_errors
float64 joint_feedback_age_s    # time since the last successful read()
float64 joint_feedback_rate_hz  # successful read() rate over the last status window
float64 write_period_mean_ms
float64 write_period_p99_ms
float64 write_period_max_ms
uint64 write_period_overruns    # write() periods longer than 2 x cmdT in the status window

# --- Configuration ---------------------------------------------------------
bool safety_config_writes_allowed  # hardware parameter allow_safety_config_writes
