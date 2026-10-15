#include "fairino_hardware/fairino_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

namespace fairino_hardware{

namespace
{
constexpr int kServoJCommunicationType = 1;  // UDP, robot port 20007
// A command stream whose last two changes were at most this far apart is
// "moving"; only a moving stream may catch up after a delivery gap.
constexpr double kActiveStreamInterval = 0.003;
// WAIT_SYNC arms only after the command matched the measured position and did
// not change for this many cycles (no moving stream is ever armed).
constexpr int kSyncStableCycles = 20;
// Consecutive flow-control skips (cycles without executed progress) after
// which the queue estimate is reset; see write().
constexpr int kFlowRebaselineSkips = 500;
// Drift probe (see read()): after this many consecutive feedback frames in
// which the estimated FIFO was full but at most kFlowLowProgress points were
// executed, sending pauses for kFlowProbeFrames frames; at most one probe per
// kFlowProbeInterval seconds.
constexpr int kFlowProbeTriggerFrames = 25;
constexpr int kFlowLowProgress = 2;
constexpr int kFlowProbeFrames = 3;
constexpr double kFlowProbeInterval = 0.5;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
// FAIRINO FR20 joint speed maxima (J1-J3 120 deg/s, J4-J6 180 deg/s), used as
// the reference for the discontinuity guard when no parameter is given.
constexpr std::array<double, 6> kDefaultMaxJointVelocityDegS{120.0, 120.0, 120.0, 180.0, 180.0, 180.0};

rclcpp::Logger logger()
{
    return rclcpp::get_logger("FairinoHardwareInterface");
}

double seconds_between(std::chrono::steady_clock::time_point later,
                       std::chrono::steady_clock::time_point earlier)
{
    return std::chrono::duration<double>(later - earlier).count();
}

std::vector<double> parse_double_list(const std::string& text)
{
    std::vector<double> values;
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
        if (!item.empty()) {
            values.push_back(std::stod(item));
        }
    }
    return values;
}

// SDK 3.9.9 does not fill frame_head/frame_cnt; the controller system time
// (millisecond resolution) identifies a new realtime frame instead.
int64_t robot_time_key(const ROBOT_STATE_PKG& pkg)
{
    return ((static_cast<int64_t>(pkg.robotTime.hour) * 60 + pkg.robotTime.minute) * 60 +
            pkg.robotTime.second) * 1000 + pkg.robotTime.millisecond;
}

// CNDE fields streamed on robot port 20004. The controller default is only
// fields 3..75 at 8 ms, which omits safety door/plane, abnormal stop, UDP
// state, driver alarms, speed scales and collision levels. The two string
// fields (CurrentLuaFileName, AxleGenComData) are not needed.
std::vector<RobotState> realtime_state_fields()
{
    std::vector<RobotState> fields;
    for (int id = static_cast<int>(RobotState::ProgramState); id <= static_cast<int>(RobotState::ProgramRunState); ++id) {
        if (id == static_cast<int>(RobotState::CurrentLuaFileName) || id == static_cast<int>(RobotState::AxleGenComData)) {
            continue;
        }
        fields.push_back(static_cast<RobotState>(id));
    }
    return fields;
}

std::string speed_text(const ROBOT_STATE_PKG& pkg)
{
    std::ostringstream text;
    text.precision(1);
    text << std::fixed << "manual=" << pkg.speedScaleManual * 100.0 << "% auto=" << pkg.speedScaleAuto * 100.0
         << "% (active: " << (pkg.robot_mode == 1 ? "manual" : "automatic") << ")";
    return text.str();
}

std::string joint_values_text(const double* values)
{
    std::ostringstream text;
    text.precision(4);
    text << std::fixed;
    for (int j = 0; j < 6; ++j) {
        text << (j ? "," : "") << values[j];
    }
    return text.str();
}
}  // namespace

FairinoHardwareInterface::~FairinoHardwareInterface()
{
    join_cleanup();
    stop_status_node();
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    shutdown_robot_connection();
}

void FairinoHardwareInterface::join_cleanup()
{
    if (cleanup_thread_.joinable()) {
        cleanup_thread_.join();
    }
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo& sysinfo){
    if (hardware_interface::SystemInterface::on_init(sysinfo) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    info_ = sysinfo;//info_是父类中定义的变量

    if (info_.joints.size() != 6) {
        RCLCPP_FATAL(logger(), "FAIRINO hardware expects 6 joints, got %zu.", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo& joint : info_.joints) {

        //指令部分
        if (joint.command_interfaces.size() != 1) {//开放servoJ
            RCLCPP_FATAL(logger(),
                        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                        joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(logger(),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        //关节状态部分
        if (joint.state_interfaces.size() != 1) {
            RCLCPP_FATAL(logger(), "Joint '%s' has %zu state interface. 1 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(logger(),
                        "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    try {
        load_parameters();
    } catch (const std::exception& error) {
        RCLCPP_FATAL(logger(), "Invalid FAIRINO hardware parameter: %s", error.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}//end on_init

std::string FairinoHardwareInterface::hardware_parameter(const std::string& name, const std::string& fallback) const
{
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
}

void FairinoHardwareInterface::load_parameters()
{
    _controller_ip = hardware_parameter("controller_ip", CONTROLLER_IP_ADDRESS);
    servoj_cmd_period_s_ = std::stod(hardware_parameter("servoj_cmd_period", "0.001"));
    status_rate_hz_ = std::stod(hardware_parameter("status_rate", "50.0"));
    feedback_timeout_s_ = std::stod(hardware_parameter("feedback_timeout", "0.1"));
    feedback_error_timeout_s_ = std::stod(hardware_parameter("feedback_error_timeout", "0.5"));
    realtime_state_period_ms_ = std::stoi(hardware_parameter("realtime_state_period_ms", "4"));
    jump_guard_factor_ = std::stod(hardware_parameter("jump_guard_factor", "2.0"));
    sync_tolerance_rad_ = std::stod(hardware_parameter("sync_tolerance", "0.002"));
    standstill_velocity_deg_s_ = std::stod(hardware_parameter("standstill_velocity_deg_s", "0.5"));
    servoj_max_queue_ = std::stoi(hardware_parameter("servoj_max_queue", "8"));
    point_velocity_factor_ = std::stod(hardware_parameter("point_velocity_factor", "1.0"));
    jump_guard_max_gap_s_ = std::stod(hardware_parameter("jump_guard_max_gap", "0.05"));
    const std::string allow = hardware_parameter("allow_safety_config_writes", "false");
    allow_safety_config_writes_ = (allow == "true" || allow == "True" || allow == "1");

    std::vector<double> max_velocity(kDefaultMaxJointVelocityDegS.begin(), kDefaultMaxJointVelocityDegS.end());
    const std::string velocity_text = hardware_parameter("max_joint_velocity_deg_s", "");
    if (!velocity_text.empty()) {
        max_velocity = parse_double_list(velocity_text);
    }
    if (max_velocity.size() != 6) {
        throw std::invalid_argument("max_joint_velocity_deg_s needs 6 comma separated values");
    }
    for (int j = 0; j < 6; ++j) {
        if (!(max_velocity[j] > 0.0)) {
            throw std::invalid_argument("max_joint_velocity_deg_s values must be positive");
        }
        max_joint_velocity_rad_s_[j] = max_velocity[j] * kDegToRad;
    }
    if (realtime_state_period_ms_ < 4 || realtime_state_period_ms_ > 1000) {
        throw std::invalid_argument("realtime_state_period_ms must be within [4, 1000] (CNDE limit)");
    }
    if (!(servoj_cmd_period_s_ >= 0.001 && servoj_cmd_period_s_ <= 0.016)) {
        throw std::invalid_argument("servoj_cmd_period must be within [0.001, 0.016] s");
    }
    if (!(status_rate_hz_ > 0.0) || !(jump_guard_factor_ >= 1.0) || !(sync_tolerance_rad_ > 0.0) ||
        !(feedback_timeout_s_ > 0.0) || feedback_error_timeout_s_ < feedback_timeout_s_ ||
        !(point_velocity_factor_ > 0.0 && point_velocity_factor_ <= jump_guard_factor_) ||
        !(jump_guard_max_gap_s_ >= servoj_cmd_period_s_ && jump_guard_max_gap_s_ <= 0.2)) {
        throw std::invalid_argument(
            "status_rate, jump_guard_factor, sync_tolerance, point_velocity_factor or feedback timeouts are invalid");
    }
    RCLCPP_INFO(logger(),
        "FAIRINO hardware: ip=%s cmdT=%.4f s status=%.1f Hz guard=%.2fx sync_tol=%.4f rad "
        "max ServoJ queue=%d safety_config_writes=%s",
        _controller_ip.c_str(), servoj_cmd_period_s_, status_rate_hz_, jump_guard_factor_,
        sync_tolerance_rad_, servoj_max_queue_, allow_safety_config_writes_ ? "ALLOWED" : "refused");
}

std::vector<hardware_interface::StateInterface> FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_state[i]));
  }

  //导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &_jnt_position_command[i]));
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    using namespace std::chrono_literals;
    RCLCPP_INFO(logger(), "Starting ...please wait...");
    join_cleanup();
    stop_status_node();
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    shutdown_robot_connection();

    //做变量的初始化工作
    _ptr_robot = std::make_unique<FRRobot>();//创建机器人实例
    for(int i=0;i<6;i++){//初始化变量
        _jnt_position_command[i] = 0;
        _jnt_velocity_command[i] = 0;
        _jnt_torque_command[i] = 0;
        _jnt_position_state[i] = 0;
        _jnt_velocity_state[i] = 0;
        _jnt_torque_state[i] = 0;
    }
    _control_mode = 0;//默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
    streaming_state_ = RobotStatus::STREAMING_DISARMED;
    disarm_cause_ = static_cast<int>(DisarmCause::kStartup);
    have_frame_ = false;
    have_last_write_ = false;
    consecutive_read_errors_ = 0;
    last_servoj_rc_ = 0;
    sync_stable_cycles_ = 0;
    previous_change_interval_ = 1.0e9;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_valid_ = false;
        frame_updates_ = 0;
    }

    // The CNDE field set must be configured before RPC(): changing it on a
    // live connection makes the SDK parse frames with the new layout while
    // the controller still sends the old one (garbage values, verified on
    // SimMachine 3.9.9).
    const std::vector<RobotState> fields = realtime_state_fields();
    int returncode = _ptr_robot->SetRobotRealtimeStateConfig(fields, realtime_state_period_ms_);
    if (returncode != 0) {
        RCLCPP_ERROR(logger(), "SetRobotRealtimeStateConfig(%zu fields, %d ms) failed with code %d",
                     fields.size(), realtime_state_period_ms_, returncode);
        shutdown_robot_connection();
        return hardware_interface::CallbackReturn::ERROR;
    }

    returncode = _ptr_robot->RPC(_controller_ip.c_str());//建立xmlrpc连接
    rclcpp::sleep_for(200ms);//等待一段时间让控制器的rpc连接建立完毕
    if(returncode != 0){
        RCLCPP_ERROR(logger(), "FAIRINO SDK connection to %s failed, error code %d", _controller_ip.c_str(), returncode);
        shutdown_robot_connection();
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger(), "FAIRINO SDK connected to %s", _controller_ip.c_str());

    std::vector<RobotState> active_fields;
    int active_period = 0;
    returncode = _ptr_robot->GetRobotRealtimeStateConfig(active_fields, active_period);
    if (returncode != 0 || active_fields.size() != fields.size() || active_period != realtime_state_period_ms_) {
        RCLCPP_ERROR(logger(),
            "Realtime state configuration not applied (rc=%d, %zu/%zu fields, period %d/%d ms); "
            "safety signals would not be monitored.",
            returncode, active_fields.size(), fields.size(), active_period, realtime_state_period_ms_);
        shutdown_robot_connection();
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger(), "FAIRINO realtime state: %zu fields every %d ms", active_fields.size(), active_period);

    char model[64] = {0};
    char web[64] = {0};
    char controller[64] = {0};
    const int version_rc = _ptr_robot->GetSoftwareVersion(model, web, controller);
    robot_model_ = version_rc == 0 ? std::string(model) : "unknown (rc=" + std::to_string(version_rc) + ")";
    web_version_ = version_rc == 0 ? std::string(web) : "";
    controller_version_ = version_rc == 0 ? std::string(controller) : "";
    RCLCPP_INFO(logger(), "FAIRINO model='%s' web='%s' controller='%s'",
                robot_model_.c_str(), web_version_.c_str(), controller_version_.c_str());

    // Wait for the realtime state stream (robot port 20004) and initialise the
    // command with the measured position: a wrong initial command would make
    // the first ServoJ jump.
    ROBOT_STATE_PKG pkg{};
    bool have_state = false;
    const auto deadline = SteadyClock::now() + 3s;
    while (SteadyClock::now() < deadline) {
        if (_ptr_robot->GetRobotRealTimeState(&pkg) == 0 &&
            std::all_of(std::begin(pkg.jt_cur_pos), std::end(pkg.jt_cur_pos),
                        [](double value) { return std::isfinite(value); })) {
            have_state = true;
            break;
        }
        rclcpp::sleep_for(20ms);
    }
    if (!have_state) {
        RCLCPP_ERROR(logger(), "No FAIRINO realtime state received; hardware cannot start safely.");
        shutdown_robot_connection();
        return hardware_interface::CallbackReturn::ERROR;
    }
    for (int j = 0; j < 6; ++j) {
        _jnt_position_state[j] = pkg.jt_cur_pos[j] * kDegToRad;
        _jnt_position_command[j] = _jnt_position_state[j];
        last_sent_command_[j] = _jnt_position_state[j];
        previous_command_[j] = _jnt_position_state[j];
    }
    have_frame_ = true;
    last_frame_key_ = robot_time_key(pkg);
    last_frame_time_ = SteadyClock::now();
    last_good_read_time_ = last_frame_time_;
    last_command_change_time_ = last_frame_time_;
    flow_have_baseline_ = false;
    RCLCPP_INFO(logger(), "Initial command position [rad]: %s", joint_values_text(_jnt_position_command).c_str());

    start_status_node();

    // A controller fault or a disabled robot must not prevent activation:
    // the operator needs the status topic and the recovery service. Streaming
    // stays disarmed until a controlled recovery succeeds.
    const std::string blocking = blocking_condition(pkg);
    if (!blocking.empty()) {
        disarm(DisarmCause::kControllerFault, pkg.main_code, pkg.sub_code);
        audit("state", "activate", "", "DISARMED", 0, false,
              "hardware active, ServoJ streaming disarmed: " + blocking);
        RCLCPP_WARN(logger(), "Hardware active but ServoJ disarmed: %s", blocking.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    int start_rc = 0;
    {
        std::lock_guard<std::mutex> lock(rpc_mutex_);
        start_rc = _ptr_robot->ServoMoveStart(kServoJCommunicationType);
    }
    reset_flow_control();
    if (start_rc != 0) {
        disarm(DisarmCause::kServoMoveStartFailed, start_rc);
        audit("state", "ServoMoveStart(UDP)", "", "DISARMED", start_rc, false,
              "hardware active, ServoJ streaming disarmed");
        RCLCPP_ERROR(logger(), "ServoMoveStart(UDP) failed with code %d; streaming disarmed.", start_rc);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    // Arm only once the command interface holds the measured position and is
    // static: a position controller that stayed active across a hardware
    // re-activation may still hold an older command.
    disarm_cause_ = static_cast<int>(DisarmCause::kAwaitingSync);
    streaming_state_ = RobotStatus::STREAMING_WAIT_SYNC;
    audit("state", "activate", "DISARMED", "WAIT_SYNC", 0, true, "ServoMoveStart(UDP) succeeded");
    RCLCPP_INFO(logger(), "FAIRINO hardware active; ServoJ UDP streaming arms when the command is static at the "
                "measured position (cmdT=%.4f s).", servoj_cmd_period_s_);
    return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(logger(), "Stopping ...please wait...");
    stop_generation_.fetch_add(1);
    disarm(DisarmCause::kStopRequest);
    join_cleanup();
    stop_status_node();
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
    shutdown_robot_connection();
    RCLCPP_INFO(logger(), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_ERROR(logger(), "FAIRINO hardware entered error handling: %s", disarm_reason_text().c_str());
    stop_generation_.fetch_add(1);
    disarm(DisarmCause::kFeedbackLost);
    // on_error() is called from the real-time thread: joining the status
    // executor and the XML-RPC calls on a dead connection can take seconds,
    // so they run in a separate thread. read()/write() skip while it holds
    // the lifecycle mutex; on_activate() joins it first.
    join_cleanup();
    cleanup_thread_ = std::thread([this]() {
        stop_status_node();
        std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
        shutdown_robot_connection();
        RCLCPP_WARN(logger(), "FAIRINO SDK connection closed after the error.");
    });
    return hardware_interface::CallbackReturn::SUCCESS;
}

void FairinoHardwareInterface::shutdown_robot_connection()
{
    if (!_ptr_robot) {
        return;
    }
    std::lock_guard<std::mutex> lock(rpc_mutex_);
    const int end_rc = _ptr_robot->ServoMoveEnd(kServoJCommunicationType);
    const int stop_rc = _ptr_robot->StopMotion();//停止机器人
    const int close_rc = _ptr_robot->CloseRPC();//销毁实例，连接断开
    if (end_rc != 0 || stop_rc != 0 || close_rc != 0) {
        RCLCPP_WARN(logger(), "Shutdown: ServoMoveEnd=%d StopMotion=%d CloseRPC=%d", end_rc, stop_rc, close_rc);
    }
    retire_robot(std::move(_ptr_robot));
}

void FairinoHardwareInterface::retire_robot(std::unique_ptr<FRRobot> robot)
{
    // SDK 3.9.9: CloseRPC() only closes the sockets; the detached CNDE receive
    // thread (FRCNDEClient::RecvRobotStateThread) keeps using its FRRobot /
    // FRCNDEClient. Destroying the instance right after CloseRPC() crashed
    // ros2_control_node (SIGSEGV in RecvRobotStateThread, std::map insert)
    // on a hardware deactivate/activate cycle against SimMachine. Instances
    // are therefore kept alive until the process exits (one small object per
    // deactivation or error).
    static auto* retired = new std::vector<std::unique_ptr<FRRobot>>();
    static std::mutex retired_mutex;
    std::lock_guard<std::mutex> lock(retired_mutex);
    retired->push_back(std::move(robot));
}



hardware_interface::return_type FairinoHardwareInterface::read(const rclcpp::Time& /*time*/,const rclcpp::Duration& /*period*/)
{//从RTDE反馈数据中获取所需的位置，速度和扭矩信息
    std::unique_lock<std::mutex> lifecycle(lifecycle_mutex_, std::try_to_lock);
    if (!lifecycle.owns_lock() || !_ptr_robot) {
        return hardware_interface::return_type::OK;  // (de)activation in progress, nothing connected
    }
    const auto now = SteadyClock::now();
    ROBOT_STATE_PKG pkg;
    int returncode = _ptr_robot->GetRobotRealTimeState(&pkg);
    bool new_frame = false;
    if (returncode == 0) {
        const int64_t frame_key = robot_time_key(pkg);
        if (!have_frame_ || frame_key != last_frame_key_) {
            have_frame_ = true;
            last_frame_key_ = frame_key;
            last_frame_time_ = now;
            new_frame = true;
        }
        for (int i = 0; i < 6; ++i) {
            if (!std::isfinite(pkg.jt_cur_pos[i])) {
                returncode = ERR_OTHER;
                break;
            }
        }
    }
    const bool frame_fresh = have_frame_ && seconds_between(now, last_frame_time_) <= feedback_timeout_s_;

    if (returncode == 0 && frame_fresh) {
        for(int i=0;i<6;i++){
            _jnt_position_state[i] = pkg.jt_cur_pos[i] * kDegToRad;//注意单位转换，moveit统一用弧度
        }
        consecutive_read_errors_ = 0;
        last_good_read_time_ = now;
        last_read_rc_ = 0;

        // servoJCmdNum counts executed ServoJ points modulo 10000.
        if (flow_baseline_pending_.exchange(false) || !flow_have_baseline_) {
            flow_have_baseline_ = true;
            last_cmd_num_ = pkg.servoJCmdNum;
            executed_points_ = 0;
            sent_points_ = 0;
            consecutive_flow_skips_ = 0;
            flow_low_progress_frames_ = 0;
            flow_probe_active_ = false;
        } else if (new_frame) {
            const int delta = ((pkg.servoJCmdNum - last_cmd_num_) % 10000 + 10000) % 10000;
            last_cmd_num_ = pkg.servoJCmdNum;
            if (delta > 5000 || executed_points_ + delta > sent_points_) {
                // The controller cannot execute more points than were sent:
                // the counter went backwards or was reset (ServoMoveStart can
                // reset it after the baseline frame was taken). Counting the
                // phantom points would make the estimate negative and let the
                // real FIFO grow unseen (measured on SimMachine after a
                // hardware re-activation: 0.3-0.6 s permanent lag).
                executed_points_ = sent_points_;
                flow_rebaselines_.fetch_add(1);
            } else {
                executed_points_ += delta;
            }
            update_flow_probe(delta, now);
        }

        // Any controller fault or safety signal stops streaming immediately;
        // only a controlled recovery may re-arm it.
        if (streaming_state_.load() != RobotStatus::STREAMING_DISARMED && has_blocking_condition(pkg)) {
            disarm(DisarmCause::kControllerFault, pkg.main_code, pkg.sub_code);
        }
    } else {
        // Feedback is not trusted: keep the last good position, report the
        // code, stop streaming and finally hand the error to ros2_control.
        read_errors_.fetch_add(1);
        consecutive_read_errors_.fetch_add(1);
        last_read_rc_ = returncode != 0 ? returncode : ERR_SOCKET_RECV_FAILED;
        const double outage = seconds_between(now, last_good_read_time_);
        if (outage > feedback_timeout_s_) {
            disarm(DisarmCause::kFeedbackLost, last_read_rc_.load());
        }
        if (outage > feedback_error_timeout_s_) {
            RCLCPP_ERROR(logger(),
                "FAIRINO joint feedback lost for %.3f s (last code %d); returning ERROR to ros2_control.",
                outage, last_read_rc_.load());
            return hardware_interface::return_type::ERROR;
        }
    }

    if (returncode == 0 && snapshot_mutex_.try_lock()) {
        snapshot_ = pkg;
        snapshot_valid_ = true;
        snapshot_frame_stamp_ = last_frame_time_;
        frame_updates_ += new_frame ? 1 : 0;
        snapshot_mutex_.unlock();
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FairinoHardwareInterface::write(const rclcpp::Time& /*time*/,const rclcpp::Duration& /*period*/)
{
    std::unique_lock<std::mutex> lifecycle(lifecycle_mutex_, std::try_to_lock);
    if (!lifecycle.owns_lock() || !_ptr_robot) {
        return hardware_interface::return_type::OK;
    }
    const auto now = SteadyClock::now();
    double write_period = servoj_cmd_period_s_;
    if (have_last_write_) {
        const double measured = seconds_between(now, last_write_time_);
        write_period = std::max(measured, servoj_cmd_period_s_);
        if (histogram_mutex_.try_lock()) {
            const double ms = measured * 1000.0;
            const int bin = std::min(PeriodHistogram::kBins - 1,
                                     static_cast<int>(ms / PeriodHistogram::kBinWidthMs));
            histogram_.bins[bin] += 1;
            histogram_.samples += 1;
            histogram_.sum_ms += ms;
            histogram_.max_ms = std::max(histogram_.max_ms, ms);
            if (measured > 2.0 * servoj_cmd_period_s_) {
                histogram_.overruns += 1;
            }
            histogram_mutex_.unlock();
        }
    }
    last_write_time_ = now;
    have_last_write_ = true;

    if(_control_mode != 0){//位置控制模式
        RCLCPP_ERROR_ONCE(logger(), "Only position control is implemented by the FAIRINO hardware.");
        return hardware_interface::return_type::ERROR;
    }

    const uint8_t state = streaming_state_.load();
    if (state == RobotStatus::STREAMING_DISARMED) {
        // Nothing is sent. The references follow the robot so a later arm is
        // compared against the real position, and the command interface is
        // primed with the measured position: a controller that reads the last
        // command on activation then holds where the robot is.
        for (int j = 0; j < 6; ++j) {
            last_sent_command_[j] = _jnt_position_state[j];
            previous_command_[j] = _jnt_position_state[j];
            _jnt_position_command[j] = _jnt_position_state[j];
        }
        last_command_change_time_ = now;
        previous_change_interval_ = 1.0e9;
        sync_stable_cycles_ = 0;
        return hardware_interface::return_type::OK;
    }

    for (int j = 0; j < 6; ++j) {
        if (!std::isfinite(_jnt_position_command[j])) {
            disarm(DisarmCause::kNonFiniteCommand, 0, j);
            return hardware_interface::return_type::OK;
        }
    }

    if (state == RobotStatus::STREAMING_WAIT_SYNC) {
        // Arm only when the command matches the measured position AND has not
        // changed for kSyncStableCycles: a moving stream (e.g. a trajectory
        // that is still playing) is never armed, even if it passes through
        // the measured position.
        double max_error = 0.0;
        bool command_static = true;
        for (int j = 0; j < 6; ++j) {
            max_error = std::max(max_error, std::abs(_jnt_position_command[j] - _jnt_position_state[j]));
            command_static = command_static && _jnt_position_command[j] == previous_command_[j];
        }
        if (max_error <= sync_tolerance_rad_ && command_static) {
            ++sync_stable_cycles_;
        } else {
            sync_stable_cycles_ = 0;
        }
        if (max_error > sync_tolerance_rad_) {
            // Prime the command interface with the measured position; an
            // active controller overwrites it in its next update.
            for (int j = 0; j < 6; ++j) {
                _jnt_position_command[j] = _jnt_position_state[j];
            }
        }
        for (int j = 0; j < 6; ++j) {
            previous_command_[j] = _jnt_position_command[j];
        }
        if (sync_stable_cycles_ < kSyncStableCycles) {
            return hardware_interface::return_type::OK;
        }
        for (int j = 0; j < 6; ++j) {
            last_sent_command_[j] = _jnt_position_state[j];
        }
        last_command_change_time_ = now;
        previous_change_interval_ = 1.0e9;
        sync_stable_cycles_ = 0;
        uint8_t expected = RobotStatus::STREAMING_WAIT_SYNC;
        if (!streaming_state_.compare_exchange_strong(expected, RobotStatus::STREAMING_ARMED)) {
            return hardware_interface::return_type::OK;  // disarmed concurrently
        }
        disarm_cause_ = static_cast<int>(DisarmCause::kNone);
    }

    // Discontinuity guard on the controller command stream. A command change
    // may move a joint at most jump_guard_factor x v_max x T, where T is the
    // write period. Only a stream that was moving (its last two changes at
    // most 3 ms apart) may catch up after a delivery gap: topic-fed
    // controllers keep the last command when a message is delayed (10-20 ms
    // seen under CPU load) and then jump to the newest one; T is then the gap,
    // capped at jump_guard_max_gap. A stream that was holding (HOLD, a
    // trajectory controller at rest, just armed) never gets the gap allowance,
    // so a stale command after a switch, recovery or re-activation trips.
    bool changed = false;
    for (int j = 0; j < 6; ++j) {
        changed = changed || _jnt_position_command[j] != previous_command_[j];
    }
    if (changed) {
        const double since_change = seconds_between(now, last_command_change_time_);
        const bool stream_moving = previous_change_interval_ <= kActiveStreamInterval;
        const double allowance_period =
            std::min(stream_moving ? std::max(since_change, write_period) : write_period, jump_guard_max_gap_s_);
        for (int j = 0; j < 6; ++j) {
            const double step = std::abs(_jnt_position_command[j] - previous_command_[j]);
            const double limit = jump_guard_factor_ * max_joint_velocity_rad_s_[j] * allowance_period;
            if (step > limit) {
                jump_guard_trips_.fetch_add(1);
                disarm(DisarmCause::kJumpGuard, 0, j, step, limit);
                return hardware_interface::return_type::OK;
            }
        }
        previous_change_interval_ = since_change;
        last_command_change_time_ = now;
        for (int j = 0; j < 6; ++j) {
            previous_command_[j] = _jnt_position_command[j];
        }
    }

    // Flow control: keep the controller's ServoJ FIFO short.
    if (!flow_probe_active_ && flow_low_progress_frames_ >= kFlowProbeTriggerFrames &&
        seconds_between(now, flow_last_probe_) >= kFlowProbeInterval) {
        flow_probe_active_ = true;
        flow_probe_frames_ = 0;
        flow_probe_progress_ = false;
        flow_probes_.fetch_add(1);
    }
    if (flow_probe_active_) {
        flow_skips_.fetch_add(1);
        return hardware_interface::return_type::OK;
    }
    const int64_t queue = sent_points_ - executed_points_;
    queue_estimate_ = static_cast<int>(queue);
    if (servoj_max_queue_ > 0 && queue > servoj_max_queue_) {
        flow_skips_.fetch_add(1);
        if (++consecutive_flow_skips_ > kFlowRebaselineSkips) {
            // No executed progress for 0.5 s: either UDP points were lost (the
            // estimate stays "full" forever and streaming would dead-lock) or
            // the controller stalls with a full FIFO. Re-baselining resolves
            // the first; in the second it adds at most servoj_max_queue
            // points of latency. A shorter timeout re-baselined during normal
            // SimMachine stalls and accumulated a hidden backlog.
            sent_points_ = executed_points_;
            consecutive_flow_skips_ = 0;
            flow_rebaselines_.fetch_add(1);
        }
        return hardware_interface::return_type::OK;
    }
    consecutive_flow_skips_ = 0;

    // Per-point step limit: the controller executes every point within cmdT,
    // so a point may not ask for more than point_velocity_factor x v_max.
    // After a skipped cycle or a controller stall the robot lags briefly and
    // catches up instead of receiving one large step (seen on SimMachine as
    // main=1 sub=23 joint command speed faults).
    std::array<double, 6> target{};
    double ratio = 0.0;
    for (int j = 0; j < 6; ++j) {
        const double max_step = point_velocity_factor_ * max_joint_velocity_rad_s_[j] * servoj_cmd_period_s_;
        ratio = std::max(ratio, std::abs(_jnt_position_command[j] - last_sent_command_[j]) / max_step);
    }
    for (int j = 0; j < 6; ++j) {
        const double step = _jnt_position_command[j] - last_sent_command_[j];
        target[j] = ratio > 1.0 ? last_sent_command_[j] + step / ratio : _jnt_position_command[j];
    }
    if (ratio > 1.0) {
        point_limited_.fetch_add(1);
    }

    // A concurrent stop/recover may have disarmed since the state was read.
    if (streaming_state_.load() != RobotStatus::STREAMING_ARMED) {
        return hardware_interface::return_type::OK;
    }
    JointPos cmd;
    ExaxisPos extcmd{0,0,0,0};
    for(auto j=0;j<6;j++){
        cmd.jPos[j] = target[j] * kRadToDeg; //注意单位转换
    }
    const int returncode = _ptr_robot->ServoJ(
        &cmd, &extcmd, 0, 0, static_cast<float>(servoj_cmd_period_s_), 0, 0, 0,
        kServoJCommunicationType);
    servoj_calls_.fetch_add(1);
    sent_points_ += 1;
    last_servoj_rc_ = returncode;
    if(returncode != 0){
        // The controller rejected the command (fault, safety stop, out of
        // range...). Stop streaming; the code is published on
        // /fairino_robot/status for the servo state machine.
        servoj_errors_.fetch_add(1);
        disarm(DisarmCause::kServoJFailed, returncode);
        return hardware_interface::return_type::OK;
    }
    last_sent_command_ = target;
    return hardware_interface::return_type::OK;
}

void FairinoHardwareInterface::disarm(DisarmCause cause, int code, int joint, double value, double limit)
{
    const uint8_t previous = streaming_state_.exchange(RobotStatus::STREAMING_DISARMED);
    // Keep the first fault cause; explicit actions (stop, recovery) and the
    // placeholder causes (none, not started, awaiting sync) are overwritten.
    const auto current = static_cast<DisarmCause>(disarm_cause_.load());
    if (previous != RobotStatus::STREAMING_DISARMED || cause == DisarmCause::kRecovery ||
        cause == DisarmCause::kRecoveryFailed || cause == DisarmCause::kStopRequest ||
        current == DisarmCause::kNone || current == DisarmCause::kStartup ||
        current == DisarmCause::kAwaitingSync || current == DisarmCause::kRecovery) {
        disarm_code_ = code;
        disarm_joint_ = joint;
        disarm_value_ = value;
        disarm_limit_ = limit;
        disarm_cause_ = static_cast<int>(cause);
    }
}

void FairinoHardwareInterface::update_flow_probe(int executed_delta, SteadyClock::time_point now)
{
    // The estimate (sent - executed) drifts upwards when the controller drops
    // or merges points without counting them. With a drift of d the real FIFO
    // holds only servoj_max_queue + 1 - d points: on SimMachine the stream
    // settled at exactly one point per 4 ms frame (250 points/s, robot at a
    // quarter of the commanded speed) while the estimate read "full".
    // A genuinely slow controller with a full FIFO shows the same counts, so
    // the two are told apart by pausing: a full FIFO keeps executing, an
    // empty one does not. The first paused frame may still execute points
    // that were in flight, so only frames 2..kFlowProbeFrames are judged.
    if (flow_probe_active_) {
        ++flow_probe_frames_;
        if (flow_probe_frames_ >= 2 && executed_delta > 0) {
            flow_probe_progress_ = true;
        }
        if (flow_probe_frames_ >= kFlowProbeFrames) {
            if (!flow_probe_progress_) {
                sent_points_ = executed_points_;
                flow_drift_corrections_.fetch_add(1);
            }
            flow_probe_active_ = false;
            flow_last_probe_ = now;
            flow_low_progress_frames_ = 0;
        }
        return;
    }
    const bool full = servoj_max_queue_ > 0 && sent_points_ - executed_points_ >= servoj_max_queue_;
    flow_low_progress_frames_ = full && executed_delta <= kFlowLowProgress ? flow_low_progress_frames_ + 1 : 0;
}

void FairinoHardwareInterface::reset_flow_control()
{
    // The next realtime frame becomes the baseline of the executed counter.
    flow_baseline_pending_ = true;
}

std::string FairinoHardwareInterface::disarm_reason_text() const
{
    const auto cause = static_cast<DisarmCause>(disarm_cause_.load());
    const int code = disarm_code_.load();
    const int joint = disarm_joint_.load();
    std::ostringstream text;
    switch (cause) {
        case DisarmCause::kNone: text << "streaming"; break;
        case DisarmCause::kStartup: text << "not started"; break;
        case DisarmCause::kServoMoveStartFailed: text << "ServoMoveStart(UDP) failed, code " << code; break;
        case DisarmCause::kControllerFault:
            text << "controller fault/safety signal (main=" << code << ", sub=" << joint << ")";
            break;
        case DisarmCause::kServoJFailed: text << "ServoJ returned code " << code; break;
        case DisarmCause::kNonFiniteCommand: text << "non-finite command on j" << (joint + 1); break;
        case DisarmCause::kJumpGuard:
            text << "jump guard on j" << (joint + 1) << ": step " << disarm_value_.load() << " rad > "
                 << disarm_limit_.load() << " rad";
            break;
        case DisarmCause::kFeedbackLost: text << "joint feedback lost, last code " << code; break;
        case DisarmCause::kRecovery: text << "recovery in progress"; break;
        case DisarmCause::kRecoveryFailed: text << "recovery failed, code " << code; break;
        case DisarmCause::kStopRequest: text << "stop requested"; break;
        case DisarmCause::kAwaitingSync:
            text << "waiting until the command is static at the measured position";
            break;
    }
    return text.str();
}

bool FairinoHardwareInterface::has_blocking_condition(const ROBOT_STATE_PKG& pkg)
{
    return pkg.main_code != 0 || pkg.sub_code != 0 || pkg.EmergencyStop != 0 || pkg.btnBoxStopSignal != 0 ||
           pkg.safety_stop0_state != 0 || pkg.safety_stop1_state != 0 || pkg.collisionState != 0 ||
           pkg.safetyDoorAlarm != 0 || pkg.safetyPlaneAlarm != 0 || pkg.safetyDataState != 0 ||
           pkg.driverAlarm != 0 || pkg.driverError != 0 || pkg.outSoftLimitError != 0 ||
           pkg.alarmRebootRobot != 0 || pkg.alarmCheckEmergStopBtn != 0 || pkg.rbtEnableState == 0 ||
           pkg.ctrlBoxError != 0 ||
           std::any_of(std::begin(pkg.driverComError), std::end(pkg.driverComError),
                       [](uint8_t value) { return value != 0; });
}

std::string FairinoHardwareInterface::blocking_condition(const ROBOT_STATE_PKG& pkg)
{
    std::string reason;
    auto add = [&reason](bool active, const char* text) {
        if (active) {
            reason += reason.empty() ? text : std::string(", ") + text;
        }
    };
    if (pkg.main_code != 0 || pkg.sub_code != 0) {
        reason = "main=" + std::to_string(pkg.main_code) + " sub=" + std::to_string(pkg.sub_code);
    }
    add(pkg.EmergencyStop != 0, "emergency stop");
    add(pkg.btnBoxStopSignal != 0, "button box stop");
    add(pkg.safety_stop0_state != 0, "safety stop SI0");
    add(pkg.safety_stop1_state != 0, "safety stop SI1");
    add(pkg.collisionState != 0, "collision");
    add(pkg.safetyDoorAlarm != 0, "safety door");
    add(pkg.safetyPlaneAlarm != 0, "safety plane");
    add(pkg.safetyDataState != 0, "safety data abnormal");
    add(pkg.driverAlarm != 0, "driver alarm");
    add(pkg.driverError != 0, "driver communication error");
    add(pkg.outSoftLimitError != 0, "soft limit exceeded");
    // abnomalStop is not blocking: SimMachine 3.9.9 keeps it at 1 after a
    // successful ResetAllError() (main/sub = 0), so it would block forever.
    add(pkg.alarmRebootRobot != 0, "power cycle required");
    add(pkg.alarmCheckEmergStopBtn != 0, "check E-stop button");
    add(pkg.rbtEnableState == 0, "robot disabled");
    add(pkg.ctrlBoxError != 0, "control box error");
    add(std::any_of(std::begin(pkg.driverComError), std::end(pkg.driverComError),
                    [](uint8_t value) { return value != 0; }),
        "drive communication error");
    return reason;
}

// ---------------------------------------------------------------------------
// Status node
// ---------------------------------------------------------------------------

void FairinoHardwareInterface::start_status_node()
{
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);
    status_node_ = std::make_shared<rclcpp::Node>("fairino_robot", options);
    timer_group_ = status_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = status_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // The node keeps only weak references to callback groups; they must be
    // stored as members or their entities are never executed.
    stop_group_ = status_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    status_pub_ = status_node_->create_publisher<RobotStatus>("~/status", rclcpp::QoS(10));
    events_pub_ = status_node_->create_publisher<AuditEvent>(
        "~/events", rclcpp::QoS(rclcpp::KeepLast(200)).reliable().transient_local());
    last_status_time_ = SteadyClock::now();
    last_status_frame_updates_ = 0;
    status_timer_ = status_node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / status_rate_hz_), [this]() { publish_status(); }, timer_group_);

    using std::placeholders::_1;
    using std::placeholders::_2;
    recover_srv_ = status_node_->create_service<RecoverRobot>(
        "~/recover", std::bind(&FairinoHardwareInterface::on_recover, this, _1, _2),
        rmw_qos_profile_services_default, service_group_);
    speed_srv_ = status_node_->create_service<SetGlobalSpeed>(
        "~/set_global_speed", std::bind(&FairinoHardwareInterface::on_set_global_speed, this, _1, _2),
        rmw_qos_profile_services_default, service_group_);
    safety_speed_srv_ = status_node_->create_service<SetSafetySpeed>(
        "~/set_safety_speed", std::bind(&FairinoHardwareInterface::on_set_safety_speed, this, _1, _2),
        rmw_qos_profile_services_default, service_group_);
    stop_srv_ = status_node_->create_service<Trigger>(
        "~/stop", std::bind(&FairinoHardwareInterface::on_stop, this, _1, _2),
        rmw_qos_profile_services_default, stop_group_);

    status_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 3);
    status_executor_->add_node(status_node_);
    status_thread_ = std::thread([executor = status_executor_]() { executor->spin(); });
}

void FairinoHardwareInterface::stop_status_node()
{
    if (status_executor_) {
        status_executor_->cancel();
    }
    if (status_thread_.joinable()) {
        status_thread_.join();
    }
    if (status_executor_ && status_node_) {
        status_executor_->remove_node(status_node_);
    }
    status_timer_.reset();
    recover_srv_.reset();
    speed_srv_.reset();
    safety_speed_srv_.reset();
    stop_srv_.reset();
    status_pub_.reset();
    events_pub_.reset();
    status_executor_.reset();
    timer_group_.reset();
    service_group_.reset();
    stop_group_.reset();
    status_node_.reset();
}

bool FairinoHardwareInterface::copy_snapshot(ROBOT_STATE_PKG& pkg, SteadyClock::time_point& stamp,
                                             uint64_t& frame_updates)
{
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!snapshot_valid_) {
        return false;
    }
    pkg = snapshot_;
    stamp = snapshot_frame_stamp_;
    frame_updates = frame_updates_;
    return true;
}

void FairinoHardwareInterface::publish_status()
{
    if (!status_pub_ || !_ptr_robot) {
        return;
    }
    const auto now = SteadyClock::now();
    RobotStatus msg;
    msg.header.stamp = status_node_->now();
    msg.robot_model = robot_model_;
    msg.controller_version = controller_version_;
    msg.web_version = web_version_;

    int com_state = 1;
    msg.sdk_com_state_return_code = _ptr_robot->GetSDKComState(&com_state);
    msg.sdk_com_state = com_state;

    ROBOT_STATE_PKG pkg;
    SteadyClock::time_point frame_stamp;
    uint64_t frame_updates = 0;
    const bool have_snapshot = copy_snapshot(pkg, frame_stamp, frame_updates);
    msg.realtime_state_return_code = have_snapshot ? 0 : last_read_rc_.load();
    msg.realtime_state_age_s = have_snapshot ? seconds_between(now, frame_stamp) : -1.0;
    msg.connected = have_snapshot && com_state == 0 && msg.realtime_state_age_s <= feedback_timeout_s_;

    if (have_snapshot) {
        msg.robot_enabled = pkg.rbtEnableState != 0;
        msg.robot_mode = pkg.robot_mode;
        msg.motion_state = pkg.robot_state;
        msg.program_state = pkg.program_state;
        msg.motion_done = pkg.motion_done;
        msg.main_error_code = pkg.main_code;
        msg.sub_error_code = pkg.sub_code;
        msg.emergency_stop = pkg.EmergencyStop != 0;
        msg.button_box_stop = pkg.btnBoxStopSignal != 0;
        msg.safety_stop0 = pkg.safety_stop0_state != 0;
        msg.safety_stop1 = pkg.safety_stop1_state != 0;
        msg.collision = pkg.collisionState != 0;
        msg.safety_door_alarm = pkg.safetyDoorAlarm != 0;
        msg.safety_plane_alarm = pkg.safetyPlaneAlarm != 0;
        msg.safety_data_abnormal = pkg.safetyDataState != 0;
        msg.driver_alarm_axis = pkg.driverAlarm;
        msg.driver_error_axis = pkg.driverError;
        for (int j = 0; j < 6; ++j) {
            msg.driver_com_error[j] = pkg.driverComError[j];
            msg.collision_level[j] = pkg.collisionLevel[j];
            msg.joint_position_deg[j] = pkg.jt_cur_pos[j];
            msg.joint_velocity_deg_s[j] = pkg.actual_qd[j];
            msg.target_joint_velocity_deg_s[j] = pkg.targetJointVel[j];
            msg.flange_pose[j] = pkg.flange_cur_pos[j];
            msg.tcp_pose[j] = pkg.tl_cur_pos[j];
        }
        msg.tool_number = pkg.tool;
        msg.user_frame_number = pkg.user;
        msg.out_soft_limit_error = pkg.outSoftLimitError != 0;
        msg.abnormal_stop = pkg.abnomalStop != 0;
        msg.alarm_reboot_robot = pkg.alarmRebootRobot;
        msg.alarm_check_estop_button = pkg.alarmCheckEmergStopBtn != 0;
        msg.singular_pose = pkg.strangePosFlag != 0;
        msg.motion_alarm = pkg.motonAlarm;
        msg.ctrl_box_error = pkg.ctrlBoxError;
        msg.udp_cmd_state = pkg.udpCmdState;
        // The realtime package reports the global speed as a fraction 0..1
        // (SetSpeed(50) reads back 0.5 on controller 3.9.9).
        msg.speed_scale_manual = pkg.speedScaleManual * 100.0;
        msg.speed_scale_auto = pkg.speedScaleAuto * 100.0;
        msg.servoj_cmd_num = pkg.servoJCmdNum;
    }

    msg.streaming_state = streaming_state_.load();
    msg.streaming_reason = disarm_reason_text();
    msg.servoj_cmd_period_s = servoj_cmd_period_s_;
    msg.last_servoj_return_code = last_servoj_rc_.load();
    msg.servoj_calls = servoj_calls_.load();
    msg.servoj_errors = servoj_errors_.load();
    msg.servoj_queue_estimate = queue_estimate_.load();
    msg.servoj_max_queue = servoj_max_queue_;
    msg.servoj_flow_skips = flow_skips_.load();
    msg.servoj_flow_rebaselines = flow_rebaselines_.load();
    msg.servoj_flow_probes = flow_probes_.load();
    msg.servoj_flow_drift_corrections = flow_drift_corrections_.load();
    msg.servoj_point_limited = point_limited_.load();
    msg.jump_guard_trips = jump_guard_trips_.load();
    msg.last_read_return_code = last_read_rc_.load();
    msg.read_errors = read_errors_.load();
    msg.consecutive_read_errors = consecutive_read_errors_.load();
    msg.joint_feedback_age_s = have_snapshot ? seconds_between(now, frame_stamp) : -1.0;
    const double window = seconds_between(now, last_status_time_);
    if (window > 0.0) {
        msg.joint_feedback_rate_hz =
            static_cast<double>(frame_updates - last_status_frame_updates_) / window;
    }
    last_status_frame_updates_ = frame_updates;
    last_status_time_ = now;

    PeriodHistogram histogram;
    {
        std::lock_guard<std::mutex> lock(histogram_mutex_);
        histogram = histogram_;
        histogram_.reset();
    }
    if (histogram.samples > 0) {
        msg.write_period_mean_ms = histogram.sum_ms / static_cast<double>(histogram.samples);
        msg.write_period_max_ms = histogram.max_ms;
        const uint64_t target = static_cast<uint64_t>(std::ceil(0.99 * static_cast<double>(histogram.samples)));
        uint64_t cumulative = 0;
        for (int bin = 0; bin < PeriodHistogram::kBins; ++bin) {
            cumulative += histogram.bins[bin];
            if (cumulative >= target) {
                msg.write_period_p99_ms = (bin + 1) * PeriodHistogram::kBinWidthMs;
                break;
            }
        }
        msg.write_period_overruns = histogram.overruns;
    }
    msg.safety_config_writes_allowed = allow_safety_config_writes_;
    status_pub_->publish(msg);
}

void FairinoHardwareInterface::audit(const std::string& category, const std::string& action,
                                     const std::string& old_value, const std::string& new_value,
                                     int result_code, bool success, const std::string& message)
{
    RCLCPP_WARN(logger(), "AUDIT [%s] %s: '%s' -> '%s' code=%d success=%s %s", category.c_str(),
                action.c_str(), old_value.c_str(), new_value.c_str(), result_code,
                success ? "true" : "false", message.c_str());
    if (!events_pub_ || !status_node_) {
        return;
    }
    AuditEvent event;
    event.stamp = status_node_->now();
    event.source = "fairino_hardware";
    event.category = category;
    event.action = action;
    event.old_value = old_value;
    event.new_value = new_value;
    event.result_code = result_code;
    event.success = success;
    event.message = message;
    events_pub_->publish(event);
}

template <typename Predicate>
bool FairinoHardwareInterface::wait_for_state(Predicate predicate, double timeout_s, ROBOT_STATE_PKG* last)
{
    const auto deadline = SteadyClock::now() + std::chrono::duration<double>(timeout_s);
    ROBOT_STATE_PKG pkg;
    SteadyClock::time_point stamp;
    uint64_t updates = 0;
    while (true) {
        const bool have = copy_snapshot(pkg, stamp, updates);
        if (have && last != nullptr) {
            *last = pkg;
        }
        if (have && predicate(pkg)) {
            return true;
        }
        if (SteadyClock::now() >= deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ---------------------------------------------------------------------------
// Services
// ---------------------------------------------------------------------------

void FairinoHardwareInterface::on_recover(const std::shared_ptr<RecoverRobot::Request> request,
                                          std::shared_ptr<RecoverRobot::Response> response)
{
    const auto started = SteadyClock::now();
    std::lock_guard<std::mutex> lock(rpc_mutex_);
    const std::string who = request->requested_by.empty() ? "unknown" : request->requested_by;
    audit("recovery", "recover requested", "", "", 0, true,
          "by " + who + ": " + request->reason + " (reset=" + std::to_string(request->reset_errors) +
          " enable=" + std::to_string(request->enable_robot) + " restart_servo=" +
          std::to_string(request->restart_servo) + ")");

    const uint64_t generation = stop_generation_.load();
    auto fail = [&](const std::string& step, int code, const std::string& message) {
        response->success = false;
        response->failed_step = step;
        response->failed_step_code = code;
        response->message = message;
        response->duration_s = seconds_between(SteadyClock::now(), started);
        if (stop_generation_.load() == generation) {
            disarm(DisarmCause::kRecoveryFailed, code);
        }
        audit("recovery", step, "", "", code, false, message);
    };
    auto stopped_meanwhile = [&]() { return stop_generation_.load() != generation; };
    auto log_step = [&](const std::string& step, int code, bool success) {
        response->steps.push_back(step + " -> " + std::to_string(code));
        audit("recovery", step, "", "", code, success, "");
    };

    if (!_ptr_robot) {
        fail("connection", ERR_OTHER, "FAIRINO SDK is not connected");
        return;
    }

    // 1-2. Streaming stops first; the active controller may keep writing.
    disarm(DisarmCause::kRecovery);
    response->steps.push_back("ServoJ streaming disarmed");

    // 3-4. End servo mode and stop any motion. Failures are reported but do
    // not stop the sequence: a faulted controller may refuse both.
    int rc = _ptr_robot->ServoMoveEnd(kServoJCommunicationType);
    log_step("ServoMoveEnd(UDP)", rc, rc == 0);
    rc = _ptr_robot->StopMotion();
    log_step("StopMotion()", rc, rc == 0);

    // 5. Standstill confirmation from the measured joint velocities.
    ROBOT_STATE_PKG pkg{};
    const double standstill = standstill_velocity_deg_s_;
    auto stopped = [standstill](const ROBOT_STATE_PKG& state) {
        if (state.robot_state == 2) {
            return false;
        }
        for (int j = 0; j < 6; ++j) {
            if (std::abs(state.actual_qd[j]) > standstill) {
                return false;
            }
        }
        return true;
    };
    if (!wait_for_state(stopped, 3.0, &pkg)) {
        fail("wait standstill", ERR_WAIT_TIMEOUT, "robot did not report standstill within 3 s");
        return;
    }
    response->steps.push_back("standstill confirmed");

    // 6-7. Reset and verify that both sources report no fault: the realtime
    // package and GetRobotErrorCode(). On SimMachine 3.9.9 GetRobotErrorCode()
    // lagged the realtime codes by more than 0.5 s after a reset (and returned
    // 0/0 right after connecting while the controller reported 1/17), so both
    // are polled for up to 2 s.
    int main_code = pkg.main_code;
    int sub_code = pkg.sub_code;
    if (request->reset_errors) {
        rc = _ptr_robot->ResetAllError();
        log_step("ResetAllError()", rc, rc == 0);
        if (rc != 0) {
            fail("ResetAllError()", rc, "ResetAllError() failed");
            return;
        }
    }
    bool cleared = false;
    const auto verify_deadline = SteadyClock::now() + std::chrono::seconds(2);
    do {
        rc = _ptr_robot->GetRobotErrorCode(&main_code, &sub_code);
        wait_for_state([](const ROBOT_STATE_PKG&) { return true; }, 0.05, &pkg);
        cleared = rc == 0 && main_code == 0 && sub_code == 0 && pkg.main_code == 0 && pkg.sub_code == 0;
        if (!cleared) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } while (!cleared && SteadyClock::now() < verify_deadline);
    response->main_error_code = pkg.main_code != 0 ? pkg.main_code : main_code;
    response->sub_error_code = pkg.main_code != 0 ? pkg.sub_code : sub_code;
    log_step("GetRobotErrorCode() main=" + std::to_string(main_code) + " sub=" + std::to_string(sub_code) +
                 ", realtime main=" + std::to_string(pkg.main_code) + " sub=" + std::to_string(pkg.sub_code),
             rc, cleared);
    if (rc != 0) {
        fail("GetRobotErrorCode()", rc, "could not read the controller error code");
        return;
    }
    if (!cleared) {
        fail("verify cleared", 0, "controller fault remains: main=" + std::to_string(response->main_error_code) +
                                      " sub=" + std::to_string(response->sub_error_code));
        return;
    }

    // 8. Enable only when requested and only after the fault cleared.
    wait_for_state([](const ROBOT_STATE_PKG&) { return true; }, 0.1, &pkg);
    if (pkg.rbtEnableState == 0) {
        if (!request->enable_robot) {
            response->robot_enabled = false;
            fail("robot disabled", 101, "robot is disabled and enabling was not requested");
            return;
        }
        rc = _ptr_robot->RobotEnable(1);
        log_step("RobotEnable(1)", rc, rc == 0);
        if (rc != 0) {
            fail("RobotEnable(1)", rc, "RobotEnable(1) failed");
            return;
        }
        if (!wait_for_state([](const ROBOT_STATE_PKG& state) { return state.rbtEnableState != 0; }, 3.0, &pkg)) {
            fail("wait enabled", ERR_WAIT_TIMEOUT, "robot did not report enabled within 3 s");
            return;
        }
    }
    response->robot_enabled = true;

    const std::string blocking = blocking_condition(pkg);
    if (!blocking.empty()) {
        fail("safety signals", 0, "blocking condition remains: " + blocking);
        return;
    }

    // 9. Restart ServoJ mode; streaming is armed by write() once the active
    // controller commands the measured position.
    if (stopped_meanwhile()) {
        fail("stop requested", 0, "a stop was requested during the recovery; streaming stays disarmed");
        return;
    }
    if (request->restart_servo) {
        rc = _ptr_robot->ServoMoveStart(kServoJCommunicationType);
        reset_flow_control();
        log_step("ServoMoveStart(UDP)", rc, rc == 0);
        if (rc != 0) {
            fail("ServoMoveStart(UDP)", rc, "ServoMoveStart(UDP) failed");
            return;
        }
        last_servoj_rc_ = 0;
        disarm_cause_ = static_cast<int>(DisarmCause::kAwaitingSync);
        uint8_t expected = RobotStatus::STREAMING_DISARMED;
        streaming_state_.compare_exchange_strong(expected, RobotStatus::STREAMING_WAIT_SYNC);
        if (stopped_meanwhile()) {
            // on_stop() increments the generation before it disarms, so a stop
            // racing with the line above is caught here.
            disarm(DisarmCause::kStopRequest);
            fail("stop requested", 0, "a stop was requested during the recovery; streaming stays disarmed");
            return;
        }
        response->steps.push_back("streaming WAIT_SYNC (arms when the command is static at the measured position)");
    }

    response->success = true;
    response->duration_s = seconds_between(SteadyClock::now(), started);
    response->message = "recovery complete";
    audit("recovery", "recover complete", "", request->restart_servo ? "WAIT_SYNC" : "DISARMED", 0, true,
          "duration " + std::to_string(response->duration_s) + " s");
}

void FairinoHardwareInterface::on_set_global_speed(const std::shared_ptr<SetGlobalSpeed::Request> request,
                                                   std::shared_ptr<SetGlobalSpeed::Response> response)
{
    const std::string action = request->instant ? "SetSpeedInstant" : "SetSpeed";
    const std::string who = request->requested_by.empty() ? "unknown" : request->requested_by;
    ROBOT_STATE_PKG pkg{};
    wait_for_state([](const ROBOT_STATE_PKG&) { return true; }, 0.2, &pkg);
    response->old_speed_scale_manual = pkg.speedScaleManual * 100.0;
    response->old_speed_scale_auto = pkg.speedScaleAuto * 100.0;
    const std::string old_value = speed_text(pkg);

    if (!request->operator_confirmed) {
        response->message = "refused: operator confirmation missing";
        audit("speed", action, old_value, std::to_string(request->percent) + "%", 0, false,
              "by " + who + ": " + response->message);
        return;
    }
    if (request->percent < 1 || request->percent > 100) {
        response->message = "refused: percent must be within 1..100";
        audit("speed", action, old_value, std::to_string(request->percent) + "%", ERR_PARAM_VALUE, false,
              "by " + who + ": " + response->message);
        return;
    }
    if (!_ptr_robot) {
        response->message = "FAIRINO SDK is not connected";
        return;
    }

    int rc = 0;
    {
        std::lock_guard<std::mutex> lock(rpc_mutex_);
        rc = request->instant ? _ptr_robot->SetSpeedInstant(request->percent)
                              : _ptr_robot->SetSpeed(request->percent);
    }
    response->sdk_return_code = rc;
    const double requested = request->percent;
    const bool changed = wait_for_state(
        [requested](const ROBOT_STATE_PKG& state) {
            const double active = state.robot_mode == 1 ? state.speedScaleManual : state.speedScaleAuto;
            return std::abs(active * 100.0 - requested) < 0.5;
        },
        1.0, &pkg);
    response->new_speed_scale_manual = pkg.speedScaleManual * 100.0;
    response->new_speed_scale_auto = pkg.speedScaleAuto * 100.0;
    response->success = rc == 0;
    response->message = rc != 0 ? action + " failed"
                        : changed ? action + " applied and read back"
                                  : action + " returned 0 but the realtime speed scale did not change within 1 s";
    audit("speed", action + "(" + std::to_string(request->percent) + ")", old_value, speed_text(pkg),
          rc, rc == 0, "by " + who + ": " + request->reason + "; " + response->message);
}

void FairinoHardwareInterface::on_set_safety_speed(const std::shared_ptr<SetSafetySpeed::Request> request,
                                                   std::shared_ptr<SetSafetySpeed::Response> response)
{
    const std::string who = request->requested_by.empty() ? "unknown" : request->requested_by;
    std::ostringstream requested;
    requested << "enable=" << request->enable << " maxTCPVel=" << request->max_tcp_velocity_mm_s
              << "mm/s strategy=" << request->strategy << " maxJointVel=[";
    for (int j = 0; j < 6; ++j) {
        requested << (j ? "," : "") << request->max_joint_velocity_deg_s[j];
    }
    requested << "]deg/s";
    const std::string old_value = "not readable via SDK 3.9.9 (verify in WebApp)";

    auto refuse = [&](const std::string& message) {
        response->success = false;
        response->message = "refused: " + message;
        audit("safety_config", "SetVelReducePara", old_value, requested.str(), 0, false,
              "by " + who + ": " + response->message);
    };
    if (!allow_safety_config_writes_) {
        refuse("hardware parameter allow_safety_config_writes is false");
        return;
    }
    if (!request->operator_confirmed) {
        refuse("operator confirmation missing");
        return;
    }
    if (request->enable < 0 || request->enable > 2 || request->strategy < 0 || request->strategy > 2) {
        refuse("enable and strategy must be within 0..2");
        return;
    }
    if (request->strategy == SetSafetySpeed::Request::STRATEGY_AUTO_LIMIT &&
        request->enable != SetSafetySpeed::Request::ENABLE_MANUAL_MODE) {
        refuse("automatic speed limiting is only available with enable=1 (manual mode)");
        return;
    }
    if (!(request->max_tcp_velocity_mm_s >= 0.0 && request->max_tcp_velocity_mm_s <= 1000.0)) {
        refuse("maxTCPVel must be within 0..1000 mm/s");
        return;
    }
    for (double value : request->max_joint_velocity_deg_s) {
        if (!(value > 0.0 && value <= 180.0)) {
            refuse("joint speed thresholds must be within (0, 180] deg/s");
            return;
        }
    }
    if (!_ptr_robot) {
        refuse("FAIRINO SDK is not connected");
        return;
    }

    std::lock_guard<std::mutex> lock(rpc_mutex_);
    if (!request->safety_password.empty()) {
        response->password_check_return_code = _ptr_robot->SafetyOPPasswordCheck(0, request->safety_password);
        audit("safety_config", "SafetyOPPasswordCheck(0)", "", "", response->password_check_return_code,
              response->password_check_return_code == 0, "by " + who);
        if (response->password_check_return_code != 0) {
            response->message = "safety password check failed";
            return;
        }
    }
    std::vector<double> joints(request->max_joint_velocity_deg_s.begin(), request->max_joint_velocity_deg_s.end());
    const int rc = _ptr_robot->SetVelReducePara(request->enable, request->max_tcp_velocity_mm_s,
                                                request->strategy, joints);
    response->sdk_return_code = rc;
    response->success = rc == 0;
    response->message = rc == 0 ? "SetVelReducePara applied; verify the WebApp Safety Speed page"
                                : "SetVelReducePara failed";
    audit("safety_config", "SetVelReducePara", old_value, requested.str(), rc, rc == 0,
          "by " + who + ": " + request->reason + "; " + response->message);
}

void FairinoHardwareInterface::on_stop(const std::shared_ptr<Trigger::Request> /*request*/,
                                       std::shared_ptr<Trigger::Response> response)
{
    // Generation first, then disarm: an in-flight recovery checks the
    // generation after re-arming and backs off (it cannot win the race).
    stop_generation_.fetch_add(1);
    disarm(DisarmCause::kStopRequest);
    if (!_ptr_robot) {
        response->success = false;
        response->message = "FAIRINO SDK is not connected; streaming disarmed";
        return;
    }
    std::lock_guard<std::mutex> lock(rpc_mutex_);
    const int end_rc = _ptr_robot->ServoMoveEnd(kServoJCommunicationType);
    const int stop_rc = _ptr_robot->StopMotion();
    response->success = stop_rc == 0;
    response->message = "streaming disarmed; ServoMoveEnd=" + std::to_string(end_rc) +
                        " StopMotion=" + std::to_string(stop_rc);
    audit("recovery", "stop", "", "DISARMED", stop_rc, stop_rc == 0, response->message);
}


}//end namesapce

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface, hardware_interface::SystemInterface)
