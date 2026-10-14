#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "libfairino/include/robot.h"
#include "libfairino/include/robot_error.h"

#include "fairino_msgs/msg/audit_event.hpp"
#include "fairino_msgs/msg/robot_status.hpp"
#include "fairino_msgs/srv/recover_robot.hpp"
#include "fairino_msgs/srv/set_global_speed.hpp"
#include "fairino_msgs/srv/set_safety_speed.hpp"
#include "std_srvs/srv/trigger.hpp"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"

namespace fairino_hardware
{

/// Write-period histogram shared between write() and the status thread.
/// Bins are 10 us wide up to 20 ms; the last bin collects everything above.
struct PeriodHistogram {
  static constexpr int kBins = 2001;
  static constexpr double kBinWidthMs = 0.01;
  std::array<uint32_t, kBins> bins{};
  uint64_t samples = 0;
  double sum_ms = 0.0;
  double max_ms = 0.0;
  uint64_t overruns = 0;
  void reset() { bins.fill(0); samples = 0; sum_ms = 0.0; max_ms = 0.0; overruns = 0; }
};

/// FAIRINO ros2_control system. It is the only owner of the FAIRINO SDK
/// connection: joint feedback and ServoJ streaming run in read()/write(), while
/// status publishing, recovery and speed configuration run in an internal
/// "fairino_robot" node served by a separate executor thread.
class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  ~FairinoHardwareInterface() override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using RobotStatus = fairino_msgs::msg::RobotStatus;
  using AuditEvent = fairino_msgs::msg::AuditEvent;
  using RecoverRobot = fairino_msgs::srv::RecoverRobot;
  using SetGlobalSpeed = fairino_msgs::srv::SetGlobalSpeed;
  using SetSafetySpeed = fairino_msgs::srv::SetSafetySpeed;
  using Trigger = std_srvs::srv::Trigger;
  using SteadyClock = std::chrono::steady_clock;

  /// Reason codes for leaving the ARMED streaming state. They are set from
  /// the real-time thread, so only atomics are used to record them.
  enum class DisarmCause : int {
    kNone = 0,
    kStartup,
    kServoMoveStartFailed,
    kControllerFault,
    kServoJFailed,
    kNonFiniteCommand,
    kJumpGuard,
    kFeedbackLost,
    kRecovery,
    kRecoveryFailed,
    kStopRequest,
    kAwaitingSync,
  };

  // ---- configuration -----------------------------------------------------
  void load_parameters();
  std::string hardware_parameter(const std::string& name, const std::string& fallback) const;

  // ---- connection lifecycle ----------------------------------------------
  void start_status_node();
  void stop_status_node();
  void shutdown_robot_connection();
  void join_cleanup();

  // ---- streaming state ---------------------------------------------------
  void disarm(DisarmCause cause, int code = 0, int joint = -1, double value = 0.0, double limit = 0.0);
  std::string disarm_reason_text() const;
  /// True when the realtime package reports a condition under which ServoJ
  /// must not be streamed (allocation free, used in read()).
  static bool has_blocking_condition(const ROBOT_STATE_PKG& pkg);
  /// Human readable description of the same conditions; empty when none.
  static std::string blocking_condition(const ROBOT_STATE_PKG& pkg);

  // ---- status thread -----------------------------------------------------
  void publish_status();
  bool copy_snapshot(ROBOT_STATE_PKG& pkg, SteadyClock::time_point& stamp, uint64_t& frame_updates);
  void audit(const std::string& category, const std::string& action, const std::string& old_value,
             const std::string& new_value, int result_code, bool success, const std::string& message);

  // ---- services ----------------------------------------------------------
  void on_recover(const std::shared_ptr<RecoverRobot::Request> request,
                  std::shared_ptr<RecoverRobot::Response> response);
  void on_set_global_speed(const std::shared_ptr<SetGlobalSpeed::Request> request,
                           std::shared_ptr<SetGlobalSpeed::Response> response);
  void on_set_safety_speed(const std::shared_ptr<SetSafetySpeed::Request> request,
                           std::shared_ptr<SetSafetySpeed::Response> response);
  void on_stop(const std::shared_ptr<Trigger::Request> request,
               std::shared_ptr<Trigger::Response> response);
  /// Polls the realtime snapshot until predicate(pkg) holds or timeout expires.
  template <typename Predicate>
  bool wait_for_state(Predicate predicate, double timeout_s, ROBOT_STATE_PKG* last = nullptr);

  // ---- ros2_control buffers ----------------------------------------------
  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  int _control_mode;

  // ---- parameters --------------------------------------------------------
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;
  double servoj_cmd_period_s_ = 0.001;
  double status_rate_hz_ = 50.0;
  double feedback_timeout_s_ = 0.1;
  double feedback_error_timeout_s_ = 0.5;
  int realtime_state_period_ms_ = 0;
  std::array<double, 6> max_joint_velocity_rad_s_{};
  double jump_guard_factor_ = 2.0;
  double sync_tolerance_rad_ = 0.002;
  double standstill_velocity_deg_s_ = 0.5;
  int servoj_max_queue_ = 8;
  double point_velocity_factor_ = 1.0;
  double jump_guard_max_gap_s_ = 0.05;
  bool allow_safety_config_writes_ = false;

  // ---- SDK ---------------------------------------------------------------
  std::unique_ptr<FRRobot> _ptr_robot;
  /// Keeps a closed SDK instance alive for the rest of the process (see .cpp).
  static void retire_robot(std::unique_ptr<FRRobot> robot);
  /// Serializes XML-RPC calls issued from service/status threads. The
  /// real-time thread never takes this lock.
  std::mutex rpc_mutex_;
  std::string robot_model_;
  std::string controller_version_;
  std::string web_version_;

  // ---- real-time shared state --------------------------------------------
  std::atomic<uint8_t> streaming_state_{RobotStatus::STREAMING_DISARMED};
  std::atomic<int> disarm_cause_{static_cast<int>(DisarmCause::kStartup)};
  std::atomic<int> disarm_code_{0};
  std::atomic<int> disarm_joint_{-1};
  std::atomic<double> disarm_value_{0.0};
  std::atomic<double> disarm_limit_{0.0};
  /// Incremented by every stop request; a recovery that sees it change
  /// aborts and never re-arms streaming.
  std::atomic<uint64_t> stop_generation_{0};
  /// Held by lifecycle callbacks while _ptr_robot is created/destroyed;
  /// read()/write() only try_lock it and skip the cycle otherwise.
  std::mutex lifecycle_mutex_;
  /// on_error() runs in the real-time thread; the slow cleanup runs here.
  std::thread cleanup_thread_;

  std::array<double, 6> last_sent_command_{};
  std::array<double, 6> previous_command_{};
  SteadyClock::time_point last_command_change_time_{};
  double previous_change_interval_ = 1.0e9;  // s between the last two command changes
  int sync_stable_cycles_ = 0;
  bool have_last_write_ = false;
  SteadyClock::time_point last_write_time_{};

  // ServoJ flow control (real-time thread only, except the atomics).
  void reset_flow_control();
  std::atomic<bool> flow_baseline_pending_{true};
  bool flow_have_baseline_ = false;
  int last_cmd_num_ = 0;
  int64_t executed_points_ = 0;
  int64_t sent_points_ = 0;
  int consecutive_flow_skips_ = 0;
  std::atomic<int> queue_estimate_{0};
  std::atomic<uint64_t> flow_skips_{0};
  std::atomic<uint64_t> flow_rebaselines_{0};
  // Drift probe (see update_flow_probe()).
  void update_flow_probe(int executed_delta, SteadyClock::time_point now);
  int flow_low_progress_frames_ = 0;
  bool flow_probe_active_ = false;
  int flow_probe_frames_ = 0;
  bool flow_probe_progress_ = false;
  SteadyClock::time_point flow_last_probe_{};
  std::atomic<uint64_t> flow_probes_{0};
  std::atomic<uint64_t> flow_drift_corrections_{0};
  std::atomic<uint64_t> point_limited_{0};

  std::atomic<int> last_servoj_rc_{0};
  std::atomic<uint64_t> servoj_calls_{0};
  std::atomic<uint64_t> servoj_errors_{0};
  std::atomic<uint64_t> jump_guard_trips_{0};
  std::atomic<int> last_read_rc_{0};
  std::atomic<uint64_t> read_errors_{0};
  std::atomic<uint32_t> consecutive_read_errors_{0};

  // Snapshot of the realtime package for the status thread.
  std::mutex snapshot_mutex_;
  ROBOT_STATE_PKG snapshot_{};
  bool snapshot_valid_ = false;
  SteadyClock::time_point snapshot_frame_stamp_{};
  uint64_t frame_updates_ = 0;

  // read()-local frame tracking.
  bool have_frame_ = false;
  int64_t last_frame_key_ = 0;
  SteadyClock::time_point last_frame_time_{};
  SteadyClock::time_point last_good_read_time_{};

  std::mutex histogram_mutex_;
  PeriodHistogram histogram_;

  // ---- status node ---------------------------------------------------------
  rclcpp::Node::SharedPtr status_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> status_executor_;
  std::thread status_thread_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr stop_group_;
  rclcpp::Publisher<RobotStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<AuditEvent>::SharedPtr events_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Service<RecoverRobot>::SharedPtr recover_srv_;
  rclcpp::Service<SetGlobalSpeed>::SharedPtr speed_srv_;
  rclcpp::Service<SetSafetySpeed>::SharedPtr safety_speed_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_srv_;
  uint64_t last_status_frame_updates_ = 0;
  SteadyClock::time_point last_status_time_{};
};

} //end namespace


#endif
