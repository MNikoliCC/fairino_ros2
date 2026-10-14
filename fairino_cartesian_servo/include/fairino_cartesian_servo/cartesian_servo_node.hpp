#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>
#include <rclcpp/rclcpp.hpp>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fairino_msgs/msg/audit_event.hpp>
#include <fairino_msgs/msg/robot_status.hpp>
#include <fairino_msgs/msg/servo_status.hpp>
#include <fairino_msgs/srv/recover_robot.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/filters/se3_low_pass_filter.hpp>
#include <roboplan_oink/barriers/self_collision_barrier.hpp>
#include <roboplan_oink/constraints/acceleration_limit.hpp>
#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/frame.hpp>

#include "fairino_cartesian_servo/fault_catalog.hpp"
#include "fairino_cartesian_servo/servo_math.hpp"

namespace fairino_cartesian_servo {

enum class ServoState { kDisabled = 0, kSyncing, kActive, kHold, kRecovering, kFault, kLockout };
const char* toString(ServoState state);

/// Cartesian servo for the FAIRINO FR20.
///
/// Threads:
///  - real-time loop (SCHED_FIFO when permitted): watchdogs, SE(3) filter,
///    Cartesian rate limiter, OInK, joint interpolation and the 1 kHz command;
///  - operation worker: enable/disable/mode switch/recovery sequences that
///    wait for controller-manager and hardware services;
///  - ROS executor: subscriptions, services, the 100 Hz supervisor (fault
///    classification, auto-recovery trigger, status/diagnostics/events).
class CartesianServoNode : public rclcpp::Node {
public:
  explicit CartesianServoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~CartesianServoNode() override;

  /// Starts the real-time and worker threads.
  void start();
  /// Stops all threads (idempotent).
  void stop();

private:
  using Clock = std::chrono::steady_clock;
  using RobotStatus = fairino_msgs::msg::RobotStatus;
  using ServoStatus = fairino_msgs::msg::ServoStatus;
  using AuditEvent = fairino_msgs::msg::AuditEvent;
  using RecoverRobot = fairino_msgs::srv::RecoverRobot;
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  enum class OperationKind {
    kEnable,         // DISABLED -> SYNCING -> ACTIVE, HOLD -> ACTIVE
    kServoModeHold,  // DISABLED -> SYNCING -> HOLD
    kDisable,        // -> plan mode (trajectory controller)
    kRecover,        // operator Recover (policy sequence incl. RobotEnable)
    kResetFault,     // operator acknowledgement (no RobotEnable)
    kAutoRecover,    // whitelisted automatic recovery
  };
  struct OperationResult {
    bool success = false;
    std::string message;
  };
  struct Operation {
    OperationKind kind;
    std::string requested_by;
    std::shared_ptr<std::promise<OperationResult>> promise;
  };
  enum class SyncKind : int { kNone = 0, kFull = 1, kTargetOnly = 2 };

  // ---- setup ------------------------------------------------------------
  void declareAndLoadParameters();
  void buildModel();
  void createInterfaces();

  // ---- callbacks ----------------------------------------------------------
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onRobotStatus(const RobotStatus::SharedPtr msg);
  void supervisorTick();

  // ---- state machine ----------------------------------------------------
  ServoState state() const { return state_.load(); }
  void transition(ServoState to, const std::string& reason, const Fault* fault = nullptr);
  /// Transition only if the state is still `expected` (atomic with respect to
  /// other threads); returns false otherwise.
  bool transitionIf(ServoState expected, ServoState to, const std::string& reason, const Fault* fault = nullptr);
  void transitionLocked(ServoState to, const std::string& reason, const Fault* fault);
  void audit(const std::string& category, const std::string& action, const std::string& old_value,
             const std::string& new_value, bool success, const std::string& message, int code = 0);
  void drainEvents();

  // ---- operations -------------------------------------------------------
  OperationResult submit(OperationKind kind, const std::string& requested_by, double timeout_s);
  void workerLoop();
  OperationResult runEnable(bool end_active);
  OperationResult runDisable();
  OperationResult runRecovery(OperationKind kind);
  bool requestSync(SyncKind kind, double timeout_s, std::string& error);
  bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate,
                         std::string& error);
  bool waitForStreamingArmed(double timeout_s, std::string& reason, Clock::time_point newer_than = {});
  bool waitForStandstill(double timeout_s);
  void requestHardwareStop(const std::string& reason);
  bool hardwareReady(std::string& reason);

  // ---- real-time loop ---------------------------------------------------
  void realtimeLoop();
  void handleSyncRequest(Clock::time_point now);
  void startBraking();
  void brakeStep();
  bool solveStep(Clock::time_point now);
  void publishCommand();
  bool readJointSample(std::array<double, 6>& q, double& age_s) const;
  /// Empty when every measured joint is inside its limits (+ margin).
  std::string jointLimitViolation() const;

  // ---- helpers ------------------------------------------------------------
  RobotSignals robotSignals(double& status_age_s) const;
  Fault currentRobotFault() const;
  void pollControllers();
  void publishStatus();
  void publishDiagnostics();
  void updateTimingSummaries();
  std::string activePositionController() const;

  // ---- parameters ---------------------------------------------------------
  std::vector<std::string> joint_names_;
  std::string group_name_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string command_topic_;
  std::string servo_controller_;
  std::string trajectory_controller_;
  double command_frequency_ = 1000.0;
  double solver_frequency_ = 1000.0;
  int solver_divider_ = 1;
  double filter_tau_ = 0.05;
  double max_linear_speed_ = 1.0;
  double max_angular_speed_ = M_PI;
  std::vector<double> max_joint_velocity_;      // rad/s, applied to OInK
  std::vector<double> max_joint_acceleration_;  // rad/s^2
  bool use_acceleration_limit_ = true;
  double position_limit_gain_ = 0.5;
  double position_cost_ = 1.0;
  double orientation_cost_ = 1.0;
  double task_gain_ = 1.0;
  double lm_damping_ = 0.0;
  double regularization_ = 1e-6;
  int qp_max_iterations_ = 200;
  int self_collision_pairs_ = 0;
  double self_collision_d_min_ = 0.02;
  double self_collision_d_max_ = 0.25;
  double self_collision_gain_ = 20.0;
  double self_collision_safe_displacement_gain_ = 0.0;
  bool self_collision_convex_ = true;
  /// Original BVH meshes kept alive after the convex conversion (the Scene's
  /// own collision data may still reference them).
  std::vector<std::shared_ptr<void>> retained_collision_geometry_;
  bool robot_status_required_ = true;
  double joint_state_timeout_ = 0.1;
  double robot_status_timeout_ = 0.5;
  double robot_feedback_timeout_ = 0.1;
  double target_timeout_ = 1.0;
  double reacquire_translation_ = 0.01;
  double reacquire_rotation_ = 0.1;
  double max_tracking_error_ = 0.15;
  double joint_limit_margin_ = 0.0087;
  bool auto_recovery_ = true;
  int max_recovery_attempts_ = 2;
  double recovery_cooldown_ = 3.0;
  double recovery_attempt_window_ = 120.0;
  double standstill_timeout_ = 1.0;
  double standstill_velocity_deg_s_ = 0.5;
  int realtime_priority_ = 80;
  int cpu_affinity_ = -1;
  double status_rate_ = 10.0;

  // ---- model (real-time thread only after start) ---------------------------
  std::shared_ptr<roboplan::Scene> scene_;
  std::unique_ptr<roboplan::Oink> oink_;
  std::shared_ptr<roboplan::FrameTask> frame_task_;
  std::shared_ptr<roboplan::AccelerationLimit> acceleration_limit_;
  std::vector<std::shared_ptr<roboplan::Task>> tasks_;
  std::vector<std::shared_ptr<roboplan::Constraints>> constraints_;
  std::vector<std::shared_ptr<roboplan::Barrier>> barriers_;
  std::vector<std::shared_ptr<roboplan::Constraints>> no_constraints_;
  std::vector<std::shared_ptr<roboplan::Barrier>> no_barriers_;
  std::unique_ptr<roboplan::SE3LowPassFilter> filter_;
  Eigen::VectorXi joint_q_index_;  // q index of j1..j6 in the full configuration
  Eigen::VectorXi joint_v_group_index_;  // position of j1..j6 inside the OInK group vector
  Eigen::VectorXd q_min_;  // limits of j1..j6
  Eigen::VectorXd q_max_;
  Eigen::VectorXd q_cmd_full_;
  Eigen::VectorXd segment_start_;
  Eigen::VectorXd segment_end_;
  Eigen::VectorXd q_next_;
  Eigen::VectorXd delta_q_;
  Eigen::VectorXd delta_q_target_;
  Eigen::VectorXd delta_full_;
  Eigen::VectorXd previous_delta_;
  pinocchio::SE3 limited_target_;
  pinocchio::SE3 raw_target_;
  pinocchio::SE3 reset_pose_;
  bool target_acquired_ = false;
  uint64_t last_target_seq_ = 0;
  int segment_tick_ = 0;
  uint64_t tick_ = 0;
  bool have_command_ = false;
  std_msgs::msg::Float64MultiArray command_msg_;
  // Controlled stop when the servo leaves ACTIVE: the last commanded joint
  // velocity is ramped to zero at the joint acceleration limits.
  ServoState last_loop_state_ = ServoState::kDisabled;
  std::atomic<bool> braking_{false};
  Eigen::VectorXd brake_velocity_;
  double brake_duration_ = 0.0;
  double brake_elapsed_ = 0.0;

  // ---- shared state ---------------------------------------------------------
  std::atomic<ServoState> state_{ServoState::kDisabled};
  mutable std::mutex state_mutex_;
  std::string state_reason_ = "startup";
  Fault fault_;
  struct PendingEvent {
    AuditEvent event;
    bool log_error = false;
  };
  std::deque<PendingEvent> pending_events_;

  mutable std::mutex joint_mutex_;
  std::array<double, 6> joint_position_{};
  Clock::time_point joint_stamp_{};
  bool joint_valid_ = false;
  uint64_t joint_messages_ = 0;
  uint64_t joint_rejected_ = 0;
  std::string joint_reject_reason_;

  mutable std::mutex target_mutex_;
  pinocchio::SE3 target_pose_ = pinocchio::SE3::Identity();
  Clock::time_point target_stamp_{};
  uint64_t target_seq_ = 0;
  bool target_received_ = false;

  mutable std::mutex robot_mutex_;
  RobotStatus robot_status_;
  Clock::time_point robot_status_stamp_{};
  bool have_robot_status_ = false;
  uint64_t robot_servoj_errors_seen_ = 0;

  std::mutex sync_call_mutex_;  // one synchronization request at a time
  std::atomic<int> sync_request_{0};
  std::atomic<int> sync_result_{0};  // 0 pending, 1 ok, -1 failed
  std::mutex sync_error_mutex_;
  std::string sync_error_;
  std::atomic<bool> publish_commands_{false};
  std::atomic<bool> reset_pose_pending_{false};
  std::mutex reset_pose_mutex_;
  pinocchio::SE3 reset_pose_out_;
  std::atomic<double> tracking_error_{0.0};
  std::atomic<bool> realtime_scheduling_{false};
  std::atomic<bool> running_{false};

  // Operations.
  std::mutex operation_mutex_;
  std::condition_variable operation_cv_;
  std::deque<Operation> operations_;
  std::atomic<bool> worker_busy_{false};
  std::string current_operation_;

  // Recovery bookkeeping (history: worker only; the rest under recovery_mutex_).
  std::map<std::string, std::vector<Clock::time_point>> recovery_history_;
  mutable std::mutex recovery_mutex_;
  uint32_t recovery_attempts_ = 0;
  Clock::time_point last_recovery_end_{};
  std::string last_recovery_result_ = "none";
  double last_recovery_duration_ = 0.0;
  void setRecoveryResult(const std::string& result, Clock::time_point started);

  // Controller manager view.
  mutable std::mutex controller_mutex_;
  std::string active_position_controller_ = "unknown";
  bool list_pending_ = false;
  uint64_t controller_generation_ = 0;  // incremented by every switch
  Clock::time_point last_list_request_{};

  // Timing.
  std::unique_ptr<TimingRecorder> timing_period_;
  std::unique_ptr<TimingRecorder> timing_cycle_;
  std::unique_ptr<TimingRecorder> timing_filter_;
  std::unique_ptr<TimingRecorder> timing_rate_limiter_;
  std::unique_ptr<TimingRecorder> timing_oink_;
  std::unique_ptr<TimingRecorder> timing_solver_;
  std::unique_ptr<TimingRecorder> timing_publish_;
  std::mutex timing_mutex_;
  std::vector<TimingSummary> timing_summaries_;
  Clock::time_point last_timing_update_{};
  Clock::time_point last_status_publish_{};
  Clock::time_point last_diagnostics_publish_{};
  double joint_state_rate_hz_ = 0.0;
  uint64_t joint_messages_at_last_rate_ = 0;

  // ---- ROS interfaces --------------------------------------------------------
  rclcpp::CallbackGroup::SharedPtr sensor_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr supervisor_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<ServoStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<AuditEvent>::SharedPtr events_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reset_pose_pub_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<ListControllers>::SharedPtr list_client_;
  rclcpp::Client<RecoverRobot>::SharedPtr recover_client_;
  rclcpp::Client<Trigger>::SharedPtr hardware_stop_client_;
  rclcpp::TimerBase::SharedPtr supervisor_timer_;

  std::thread realtime_thread_;
  std::thread worker_thread_;
};

}  // namespace fairino_cartesian_servo
