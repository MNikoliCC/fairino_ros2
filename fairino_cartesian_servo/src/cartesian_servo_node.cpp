#include "fairino_cartesian_servo/cartesian_servo_node.hpp"

#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <coal/BVH/BVH_model.h>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace fairino_cartesian_servo {

namespace {

// Maximum age of the joint sample copied into the command at a full sync.
constexpr double kFullSyncMaxSampleAge = 0.02;

constexpr double kDegToRad = M_PI / 180.0;

double secondsSince(std::chrono::steady_clock::time_point now, std::chrono::steady_clock::time_point then) {
  return std::chrono::duration<double>(now - then).count();
}

double millisecondsBetween(std::chrono::steady_clock::time_point later,
                           std::chrono::steady_clock::time_point earlier) {
  return std::chrono::duration<double, std::milli>(later - earlier).count();
}

std::string readFile(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot read " + path);
  }
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::string join(const std::vector<std::string>& values) {
  std::string out;
  for (const auto& value : values) {
    out += (out.empty() ? "" : ",") + value;
  }
  return out;
}

std::string formatDouble(double value, int precision = 3) {
  std::ostringstream text;
  text.precision(precision);
  text << std::fixed << value;
  return text.str();
}

void addTimespec(timespec& time, int64_t nanoseconds) {
  time.tv_nsec += nanoseconds;
  while (time.tv_nsec >= 1000000000L) {
    time.tv_nsec -= 1000000000L;
    time.tv_sec += 1;
  }
}

}  // namespace

const char* toString(ServoState state) {
  switch (state) {
    case ServoState::kDisabled: return "DISABLED";
    case ServoState::kSyncing: return "SYNCING";
    case ServoState::kActive: return "ACTIVE";
    case ServoState::kHold: return "HOLD";
    case ServoState::kRecovering: return "RECOVERING";
    case ServoState::kFault: return "FAULT";
    case ServoState::kLockout: return "LOCKOUT";
  }
  return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

CartesianServoNode::CartesianServoNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("fairino_cartesian_servo", options) {
  declareAndLoadParameters();
  buildModel();
  createInterfaces();
  RCLCPP_INFO(get_logger(),
              "Cartesian servo ready: OInK %.0f Hz, command %.0f Hz (divider %d), filter tau %.3f s, "
              "Cartesian limits %.3f m/s %.3f rad/s, acceleration limit %s, self-collision pairs %d, "
              "robot status %s",
              solver_frequency_, command_frequency_, solver_divider_, filter_tau_, max_linear_speed_,
              max_angular_speed_, use_acceleration_limit_ ? "on" : "off", self_collision_pairs_,
              robot_status_required_ ? "required" : "not required (mock)");
}

CartesianServoNode::~CartesianServoNode() { stop(); }

void CartesianServoNode::declareAndLoadParameters() {
  joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{"j1", "j2", "j3", "j4", "j5", "j6"});
  if (joint_names_.size() != 6) {
    throw std::invalid_argument("joint_names must contain 6 joints");
  }
  group_name_ = declare_parameter<std::string>("group", "fairino20_v6_group");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  tip_frame_ = declare_parameter<std::string>("tip_frame", "wrist3_link");
  command_topic_ = declare_parameter<std::string>("command_topic", "/fairino_servo_controller/commands");
  servo_controller_ = declare_parameter<std::string>("servo_controller", "fairino_servo_controller");
  trajectory_controller_ = declare_parameter<std::string>("trajectory_controller", "fairino20_controller");

  command_frequency_ = declare_parameter<double>("command_frequency", 1000.0);
  solver_frequency_ = declare_parameter<double>("solver_frequency", 1000.0);
  if (!(command_frequency_ > 0.0) || !(solver_frequency_ > 0.0) || solver_frequency_ > command_frequency_) {
    throw std::invalid_argument("0 < solver_frequency <= command_frequency required");
  }
  const double divider = command_frequency_ / solver_frequency_;
  solver_divider_ = static_cast<int>(std::lround(divider));
  if (std::abs(divider - solver_divider_) > 1e-6) {
    throw std::invalid_argument("command_frequency must be an integer multiple of solver_frequency");
  }

  filter_tau_ = declare_parameter<double>("filter_tau", 0.05);
  max_linear_speed_ = declare_parameter<double>("max_linear_speed", 1.0);
  max_angular_speed_ = declare_parameter<double>("max_angular_speed", M_PI);

  // FAIRINO FR20 joint speed maxima (J1-J3 120 deg/s, J4-J6 180 deg/s).
  const auto velocity_deg = declare_parameter<std::vector<double>>(
      "max_joint_velocity_deg_s", std::vector<double>{120.0, 120.0, 120.0, 180.0, 180.0, 180.0});
  const double velocity_scale = declare_parameter<double>("velocity_scale", 1.0);
  // FAIRINO does not publish FR20 joint accelerations. 540 deg/s^2 is the peak
  // the controller itself planned for MoveJ at vel=acc=100 % on SimMachine
  // 3.9.9 (541.7 deg/s^2 on every joint); verify on the real robot.
  const auto acceleration_deg = declare_parameter<std::vector<double>>(
      "max_joint_acceleration_deg_s2", std::vector<double>{540.0, 540.0, 540.0, 540.0, 540.0, 540.0});
  if (velocity_deg.size() != 6 || acceleration_deg.size() != 6) {
    throw std::invalid_argument("joint velocity/acceleration limits need 6 values");
  }
  if (!(velocity_scale > 0.0 && velocity_scale <= 1.0)) {
    throw std::invalid_argument("velocity_scale must be within (0, 1]");
  }
  max_joint_velocity_.resize(6);
  max_joint_acceleration_.resize(6);
  for (int j = 0; j < 6; ++j) {
    max_joint_velocity_[j] = velocity_deg[j] * kDegToRad * velocity_scale;
    max_joint_acceleration_[j] = acceleration_deg[j] * kDegToRad;
  }
  use_acceleration_limit_ = declare_parameter<bool>("use_acceleration_limit", true);
  position_limit_gain_ = declare_parameter<double>("position_limit_gain", 0.5);
  position_cost_ = declare_parameter<double>("position_cost", 1.0);
  orientation_cost_ = declare_parameter<double>("orientation_cost", 1.0);
  task_gain_ = declare_parameter<double>("task_gain", 1.0);
  lm_damping_ = declare_parameter<double>("lm_damping", 0.0);
  regularization_ = declare_parameter<double>("regularization", 1e-6);
  qp_max_iterations_ = declare_parameter<int>("qp_max_iterations", 200);

  self_collision_pairs_ = declare_parameter<int>("self_collision_pairs", 0);
  self_collision_d_min_ = declare_parameter<double>("self_collision_d_min", 0.02);
  self_collision_d_max_ = declare_parameter<double>("self_collision_d_max", 0.25);
  self_collision_gain_ = declare_parameter<double>("self_collision_gain", 20.0);
  // RoboPlan weights this term by 1/|J_h|^2. The forearm/wrist2 housings are
  // rotationally symmetric (J_h ~ 0), so any non-zero gain freezes all joints.
  self_collision_safe_displacement_gain_ =
      declare_parameter<double>("self_collision_safe_displacement_gain", 0.0);
  self_collision_convex_ = declare_parameter<bool>("self_collision_convex", true);

  robot_status_required_ = declare_parameter<bool>("robot_status_required", true);
  joint_state_timeout_ = declare_parameter<double>("joint_state_timeout", 0.1);
  robot_status_timeout_ = declare_parameter<double>("robot_status_timeout", 0.5);
  robot_feedback_timeout_ = declare_parameter<double>("robot_feedback_timeout", 0.1);
  target_timeout_ = declare_parameter<double>("target_timeout", 1.0);
  reacquire_translation_ = declare_parameter<double>("target_reacquire_translation", 0.01);
  reacquire_rotation_ = declare_parameter<double>("target_reacquire_rotation", 0.1);
  max_tracking_error_ = declare_parameter<double>("max_tracking_error", 0.15);
  joint_limit_margin_ = declare_parameter<double>("joint_limit_margin", 0.0087);

  auto_recovery_ = declare_parameter<bool>("auto_recovery", true);
  max_recovery_attempts_ = declare_parameter<int>("max_recovery_attempts", 2);
  recovery_cooldown_ = declare_parameter<double>("recovery_cooldown", 3.0);
  recovery_attempt_window_ = declare_parameter<double>("recovery_attempt_window", 120.0);
  standstill_timeout_ = declare_parameter<double>("standstill_timeout", 1.0);
  standstill_velocity_deg_s_ = declare_parameter<double>("standstill_velocity_deg_s", 0.5);

  realtime_priority_ = declare_parameter<int>("realtime_priority", 80);
  cpu_affinity_ = declare_parameter<int>("cpu_affinity", -1);
  status_rate_ = declare_parameter<double>("status_rate", 10.0);
}

void CartesianServoNode::buildModel() {
  const std::string description_share = ament_index_cpp::get_package_share_directory("fairino_description");
  const std::string bringup_share = ament_index_cpp::get_package_share_directory("fairinoplan_bringup");
  const std::string urdf_path =
      declare_parameter<std::string>("urdf_path", description_share + "/urdf/fairino20_v6.urdf");
  const std::string srdf_path =
      declare_parameter<std::string>("srdf_path", bringup_share + "/config/fairino20_v6_robot.srdf");
  const std::string config_path =
      declare_parameter<std::string>("roboplan_config_path", bringup_share + "/config/roboplan.yaml");

  // The self-collision barrier uses a URDF with identical kinematics whose
  // collision meshes are convex hulls (conservative, see
  // tools/generate_servo_collision_model.py); the vendor meshes take 7-19 ms
  // per distance evaluation.
  const std::string self_collision_urdf_path = declare_parameter<std::string>(
      "self_collision_urdf_path", bringup_share + "/config/fairino20_v6_servo_collision.urdf");
  const std::string model_urdf = self_collision_pairs_ > 0 ? self_collision_urdf_path : urdf_path;

  const std::vector<std::filesystem::path> package_paths{
      std::filesystem::path(description_share), std::filesystem::path(description_share).parent_path(),
      std::filesystem::path(bringup_share).parent_path()};
  scene_ = std::make_shared<roboplan::Scene>("fairino_servo_scene", readFile(model_urdf), readFile(srdf_path),
                                             package_paths, std::filesystem::path(config_path));

  // GJK on coal::Convex objects instead of BVH-vs-BVH: must happen before the
  // OInK solver snapshots the collision geometry into its SceneContext.
  if (self_collision_pairs_ > 0 && self_collision_convex_) {
    auto& geometry_model = const_cast<pinocchio::GeometryModel&>(scene_->getCollisionModel());
    int converted = 0;
    for (auto& object : geometry_model.geometryObjects) {
      auto bvh = std::dynamic_pointer_cast<coal::BVHModelBase>(object.geometry);
      if (!bvh) {
        continue;
      }
      bvh->buildConvexRepresentation(false);
      if (bvh->convex) {
        retained_collision_geometry_.push_back(bvh);
        object.geometry = bvh->convex;
        ++converted;
      }
    }
    RCLCPP_INFO(get_logger(), "Self-collision model %s: %d collision meshes converted to convex objects",
                model_urdf.c_str(), converted);
  }

  roboplan::OinkSettings settings;
  settings.max_iter = qp_max_iterations_;
  settings.warm_start = true;
  oink_ = std::make_unique<roboplan::Oink>(*scene_, group_name_, settings);
  if (oink_->num_variables != 6) {
    throw std::runtime_error("OInK group '" + group_name_ + "' must have 6 velocity variables");
  }

  const auto& model = scene_->getModel();
  joint_q_index_.resize(6);
  joint_v_group_index_.resize(6);
  q_min_.resize(6);
  q_max_.resize(6);
  Eigen::VectorXd velocity_group(6);
  Eigen::VectorXd acceleration_group(6);
  for (int j = 0; j < 6; ++j) {
    if (!model.existJointName(joint_names_[j])) {
      throw std::runtime_error("joint " + joint_names_[j] + " not in the URDF");
    }
    const auto joint_id = model.getJointId(joint_names_[j]);
    const int idx_q = model.joints[joint_id].idx_q();
    const int idx_v = model.joints[joint_id].idx_v();
    joint_q_index_(j) = idx_q;
    q_min_(j) = model.lowerPositionLimit(idx_q);
    q_max_(j) = model.upperPositionLimit(idx_q);
    int group_index = -1;
    for (int i = 0; i < oink_->v_indices.size(); ++i) {
      if (oink_->v_indices(i) == idx_v) {
        group_index = i;
      }
    }
    if (group_index < 0) {
      throw std::runtime_error("joint " + joint_names_[j] + " is not part of group " + group_name_);
    }
    joint_v_group_index_(j) = group_index;
    // Never exceed the URDF limit either.
    const double urdf_velocity = model.velocityLimit(idx_v);
    if (urdf_velocity > 0.0) {
      max_joint_velocity_[j] = std::min(max_joint_velocity_[j], urdf_velocity);
    }
    velocity_group(group_index) = max_joint_velocity_[j];
    acceleration_group(group_index) = max_joint_acceleration_[j];
  }

  const double dt = 1.0 / solver_frequency_;
  roboplan::CartesianConfiguration goal;
  goal.base_frame = base_frame_;
  goal.tip_frame = tip_frame_;
  roboplan::FrameTaskOptions task_options;
  task_options.position_cost = position_cost_;
  task_options.orientation_cost = orientation_cost_;
  task_options.task_gain = task_gain_;
  task_options.lm_damping = lm_damping_;
  frame_task_ = std::make_shared<roboplan::FrameTask>(*oink_, *scene_, goal, task_options);
  tasks_ = {frame_task_};

  constraints_.push_back(std::make_shared<roboplan::PositionLimit>(*oink_, position_limit_gain_));
  constraints_.push_back(std::make_shared<roboplan::VelocityLimit>(*oink_, dt, velocity_group));
  if (use_acceleration_limit_) {
    acceleration_limit_ = std::make_shared<roboplan::AccelerationLimit>(*oink_, dt, acceleration_group);
    constraints_.push_back(acceleration_limit_);
  }
  if (self_collision_pairs_ > 0) {
    roboplan::SelfCollisionBarrierOptions options;
    options.n_collision_pairs = self_collision_pairs_;
    options.gain = self_collision_gain_;
    options.safe_displacement_gain = self_collision_safe_displacement_gain_;
    options.d_min = self_collision_d_min_;
    options.d_max = self_collision_d_max_;
    barriers_.push_back(std::make_shared<roboplan::SelfCollisionBarrier>(*oink_, *scene_, dt, options));
  }
  filter_ = std::make_unique<roboplan::SE3LowPassFilter>(filter_tau_);

  q_cmd_full_ = scene_->getCurrentJointPositions();
  segment_start_ = q_cmd_full_;
  segment_end_ = q_cmd_full_;
  q_next_ = q_cmd_full_;
  delta_q_ = Eigen::VectorXd::Zero(6);
  delta_q_target_ = Eigen::VectorXd::Zero(6);
  previous_delta_ = Eigen::VectorXd::Zero(6);
  delta_full_ = Eigen::VectorXd::Zero(model.nv);
  limited_target_ = pinocchio::SE3::Identity();
  raw_target_ = pinocchio::SE3::Identity();
  reset_pose_ = pinocchio::SE3::Identity();
  command_msg_.data.assign(6, 0.0);

  const double command_budget_ms = 1000.0 / command_frequency_;
  const double solver_budget_ms = 1000.0 / solver_frequency_;
  timing_period_ = std::make_unique<TimingRecorder>("period", 1.5 * command_budget_ms);
  timing_cycle_ = std::make_unique<TimingRecorder>("cycle", command_budget_ms);
  timing_filter_ = std::make_unique<TimingRecorder>("filter", solver_budget_ms);
  timing_rate_limiter_ = std::make_unique<TimingRecorder>("rate_limiter", solver_budget_ms);
  timing_oink_ = std::make_unique<TimingRecorder>("oink", solver_budget_ms);
  timing_solver_ = std::make_unique<TimingRecorder>("solver_total", solver_budget_ms);
  timing_publish_ = std::make_unique<TimingRecorder>("publish", command_budget_ms);

  std::ostringstream limits;
  for (int j = 0; j < 6; ++j) {
    limits << joint_names_[j] << "=" << formatDouble(max_joint_velocity_[j] / kDegToRad, 1) << "deg/s/"
           << formatDouble(max_joint_acceleration_[j] / kDegToRad, 0) << "deg/s2 ";
  }
  RCLCPP_INFO(get_logger(), "OInK joint limits: %s", limits.str().c_str());
}

void CartesianServoNode::createInterfaces() {
  sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  supervisor_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sensor_options;
  sensor_options.callback_group = sensor_group_;
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(5), [this](sensor_msgs::msg::JointState::SharedPtr msg) { onJointState(msg); },
      sensor_options);
  target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/fairino_servo/target_pose", rclcpp::QoS(5),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { onTarget(msg); }, sensor_options);
  robot_status_sub_ = create_subscription<RobotStatus>(
      "/fairino_robot/status", rclcpp::QoS(10), [this](RobotStatus::SharedPtr msg) { onRobotStatus(msg); },
      sensor_options);

  command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, rclcpp::QoS(1));
  status_pub_ = create_publisher<ServoStatus>("/fairino_servo/status", rclcpp::QoS(10));
  diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/fairino_servo/diagnostics", rclcpp::QoS(10));
  events_pub_ = create_publisher<AuditEvent>("/fairino_servo/events",
                                             rclcpp::QoS(rclcpp::KeepLast(200)).reliable().transient_local());
  reset_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/fairino_servo/reset_target_pose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  auto add_trigger = [this](const std::string& name, std::function<OperationResult()> handler) {
    services_.push_back(create_service<Trigger>(
        name,
        [handler](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
          const auto result = handler();
          response->success = result.success;
          response->message = result.message;
        },
        rmw_qos_profile_services_default, service_group_));
  };
  add_trigger("/fairino_servo/enable", [this]() { return submit(OperationKind::kEnable, "operator", 15.0); });
  add_trigger("/fairino_servo/disable", [this]() { return submit(OperationKind::kDisable, "operator", 15.0); });
  add_trigger("/fairino_servo/recover", [this]() { return submit(OperationKind::kRecover, "operator", 60.0); });
  add_trigger("/fairino_servo/reset_fault",
              [this]() { return submit(OperationKind::kResetFault, "operator", 60.0); });
  add_trigger("/fairino_servo/hold", [this]() {
    OperationResult result;
    const auto current = state();
    if (current == ServoState::kHold) {
      result.success = true;
      result.message = "already holding";
    } else if (current == ServoState::kActive &&
               transitionIf(ServoState::kActive, ServoState::kHold, "operator hold")) {
      audit("operator", "hold", "ACTIVE", "HOLD", true, "operator hold");
      result.success = true;
      result.message = "holding last safe joint command";
    } else {
      result.message = std::string("hold is only possible from ACTIVE (state ") + toString(current) + ")";
    }
    return result;
  });
  add_trigger("/fairino_servo/reset_target", [this]() {
    OperationResult result;
    std::string error;
    const ServoState current = state();
    if (current == ServoState::kSyncing || current == ServoState::kRecovering) {
      result.message = std::string("target reset refused while ") + toString(current);
      return result;
    }
    result.success = requestSync(SyncKind::kTargetOnly, 1.0, error);
    result.message = result.success ? "target reset to the current TCP" : "target reset failed: " + error;
    if (result.success) {
      audit("operator", "reset_target", "", "current TCP", true, "target reset to current TCP");
    }
    return result;
  });
  // Benchmark support: exact statistics over a whole test run (last 65 s).
  add_trigger("/fairino_servo/reset_timing", [this]() {
    for (auto* recorder : {timing_period_.get(), timing_cycle_.get(), timing_filter_.get(),
                           timing_rate_limiter_.get(), timing_oink_.get(), timing_solver_.get(),
                           timing_publish_.get()}) {
      recorder->reset();
    }
    return OperationResult{true, "timing statistics reset"};
  });
  add_trigger("/fairino_servo/timing_report", [this]() {
    std::ostringstream json;
    json.precision(6);
    json << "{\"solver_frequency_hz\":" << solver_frequency_ << ",\"command_frequency_hz\":" << command_frequency_
         << ",\"self_collision_pairs\":" << self_collision_pairs_
         << ",\"realtime_scheduling\":" << (realtime_scheduling_ ? "true" : "false") << ",\"stages\":[";
    bool first = true;
    for (auto* recorder : {timing_period_.get(), timing_cycle_.get(), timing_filter_.get(),
                           timing_rate_limiter_.get(), timing_oink_.get(), timing_solver_.get(),
                           timing_publish_.get()}) {
      const auto t = recorder->summarizeTotal();
      json << (first ? "" : ",") << "{\"name\":\"" << t.name << "\",\"samples\":" << t.samples
           << ",\"mean_ms\":" << t.mean_ms << ",\"p95_ms\":" << t.p95_ms << ",\"p99_ms\":" << t.p99_ms
           << ",\"max_ms\":" << t.max_ms << ",\"budget_ms\":" << t.budget_ms
           << ",\"overruns\":" << t.overruns_total << "}";
      first = false;
    }
    json << "]}";
    return OperationResult{true, json.str()};
  });
  services_.push_back(create_service<SetBool>(
      "/fairino_servo/set_mode",
      [this](const std::shared_ptr<SetBool::Request> request, std::shared_ptr<SetBool::Response> response) {
        const auto result = submit(request->data ? OperationKind::kServoModeHold : OperationKind::kDisable,
                                   "operator", 15.0);
        response->success = result.success;
        response->message = result.message;
      },
      rmw_qos_profile_services_default, service_group_));

  switch_client_ = create_client<SwitchController>("/controller_manager/switch_controller",
                                                   rmw_qos_profile_services_default, client_group_);
  list_client_ = create_client<ListControllers>("/controller_manager/list_controllers",
                                                rmw_qos_profile_services_default, client_group_);
  recover_client_ =
      create_client<RecoverRobot>("/fairino_robot/recover", rmw_qos_profile_services_default, client_group_);
  hardware_stop_client_ =
      create_client<Trigger>("/fairino_robot/stop", rmw_qos_profile_services_default, client_group_);

  supervisor_timer_ =
      create_wall_timer(std::chrono::milliseconds(10), [this]() { supervisorTick(); }, supervisor_group_);
}

void CartesianServoNode::start() {
  if (running_.exchange(true)) {
    return;
  }
  realtime_thread_ = std::thread([this]() { realtimeLoop(); });
  worker_thread_ = std::thread([this]() { workerLoop(); });
  audit("state", "start", "", "DISABLED", true, "servo node started in plan mode");
}

void CartesianServoNode::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  operation_cv_.notify_all();
  if (realtime_thread_.joinable()) {
    realtime_thread_.join();
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

// ---------------------------------------------------------------------------
// Subscriptions
// ---------------------------------------------------------------------------

void CartesianServoNode::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::array<double, 6> q{};
  for (int j = 0; j < 6; ++j) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[j]);
    if (it == msg->name.end() || static_cast<std::size_t>(it - msg->name.begin()) >= msg->position.size()) {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      ++joint_rejected_;
      joint_reject_reason_ = "missing joint " + joint_names_[j];
      return;
    }
    q[j] = msg->position[static_cast<std::size_t>(it - msg->name.begin())];
    if (!std::isfinite(q[j])) {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      ++joint_rejected_;
      joint_reject_reason_ = "non-finite position for " + joint_names_[j];
      return;
    }
  }
  std::lock_guard<std::mutex> lock(joint_mutex_);
  joint_position_ = q;
  joint_stamp_ = Clock::now();
  joint_valid_ = true;
  ++joint_messages_;
}

void CartesianServoNode::onTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Rejecting target in frame '%s'; expected '%s'",
                          msg->header.frame_id.c_str(), base_frame_.c_str());
    return;
  }
  const auto& p = msg->pose.position;
  const auto& o = msg->pose.orientation;
  Eigen::Quaterniond quaternion(o.w, o.x, o.y, o.z);
  const double norm = quaternion.norm();
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) || norm < 0.5) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Rejecting invalid target pose");
    return;
  }
  quaternion.normalize();
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_pose_ = pinocchio::SE3(quaternion.toRotationMatrix(), Eigen::Vector3d(p.x, p.y, p.z));
  target_stamp_ = Clock::now();
  ++target_seq_;
  target_received_ = true;
}

void CartesianServoNode::onRobotStatus(const RobotStatus::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(robot_mutex_);
  robot_status_ = *msg;
  robot_status_stamp_ = Clock::now();
  have_robot_status_ = true;
}

bool CartesianServoNode::readJointSample(std::array<double, 6>& q, double& age_s) const {
  std::lock_guard<std::mutex> lock(joint_mutex_);
  if (!joint_valid_) {
    return false;
  }
  q = joint_position_;
  age_s = secondsSince(Clock::now(), joint_stamp_);
  return true;
}

std::string CartesianServoNode::jointLimitViolation() const {
  std::array<double, 6> q{};
  double age = 0.0;
  if (!readJointSample(q, age)) {
    return "";
  }
  for (int j = 0; j < 6; ++j) {
    if (q[j] < q_min_(j) - joint_limit_margin_ || q[j] > q_max_(j) + joint_limit_margin_) {
      return joint_names_[j] + "=" + formatDouble(q[j], 4) + " rad";
    }
  }
  return "";
}

// ---------------------------------------------------------------------------
// State machine and audit
// ---------------------------------------------------------------------------

void CartesianServoNode::transition(ServoState to, const std::string& reason, const Fault* fault) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  transitionLocked(to, reason, fault);
}

bool CartesianServoNode::transitionIf(ServoState expected, ServoState to, const std::string& reason,
                                      const Fault* fault) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_.load() != expected) {
    return false;
  }
  transitionLocked(to, reason, fault);
  return true;
}

void CartesianServoNode::transitionLocked(ServoState to, const std::string& reason, const Fault* fault) {
  const ServoState from = state_.load();
  if (fault != nullptr) {
    fault_ = *fault;
  } else if (to != ServoState::kFault && to != ServoState::kLockout && to != ServoState::kRecovering) {
    fault_ = Fault{};
  }
  if (from == to && reason == state_reason_) {
    return;
  }
  state_reason_ = reason;
  state_.store(to);
  PendingEvent pending;
  pending.event.source = "fairino_cartesian_servo";
  pending.event.category = "state";
  pending.event.action = std::string(toString(from)) + "->" + toString(to);
  pending.event.old_value = toString(from);
  pending.event.new_value = toString(to);
  pending.event.success = to != ServoState::kFault && to != ServoState::kLockout;
  pending.event.message = reason;
  if (fault != nullptr && fault->active()) {
    pending.event.result_code = fault->sdk_code != 0 ? fault->sdk_code : fault->main_code;
    pending.event.message += " [" + fault->source + " main=" + std::to_string(fault->main_code) +
                             " sub=" + std::to_string(fault->sub_code) + " sdk=" + std::to_string(fault->sdk_code) +
                             " policy=" + toString(fault->policy) + "]";
  }
  pending.log_error = !pending.event.success;
  pending_events_.push_back(std::move(pending));
}

void CartesianServoNode::audit(const std::string& category, const std::string& action, const std::string& old_value,
                               const std::string& new_value, bool success, const std::string& message, int code) {
  PendingEvent pending;
  pending.event.source = "fairino_cartesian_servo";
  pending.event.category = category;
  pending.event.action = action;
  pending.event.old_value = old_value;
  pending.event.new_value = new_value;
  pending.event.success = success;
  pending.event.message = message;
  pending.event.result_code = code;
  pending.log_error = !success;
  std::lock_guard<std::mutex> lock(state_mutex_);
  pending_events_.push_back(std::move(pending));
}

void CartesianServoNode::drainEvents() {
  std::deque<PendingEvent> events;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    events.swap(pending_events_);
  }
  for (auto& pending : events) {
    pending.event.stamp = now();
    const auto& e = pending.event;
    if (pending.log_error) {
      RCLCPP_ERROR(get_logger(), "AUDIT [%s] %s: '%s' -> '%s' code=%d %s", e.category.c_str(), e.action.c_str(),
                   e.old_value.c_str(), e.new_value.c_str(), e.result_code, e.message.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "AUDIT [%s] %s: '%s' -> '%s' code=%d %s", e.category.c_str(), e.action.c_str(),
                  e.old_value.c_str(), e.new_value.c_str(), e.result_code, e.message.c_str());
    }
    events_pub_->publish(e);
  }
}

// ---------------------------------------------------------------------------
// Operations (worker thread)
// ---------------------------------------------------------------------------

CartesianServoNode::OperationResult CartesianServoNode::submit(OperationKind kind, const std::string& requested_by,
                                                               double timeout_s) {
  static const std::map<OperationKind, const char*> names = {
      {OperationKind::kEnable, "enable"},           {OperationKind::kServoModeHold, "servo_mode"},
      {OperationKind::kDisable, "disable"},         {OperationKind::kRecover, "recover"},
      {OperationKind::kResetFault, "reset_fault"},  {OperationKind::kAutoRecover, "auto_recover"},
  };
  auto promise = std::make_shared<std::promise<OperationResult>>();
  auto future = promise->get_future();
  {
    std::lock_guard<std::mutex> lock(operation_mutex_);
    if (worker_busy_ || !operations_.empty()) {
      return {false, "busy: " + current_operation_ + " in progress"};
    }
    operations_.push_back({kind, requested_by, promise});
    current_operation_ = names.at(kind);
  }
  if (kind != OperationKind::kAutoRecover) {
    audit("operator", names.at(kind), toString(state()), "", true, "requested by " + requested_by);
  }
  operation_cv_.notify_one();
  if (timeout_s <= 0.0) {
    return {true, "accepted"};
  }
  if (future.wait_for(std::chrono::duration<double>(timeout_s)) != std::future_status::ready) {
    return {false, std::string(names.at(kind)) + " still running (result not known yet); see /fairino_servo/status"};
  }
  return future.get();
}

void CartesianServoNode::workerLoop() {
  while (running_) {
    Operation operation;
    {
      std::unique_lock<std::mutex> lock(operation_mutex_);
      operation_cv_.wait_for(lock, std::chrono::milliseconds(200),
                             [this]() { return !operations_.empty() || !running_; });
      if (!running_) {
        return;
      }
      if (operations_.empty()) {
        continue;
      }
      operation = std::move(operations_.front());
      operations_.pop_front();
      worker_busy_ = true;
    }
    std::string name;
    {
      std::lock_guard<std::mutex> lock(operation_mutex_);
      name = current_operation_;
    }
    OperationResult result;
    try {
      switch (operation.kind) {
        case OperationKind::kEnable: result = runEnable(true); break;
        case OperationKind::kServoModeHold: result = runEnable(false); break;
        case OperationKind::kDisable: result = runDisable(); break;
        case OperationKind::kRecover:
        case OperationKind::kResetFault:
        case OperationKind::kAutoRecover: result = runRecovery(operation.kind); break;
      }
    } catch (const std::exception& error) {
      result = {false, std::string("operation failed: ") + error.what()};
      const Fault fault = servoFault(result.message);
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_.load() != ServoState::kLockout) {  // never downgrade a LOCKOUT
        transitionLocked(ServoState::kFault, result.message, &fault);
      }
    }
    audit("operator", name + " result", "", toString(state()), result.success, result.message);
    {
      std::lock_guard<std::mutex> lock(operation_mutex_);
      worker_busy_ = false;
      current_operation_.clear();
    }
    operation.promise->set_value(result);
  }
}

bool CartesianServoNode::hardwareReady(std::string& reason) {
  if (!robot_status_required_) {
    return true;
  }
  const Fault fault = currentRobotFault();
  if (fault.active()) {
    reason = "robot not ready: " + fault.description;
    return false;
  }
  std::lock_guard<std::mutex> lock(robot_mutex_);
  if (robot_status_.streaming_state != RobotStatus::STREAMING_ARMED) {
    reason = "hardware ServoJ streaming not armed: " + robot_status_.streaming_reason;
    return false;
  }
  return true;
}

CartesianServoNode::OperationResult CartesianServoNode::runEnable(bool end_active) {
  const ServoState current = state();
  std::string error;
  if (current == ServoState::kFault || current == ServoState::kLockout || current == ServoState::kRecovering) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return {false, std::string("cannot enable in ") + toString(current) + ": " + state_reason_};
  }
  if (current == ServoState::kSyncing) {
    return {false, "already synchronizing"};
  }
  if (current == ServoState::kActive) {
    return {true, end_active ? "servo already active" : "servo mode already active (ACTIVE)"};
  }
  if (!hardwareReady(error)) {
    return {false, error};
  }
  if (current == ServoState::kHold) {
    if (!end_active) {
      return {true, "servo mode already active (HOLD)"};
    }
    // HOLD never resumes towards a stale target: the reference restarts at the
    // TCP of the held command and the previous OInK velocity is zeroed. A
    // braking ramp still running after ACTIVE -> HOLD finishes first.
    const auto brake_deadline = Clock::now() + std::chrono::seconds(2);
    while (braking_.load() && Clock::now() < brake_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (braking_.load()) {
      return {false, "resume refused: braking ramp still running"};
    }
    if (!requestSync(SyncKind::kTargetOnly, 1.0, error)) {
      return {false, "resume failed: " + error};
    }
    if (!transitionIf(ServoState::kHold, ServoState::kActive, "operator resumed servo (target reset to current TCP)")) {
      return {false, std::string("state changed during resume: ") + toString(state())};
    }
    return {true, "servo resumed"};
  }

  // DISABLED: the robot must be at rest (no trajectory still playing), then
  // q_command = q_measured, target = FK(q_measured), filter reset, previous
  // velocity zero, and only then the controller switch.
  if (!waitForStandstill(0.5)) {
    return {false, "robot is moving (trajectory running?): wait until it stops before enabling the servo"};
  }
  if (!transitionIf(ServoState::kDisabled, ServoState::kSyncing,
                    "synchronizing command and target to the measured state")) {
    return {false, std::string("state changed: ") + toString(state())};
  }
  if (!requestSync(SyncKind::kFull, 1.0, error)) {
    const Fault fault = servoFault("synchronization failed: " + error);
    transitionIf(ServoState::kSyncing, ServoState::kFault, fault.description, &fault);
    return {false, fault.description};
  }
  publish_commands_ = true;
  if (!switchControllers({servo_controller_}, {trajectory_controller_}, error)) {
    publish_commands_ = false;
    const Fault fault = servoFault("controller switch to servo failed: " + error);
    transitionIf(ServoState::kSyncing, ServoState::kFault, fault.description, &fault);
    return {false, fault.description};
  }
  if (!transitionIf(ServoState::kSyncing, end_active ? ServoState::kActive : ServoState::kHold,
                    end_active ? "servo controller active, tracking" : "servo controller active, holding")) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return {false, std::string("state changed during switch: ") + toString(state_.load()) + ": " + state_reason_};
  }
  return {true, end_active ? "servo enabled" : "servo mode active (HOLD)"};
}

CartesianServoNode::OperationResult CartesianServoNode::runDisable() {
  const ServoState current = state();
  if (current == ServoState::kRecovering || current == ServoState::kSyncing) {
    return {false, std::string("cannot disable while ") + toString(current)};
  }
  const bool servo_owned = publish_commands_.load() || activePositionController() == servo_controller_;
  if (current == ServoState::kActive) {
    transitionIf(ServoState::kActive, ServoState::kHold, "disable requested: holding before the controller switch");
  }
  if (servo_owned) {
    if (!waitForStandstill(standstill_timeout_)) {
      audit("controller", "disable", "", "", false,
            "no standstill within " + formatDouble(standstill_timeout_, 1) + " s; switching anyway");
    }
    std::string error;
    if (!switchControllers({trajectory_controller_}, {servo_controller_}, error)) {
      return {false, "controller switch to trajectory controller failed: " + error};
    }
    publish_commands_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const ServoState after = state_.load();
    if (after == ServoState::kFault || after == ServoState::kLockout) {
      return {true, std::string("plan mode active; ") + toString(after) + " remains: " + state_reason_};
    }
    transitionLocked(ServoState::kDisabled, "plan mode: trajectory controller owns the joints", nullptr);
  }
  return {true, "plan mode active"};
}

CartesianServoNode::OperationResult CartesianServoNode::runRecovery(OperationKind kind) {
  const auto started = Clock::now();
  const ServoState current = state();
  Fault fault;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    fault = fault_;
  }
  const char* kind_name = kind == OperationKind::kResetFault    ? "reset_fault"
                          : kind == OperationKind::kAutoRecover ? "auto_recover"
                                                                : "recover";
  const Fault present = currentRobotFault();

  // ---- permission ------------------------------------------------------------
  if (kind == OperationKind::kResetFault) {
    if (current != ServoState::kFault && current != ServoState::kLockout) {
      return {false, std::string("Reset Fault is only possible in FAULT or LOCKOUT (state ") + toString(current) + ")"};
    }
    if (current == ServoState::kLockout && present.policy == Policy::kLockout) {
      return {false, "lockout condition still active: " + present.description + ". " + present.action};
    }
  } else {
    if (current != ServoState::kFault) {
      return {false, std::string("Recover is only possible in FAULT (state ") + toString(current) + ")"};
    }
    if (!fault.known) {
      return {false, "unknown fault: no recovery procedure is guessed; inspect, then use Reset Fault"};
    }
    if (fault.policy != Policy::kAutoRecover && fault.policy != Policy::kOperatorReset) {
      return {false, std::string("fault policy ") + toString(fault.policy) + " does not allow recovery"};
    }
    if (kind == OperationKind::kAutoRecover && (fault.policy != Policy::kAutoRecover || !auto_recovery_)) {
      return {false, "fault is not whitelisted for automatic recovery"};
    }
    // "Joint position outside the safe range" is never recovered automatically.
    const std::string violation = jointLimitViolation();
    if (kind == OperationKind::kAutoRecover && !violation.empty()) {
      Fault limit_fault = servoFault("joint position outside the safe range: " + violation);
      limit_fault.action = "Jog/drag the joint back inside its limits in the WebApp, then Reset Fault.";
      transitionIf(ServoState::kFault, ServoState::kFault, limit_fault.description, &limit_fault);
      return {false, "automatic recovery refused: " + limit_fault.description};
    }
  }

  // ---- attempt bookkeeping ------------------------------------------------------
  const std::string signature = fault.signature();
  if (kind == OperationKind::kResetFault) {
    recovery_history_.clear();
    std::lock_guard<std::mutex> lock(recovery_mutex_);
    recovery_attempts_ = 0;
  } else {
    auto& history = recovery_history_[signature];
    history.erase(std::remove_if(history.begin(), history.end(),
                                 [&](const Clock::time_point& t) {
                                   return secondsSince(started, t) > recovery_attempt_window_;
                                 }),
                  history.end());
    if (static_cast<int>(history.size()) >= max_recovery_attempts_) {
      Fault lockout = fault;
      lockout.policy = Policy::kLockout;
      lockout.severity = Severity::kCritical;
      lockout.description = "same fault repeated after " + std::to_string(history.size()) +
                            " recovery attempts: " + fault.description;
      lockout.action = "Find the root cause; Reset Fault clears the attempt counter.";
      transition(ServoState::kLockout, lockout.description, &lockout);
      requestHardwareStop(lockout.description);
      return {false, lockout.description};
    }
    history.push_back(started);
    std::lock_guard<std::mutex> lock(recovery_mutex_);
    recovery_attempts_ = static_cast<uint32_t>(history.size());
  }

  const bool servo_mode = publish_commands_.load() || activePositionController() == servo_controller_;
  if (!transitionIf(current, ServoState::kRecovering, std::string(kind_name) + " of: " + fault.description,
                    &fault)) {
    return {false, std::string("state changed before recovery: ") + toString(state())};
  }

  auto finish_failure = [&](const std::string& message) -> OperationResult {
    setRecoveryResult(std::string("FAILED: ") + message, started);
    const Fault now_fault = currentRobotFault();
    Fault report = now_fault.active() ? now_fault : fault;
    if (kind != OperationKind::kResetFault &&
        static_cast<int>(recovery_history_[signature].size()) >= max_recovery_attempts_) {
      report.policy = Policy::kLockout;
      report.description = "recovery failed " + std::to_string(recovery_history_[signature].size()) +
                           " times: " + message;
      transitionIf(ServoState::kRecovering, ServoState::kLockout, report.description, &report);
      requestHardwareStop(report.description);
    } else if (report.policy == Policy::kLockout) {
      transitionIf(ServoState::kRecovering, ServoState::kLockout, "recovery failed: " + message, &report);
      requestHardwareStop("recovery failed in a lockout condition: " + message);
    } else {
      transitionIf(ServoState::kRecovering, ServoState::kFault, "recovery failed: " + message, &report);
    }
    audit("recovery", kind_name, fault.description, "failed", false, message);
    return {false, "recovery failed: " + message};
  };

  // ---- hardware sequence (ServoMoveEnd, StopMotion, standstill, ResetAllError,
  //      error-code check, RobotEnable, ServoMoveStart) ----------------------------
  std::string error;
  Clock::time_point hardware_recovered{};
  if (robot_status_required_) {
    if (!recover_client_->wait_for_service(std::chrono::seconds(2))) {
      return finish_failure("/fairino_robot/recover is not available");
    }
    auto request = std::make_shared<RecoverRobot::Request>();
    request->reset_errors = true;
    request->enable_robot = kind != OperationKind::kResetFault;
    request->restart_servo = true;
    request->requested_by = std::string("fairino_cartesian_servo/") + kind_name;
    request->reason = fault.description;
    auto future = recover_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(25)) != std::future_status::ready) {
      return finish_failure("hardware recovery timed out");
    }
    const auto response = future.get();
    hardware_recovered = Clock::now();
    std::string steps;
    for (const auto& step : response->steps) {
      steps += (steps.empty() ? "" : "; ") + step;
    }
    audit("recovery", "hardware recover", "", response->success ? "ok" : "failed", response->success,
          steps + (response->message.empty() ? "" : " | " + response->message), response->failed_step_code);
    if (!response->success) {
      return finish_failure("hardware step '" + response->failed_step + "' code " +
                            std::to_string(response->failed_step_code) + ": " + response->message);
    }
  }

  // ---- command/target resynchronization -------------------------------------------
  if (servo_mode) {
    if (!requestSync(SyncKind::kFull, 1.0, error)) {
      return finish_failure("resynchronization failed: " + error);
    }
    publish_commands_ = true;
  } else {
    // Plan mode: restarting the trajectory controller makes it hold the
    // measured position, which arms the hardware without a jump.
    if (!switchControllers({}, {trajectory_controller_}, error) ||
        !switchControllers({trajectory_controller_}, {}, error)) {
      return finish_failure("trajectory controller restart failed: " + error);
    }
  }
  if (robot_status_required_) {
    if (!waitForStreamingArmed(3.0, error, hardware_recovered)) {
      return finish_failure("hardware did not re-arm ServoJ streaming: " + error);
    }
    const Fault remaining = currentRobotFault();
    if (remaining.active()) {
      return finish_failure("fault still present: " + remaining.description);
    }
  }

  const double duration = secondsSince(Clock::now(), started);
  if (!transitionIf(ServoState::kRecovering, servo_mode ? ServoState::kHold : ServoState::kDisabled,
                    std::string(kind_name) + " complete; operator must resume")) {
    const std::string aborted = std::string("ABORTED: state changed to ") + toString(state()) + " during recovery";
    setRecoveryResult(aborted, started);
    audit("recovery", kind_name, fault.description, toString(state()), false, aborted);
    return {false, aborted};
  }
  const std::string ok = std::string("OK (") + kind_name + ", " + formatDouble(duration, 2) + " s)";
  setRecoveryResult(ok, started);
  audit("recovery", kind_name, fault.description, servo_mode ? "HOLD" : "DISABLED", true,
        "duration " + formatDouble(duration, 3) + " s");
  return {true, ok};
}

void CartesianServoNode::setRecoveryResult(const std::string& result, Clock::time_point started) {
  std::lock_guard<std::mutex> lock(recovery_mutex_);
  last_recovery_end_ = Clock::now();
  last_recovery_duration_ = secondsSince(last_recovery_end_, started);
  last_recovery_result_ = result;
}

bool CartesianServoNode::requestSync(SyncKind kind, double timeout_s, std::string& error) {
  // The worker and the reset_target service may both synchronize; one shared
  // request slot must never be overwritten by the other caller.
  std::lock_guard<std::mutex> sync_lock(sync_call_mutex_);
  if (!running_) {
    error = "real-time loop not running";
    return false;
  }
  sync_result_ = 0;
  sync_request_ = static_cast<int>(kind);
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
  while (Clock::now() < deadline) {
    const int result = sync_result_.load();
    if (result != 0) {
      if (result < 0) {
        std::lock_guard<std::mutex> lock(sync_error_mutex_);
        error = sync_error_;
      }
      return result > 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  sync_request_ = 0;
  error = "synchronization timed out";
  return false;
}

bool CartesianServoNode::switchControllers(const std::vector<std::string>& activate,
                                           const std::vector<std::string>& deactivate, std::string& error) {
  if (!switch_client_->wait_for_service(std::chrono::seconds(2))) {
    error = "controller manager switch service unavailable";
    return false;
  }
  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration::from_seconds(2.0);
  auto future = switch_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    error = "controller switch timed out";
    audit("controller", "switch", join(deactivate), join(activate), false, error);
    return false;
  }
  const bool ok = future.get()->ok;
  audit("controller", "switch", "deactivate[" + join(deactivate) + "]", "activate[" + join(activate) + "]", ok,
        ok ? "switch accepted (strict)" : "controller manager rejected the switch");
  if (!ok) {
    error = "controller manager rejected the switch";
    return false;
  }
  std::lock_guard<std::mutex> lock(controller_mutex_);
  ++controller_generation_;  // older list_controllers answers are discarded
  if (!activate.empty()) {
    active_position_controller_ = activate.front();
  } else {
    active_position_controller_ = "none";
  }
  return true;
}

bool CartesianServoNode::waitForStreamingArmed(double timeout_s, std::string& reason, Clock::time_point newer_than) {
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
  while (Clock::now() < deadline) {
    {
      // Only a status received after `newer_than` counts: the one cached from
      // before a hardware recovery may still say ARMED.
      std::lock_guard<std::mutex> lock(robot_mutex_);
      if (robot_status_.streaming_state == RobotStatus::STREAMING_ARMED && robot_status_stamp_ > newer_than &&
          secondsSince(Clock::now(), robot_status_stamp_) < robot_status_timeout_) {
        return true;
      }
      reason = robot_status_.streaming_reason + " (state " + std::to_string(robot_status_.streaming_state) + ")";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

bool CartesianServoNode::waitForStandstill(double timeout_s) {
  // Two joint samples 50 ms apart must agree within 0.1 mrad (works with mock
  // hardware too); with FAIRINO hardware the measured joint speeds must also
  // be below standstill_velocity_deg_s.
  const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
  while (Clock::now() < deadline) {
    std::array<double, 6> q0{};
    std::array<double, 6> q1{};
    double age = 0.0;
    bool still = readJointSample(q0, age) && age < joint_state_timeout_;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    still = still && readJointSample(q1, age) && age < joint_state_timeout_;
    for (int j = 0; still && j < 6; ++j) {
      still = std::abs(q1[j] - q0[j]) < 1e-4;
    }
    if (robot_status_required_) {
      std::lock_guard<std::mutex> lock(robot_mutex_);
      for (double velocity : robot_status_.joint_velocity_deg_s) {
        still = still && std::abs(velocity) < standstill_velocity_deg_s_;
      }
    }
    if (still) {
      return true;
    }
  }
  return false;
}

void CartesianServoNode::requestHardwareStop(const std::string& reason) {
  if (!robot_status_required_) {
    return;
  }
  if (!hardware_stop_client_->service_is_ready()) {
    audit("recovery", "hardware stop", "", "", false, "/fairino_robot/stop unavailable: " + reason);
    return;
  }
  hardware_stop_client_->async_send_request(std::make_shared<Trigger::Request>());
  audit("recovery", "hardware stop", "", "DISARMED", true, "requested: " + reason);
}

// ---------------------------------------------------------------------------
// Robot status classification
// ---------------------------------------------------------------------------

RobotSignals CartesianServoNode::robotSignals(double& status_age_s) const {
  RobotSignals s;
  std::lock_guard<std::mutex> lock(robot_mutex_);
  if (!have_robot_status_) {
    status_age_s = -1.0;
    return s;
  }
  const auto& r = robot_status_;
  status_age_s = secondsSince(Clock::now(), robot_status_stamp_);
  s.status_available = status_age_s <= robot_status_timeout_;
  s.connected = r.connected && r.sdk_com_state == 0;
  s.feedback_age_s = r.joint_feedback_age_s;
  s.robot_enabled = r.robot_enabled;
  s.main_code = r.main_error_code;
  s.sub_code = r.sub_error_code;
  s.emergency_stop = r.emergency_stop;
  s.button_box_stop = r.button_box_stop;
  s.safety_stop0 = r.safety_stop0;
  s.safety_stop1 = r.safety_stop1;
  s.collision = r.collision;
  s.safety_door = r.safety_door_alarm;
  s.safety_plane = r.safety_plane_alarm;
  s.safety_data_abnormal = r.safety_data_abnormal;
  s.out_soft_limit = r.out_soft_limit_error;
  s.abnormal_stop = r.abnormal_stop;
  s.alarm_check_estop = r.alarm_check_estop_button;
  s.alarm_reboot = r.alarm_reboot_robot;
  s.driver_alarm_axis = r.driver_alarm_axis;
  s.driver_error_axis = r.driver_error_axis;
  for (auto value : r.driver_com_error) {
    s.driver_com_error = s.driver_com_error || value != 0;
  }
  s.ctrl_box_error = r.ctrl_box_error;
  if (r.streaming_state == RobotStatus::STREAMING_DISARMED) {
    if (r.last_servoj_return_code != 0) {
      s.servoj_failed = true;
      s.servoj_code = r.last_servoj_return_code;
    }
    const auto& reason = r.streaming_reason;
    if (reason.find("jump guard") != std::string::npos || reason.find("non-finite") != std::string::npos) {
      s.command_rejected = true;
      s.command_rejected_reason = reason;
    }
  }
  return s;
}

Fault CartesianServoNode::currentRobotFault() const {
  if (!robot_status_required_) {
    return Fault{};
  }
  double age = 0.0;
  const RobotSignals signals = robotSignals(age);
  if (age < 0.0) {
    return Fault{};  // no status received yet: startup grace, enabling is refused separately
  }
  Fault fault = classifyRobot(signals, robot_feedback_timeout_);
  if (!fault.active()) {
    // Any disarm counts, including an external /fairino_robot/stop: the servo
    // must not silently resume when the hardware re-arms. A recovery still in
    // progress is transient (it ends ARMED/WAIT_SYNC or "recovery failed");
    // in ACTIVE/HOLD the supervisor faults on any non-ARMED streaming anyway,
    // and during the servo's own recovery the state is RECOVERING.
    std::lock_guard<std::mutex> lock(robot_mutex_);
    if (robot_status_.streaming_state == RobotStatus::STREAMING_DISARMED &&
        robot_status_.streaming_reason != "recovery in progress") {
      fault = servoFault("hardware ServoJ streaming disarmed: " + robot_status_.streaming_reason,
                         Policy::kOperatorReset, Severity::kWarning);
      fault.action = "Reset Fault re-arms streaming after the cause is cleared.";
    }
  }
  return fault;
}

std::string CartesianServoNode::activePositionController() const {
  std::lock_guard<std::mutex> lock(controller_mutex_);
  return active_position_controller_;
}

void CartesianServoNode::pollControllers() {
  const auto now = Clock::now();
  {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    if (list_pending_ && secondsSince(now, last_list_request_) < 2.0) {
      return;
    }
    if (secondsSince(now, last_list_request_) < 0.5) {
      return;
    }
    last_list_request_ = now;
    list_pending_ = true;
  }
  uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    generation = controller_generation_;
  }
  if (!list_client_->service_is_ready()) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    list_pending_ = false;
    return;
  }
  list_client_->async_send_request(
      std::make_shared<ListControllers::Request>(),
      [this, generation](rclcpp::Client<ListControllers>::SharedFuture future) {
        std::string active = "none";
        for (const auto& controller : future.get()->controller) {
          if (controller.state != "active") {
            continue;
          }
          if (controller.name == servo_controller_) {
            active = servo_controller_;
          } else if (controller.name == trajectory_controller_ && active == "none") {
            active = trajectory_controller_;
          }
        }
        std::lock_guard<std::mutex> lock(controller_mutex_);
        if (!worker_busy_ && generation == controller_generation_) {
          active_position_controller_ = active;
        }
        list_pending_ = false;
      });
}

// ---------------------------------------------------------------------------
// Supervisor
// ---------------------------------------------------------------------------

void CartesianServoNode::supervisorTick() {
  drainEvents();
  const auto now = Clock::now();
  ServoState current = state();

  // Robot faults and safety signals.
  if (robot_status_required_) {
    const Fault fault = currentRobotFault();
    if (fault.active()) {
      if (fault.policy == Policy::kLockout) {
        if (current != ServoState::kLockout) {
          transition(ServoState::kLockout, "LOCKOUT: " + fault.description, &fault);
          requestHardwareStop(fault.description);
        }
      } else if (current == ServoState::kDisabled || current == ServoState::kSyncing ||
                 current == ServoState::kActive || current == ServoState::kHold) {
        transitionIf(current, ServoState::kFault, fault.description, &fault);
      } else if (current == ServoState::kFault) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (fault_.signature() != fault.signature() && &worst(fault_, fault) == &fault) {
          fault_ = fault;
        }
      }
    }
  }

  // Measured joint position outside the safe range never recovers automatically.
  current = state();
  if (current == ServoState::kDisabled || current == ServoState::kActive || current == ServoState::kHold) {
    const std::string violation = jointLimitViolation();
    if (!violation.empty()) {
      Fault fault = servoFault("joint position outside the safe range: " + violation);
      fault.action = "Jog/drag the joint back inside its limits in the WebApp, then Reset Fault.";
      transitionIf(current, ServoState::kFault, fault.description, &fault);
    }
  }

  // Whitelisted automatic recovery with cooldown.
  current = state();
  if (current == ServoState::kFault && auto_recovery_ && !worker_busy_) {
    Fault fault;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      fault = fault_;
    }
    Clock::time_point last_end;
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      last_end = last_recovery_end_;
    }
    if (fault.policy == Policy::kAutoRecover && fault.known && secondsSince(now, last_end) >= recovery_cooldown_) {
      submit(OperationKind::kAutoRecover, "auto_recovery", 0.0);
    }
  }

  // Tracking needs armed streaming; a re-arm must never resume motion by itself.
  current = state();
  if (robot_status_required_ && (current == ServoState::kActive || current == ServoState::kHold) && !worker_busy_) {
    std::string reason;
    uint8_t streaming = RobotStatus::STREAMING_ARMED;
    {
      std::lock_guard<std::mutex> lock(robot_mutex_);
      streaming = robot_status_.streaming_state;
      reason = robot_status_.streaming_reason;
    }
    if (streaming != RobotStatus::STREAMING_ARMED) {
      const Fault fault = servoFault("hardware ServoJ streaming not armed while " + std::string(toString(current)) +
                                     ": " + reason);
      transitionIf(current, ServoState::kFault, fault.description, &fault);
    }
  }

  // Controller ownership.
  pollControllers();
  current = state();
  if ((current == ServoState::kActive || current == ServoState::kHold) && !worker_busy_) {
    const std::string active = activePositionController();
    if (active != "unknown" && active != servo_controller_) {
      const Fault fault = servoFault("servo controller is not active (active: " + active + ")");
      transitionIf(current, ServoState::kFault, fault.description, &fault);
    }
  }

  // Publish the pose the reference was reset to (the UI moves its gizmo).
  if (reset_pose_pending_.exchange(false)) {
    pinocchio::SE3 pose;
    {
      std::lock_guard<std::mutex> lock(reset_pose_mutex_);
      pose = reset_pose_out_;
    }
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = base_frame_;
    msg.pose.position.x = pose.translation().x();
    msg.pose.position.y = pose.translation().y();
    msg.pose.position.z = pose.translation().z();
    const Eigen::Quaterniond quaternion(pose.rotation());
    msg.pose.orientation.x = quaternion.x();
    msg.pose.orientation.y = quaternion.y();
    msg.pose.orientation.z = quaternion.z();
    msg.pose.orientation.w = quaternion.w();
    reset_pose_pub_->publish(msg);
  }

  if (secondsSince(now, last_timing_update_) >= 1.0) {
    updateTimingSummaries();
  }
  if (secondsSince(now, last_status_publish_) >= 1.0 / status_rate_) {
    last_status_publish_ = now;
    publishStatus();
  }
  if (secondsSince(now, last_diagnostics_publish_) >= 1.0) {
    last_diagnostics_publish_ = now;
    publishDiagnostics();
  }
}

void CartesianServoNode::updateTimingSummaries() {
  const auto now = Clock::now();
  const double window = secondsSince(now, last_timing_update_);
  last_timing_update_ = now;
  std::vector<TimingSummary> summaries;
  for (auto* recorder : {timing_period_.get(), timing_cycle_.get(), timing_filter_.get(), timing_rate_limiter_.get(),
                         timing_oink_.get(), timing_solver_.get(), timing_publish_.get()}) {
    summaries.push_back(recorder->summarize());
  }
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    if (window > 0.0 && window < 10.0) {
      joint_state_rate_hz_ = static_cast<double>(joint_messages_ - joint_messages_at_last_rate_) / window;
    }
    joint_messages_at_last_rate_ = joint_messages_;
  }
  std::lock_guard<std::mutex> lock(timing_mutex_);
  timing_summaries_ = std::move(summaries);
}

void CartesianServoNode::publishStatus() {
  ServoStatus msg;
  msg.header.stamp = now();
  const ServoState current = state();
  msg.state = toString(current);
  Fault fault;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    msg.reason = state_reason_;
    fault = fault_;
  }
  const std::string active = activePositionController();
  msg.active_position_controller = active;
  msg.mode = active == servo_controller_ ? "servo" : active == trajectory_controller_ ? "plan" : "unknown";
  if (worker_busy_) {
    msg.mode = "transition";
  }
  msg.robot_status_required = robot_status_required_;
  msg.fault_source = fault.source;
  msg.main_error_code = fault.main_code;
  msg.sub_error_code = fault.sub_code;
  msg.sdk_return_code = fault.sdk_code;
  msg.fault_description = fault.description;
  msg.fault_severity = toString(fault.severity);
  msg.fault_policy = toString(fault.policy);
  msg.suggested_action = fault.action;
  if (!fault.active()) {
    // Show warnings that do not change the state (e.g. waiting for status).
    double age = 0.0;
    robotSignals(age);
    if (robot_status_required_ && age < 0.0) {
      msg.suggested_action = "waiting for /fairino_robot/status from the hardware plugin";
    }
  }
  msg.auto_recovery_enabled = auto_recovery_;
  {
    std::lock_guard<std::mutex> lock(recovery_mutex_);
    msg.recovery_attempts = recovery_attempts_;
    msg.last_recovery_result = last_recovery_result_;
    msg.last_recovery_duration_s = last_recovery_duration_;
  }
  msg.max_recovery_attempts = static_cast<uint32_t>(max_recovery_attempts_);
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    msg.joint_state_age_s = joint_valid_ ? secondsSince(Clock::now(), joint_stamp_) : -1.0;
    msg.joint_state_rate_hz = joint_state_rate_hz_;
  }
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    msg.target_received = target_received_;
    msg.target_age_s = target_received_ ? secondsSince(Clock::now(), target_stamp_) : -1.0;
  }
  msg.tracking_error_rad = tracking_error_.load();
  msg.solver_frequency_hz = solver_frequency_;
  msg.command_frequency_hz = command_frequency_;
  msg.self_collision_pairs = static_cast<uint32_t>(std::max(0, self_collision_pairs_));
  msg.realtime_scheduling = realtime_scheduling_.load();
  {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    for (const auto& summary : timing_summaries_) {
      fairino_msgs::msg::TimingStats stats;
      stats.name = summary.name;
      stats.samples = summary.samples;
      stats.mean_ms = summary.mean_ms;
      stats.p95_ms = summary.p95_ms;
      stats.p99_ms = summary.p99_ms;
      stats.max_ms = summary.max_ms;
      stats.budget_ms = summary.budget_ms;
      stats.overruns_window = summary.overruns_window;
      stats.overruns_total = summary.overruns_total;
      msg.timing.push_back(stats);
    }
  }
  msg.joint_velocity_limit_rad_s = max_joint_velocity_;
  msg.joint_acceleration_limit_rad_s2 = use_acceleration_limit_ ? max_joint_acceleration_ : std::vector<double>{};
  msg.max_linear_speed_m_s = max_linear_speed_;
  msg.max_angular_speed_rad_s = max_angular_speed_;
  msg.filter_tau_s = filter_tau_;
  status_pub_->publish(msg);
}

void CartesianServoNode::publishDiagnostics() {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "fairino_cartesian_servo";
  status.hardware_id = "FAIRINO FR20";
  const ServoState current = state();
  status.level = current == ServoState::kFault || current == ServoState::kLockout ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                 : current == ServoState::kSyncing || current == ServoState::kRecovering
                     ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                     : diagnostic_msgs::msg::DiagnosticStatus::OK;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    status.message = std::string(toString(current)) + ": " + state_reason_;
  }
  auto add = [&status](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };
  add("active_position_controller", activePositionController());
  add("tracking_error_rad", formatDouble(tracking_error_.load(), 5));
  add("realtime_scheduling", realtime_scheduling_ ? "SCHED_FIFO" : "SCHED_OTHER");
  {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    for (const auto& summary : timing_summaries_) {
      add(summary.name + "_p99_ms", formatDouble(summary.p99_ms, 4));
      add(summary.name + "_max_ms", formatDouble(summary.max_ms, 4));
      add(summary.name + "_overruns_total", std::to_string(summary.overruns_total));
    }
  }
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    add("joint_state_rate_hz", formatDouble(joint_state_rate_hz_, 1));
    add("joint_states_rejected", std::to_string(joint_rejected_));
    if (!joint_reject_reason_.empty()) {
      add("last_joint_state_rejection", joint_reject_reason_);
    }
  }
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

// ---------------------------------------------------------------------------
// Real-time loop
// ---------------------------------------------------------------------------

void CartesianServoNode::realtimeLoop() {
  if (realtime_priority_ > 0) {
    sched_param parameters{};
    parameters.sched_priority = realtime_priority_;
    realtime_scheduling_ = pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameters) == 0;
  }
  if (cpu_affinity_ >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cpu_affinity_, &set);
    pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
  }
  RCLCPP_INFO(get_logger(), "Servo loop running at %.0f Hz with %s scheduling", command_frequency_,
              realtime_scheduling_ ? "SCHED_FIFO" : "default (no real-time permission)");

  const int64_t period_ns = static_cast<int64_t>(std::llround(1e9 / command_frequency_));
  timespec next{};
  clock_gettime(CLOCK_MONOTONIC, &next);
  Clock::time_point last_wake = Clock::now();
  bool first = true;

  while (running_) {
    addTimespec(next, period_ns);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    const auto wake = Clock::now();
    if (!first) {
      timing_period_->record(millisecondsBetween(wake, last_wake));
    }
    first = false;
    last_wake = wake;

    handleSyncRequest(wake);
    const ServoState current = state();

    if ((current == ServoState::kActive || current == ServoState::kHold) && have_command_) {
      std::array<double, 6> q{};
      double age = 0.0;
      if (!readJointSample(q, age) || age > joint_state_timeout_) {
        const Fault fault = servoFault("joint state watchdog expired (age " + formatDouble(age, 3) + " s)");
        transitionIf(current, ServoState::kFault, fault.description, &fault);
      } else {
        double max_error = 0.0;
        for (int j = 0; j < 6; ++j) {
          max_error = std::max(max_error, std::abs(q_cmd_full_(joint_q_index_(j)) - q[j]));
        }
        tracking_error_ = max_error;
        if (max_error > max_tracking_error_) {
          const Fault fault = servoFault("joint command tracking error " + formatDouble(max_error, 4) +
                                         " rad exceeds " + formatDouble(max_tracking_error_, 3) + " rad");
          transitionIf(current, ServoState::kFault, fault.description, &fault);
        }
      }
    }

    if (state() == ServoState::kActive) {
      braking_ = false;
      if (tick_ % static_cast<uint64_t>(solver_divider_) == 0) {
        solveStep(wake);
      }
      if (state() == ServoState::kActive && segment_tick_ < solver_divider_) {
        ++segment_tick_;
        const double alpha = static_cast<double>(segment_tick_) / solver_divider_;
        q_cmd_full_ = segment_start_ + alpha * (segment_end_ - segment_start_);
      }
    } else {
      if (last_loop_state_ == ServoState::kActive && have_command_) {
        startBraking();
      }
      if (braking_) {
        brakeStep();
      }
      previous_delta_.setZero();
      segment_start_ = q_cmd_full_;
      segment_end_ = q_cmd_full_;
      segment_tick_ = solver_divider_;
    }
    last_loop_state_ = state();

    if (publish_commands_.load() && have_command_) {
      const auto publish_start = Clock::now();
      publishCommand();
      timing_publish_->record(millisecondsBetween(Clock::now(), publish_start));
    }
    if (current == ServoState::kActive) {
      timing_cycle_->record(millisecondsBetween(Clock::now(), wake));
    }
    ++tick_;

    // After a long stall do not try to catch up with a burst of cycles.
    timespec now_ts{};
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    const int64_t lag_ns = (now_ts.tv_sec - next.tv_sec) * 1000000000L + (now_ts.tv_nsec - next.tv_nsec);
    if (lag_ns > period_ns) {
      next = now_ts;
    }
  }
}

void CartesianServoNode::startBraking() {
  // Commanded velocity of the current segment, ramped linearly to zero so that
  // the joint with the longest stop decelerates at its acceleration limit and
  // the others follow the same direction (the path is kept).
  brake_velocity_ = (segment_end_ - segment_start_) * solver_frequency_;
  brake_duration_ = 0.0;
  for (int j = 0; j < 6; ++j) {
    const double velocity = std::abs(brake_velocity_(joint_q_index_(j)));
    brake_duration_ = std::max(brake_duration_, velocity / max_joint_acceleration_[j]);
  }
  brake_elapsed_ = 0.0;
  braking_ = brake_duration_ > 0.0;
}

void CartesianServoNode::brakeStep() {
  const double dt = 1.0 / command_frequency_;
  brake_elapsed_ += dt;
  // Mid-point velocity of this tick for an exact linear ramp.
  const double scale = std::max(0.0, 1.0 - (brake_elapsed_ - 0.5 * dt) / brake_duration_);
  for (int j = 0; j < 6; ++j) {
    const int index = joint_q_index_(j);
    q_cmd_full_(index) = std::clamp(q_cmd_full_(index) + brake_velocity_(index) * scale * dt, q_min_(j), q_max_(j));
  }
  if (brake_elapsed_ >= brake_duration_) {
    braking_ = false;
  }
}

void CartesianServoNode::handleSyncRequest(Clock::time_point now) {
  (void)now;
  const int request = sync_request_.exchange(0);
  if (request == 0) {
    return;
  }
  auto fail = [this](const std::string& message) {
    std::lock_guard<std::mutex> lock(sync_error_mutex_);
    sync_error_ = message;
    sync_result_ = -1;
  };
  std::array<double, 6> q{};
  double age = 0.0;
  const bool have_joint = readJointSample(q, age) && age <= joint_state_timeout_;
  const ServoState current = state();

  Eigen::VectorXd reference = q_cmd_full_;
  if (request == static_cast<int>(SyncKind::kFull)) {
    // The command is copied from this sample: it must be recent enough that
    // the robot cannot have moved noticeably since (JSB publishes at 250 Hz).
    if (!have_joint || age > kFullSyncMaxSampleAge) {
      fail("no fresh joint state for synchronization (age " + formatDouble(age, 3) + " s, limit " +
           formatDouble(kFullSyncMaxSampleAge, 3) + " s)");
      return;
    }
    for (int j = 0; j < 6; ++j) {
      q_cmd_full_(joint_q_index_(j)) = q[j];
    }
    have_command_ = true;
    reference = q_cmd_full_;
  } else {
    const bool use_command = have_command_ && (current == ServoState::kActive || current == ServoState::kHold);
    if (!use_command) {
      if (!have_joint) {
        fail("no fresh joint state (age " + formatDouble(age, 3) + " s)");
        return;
      }
      for (int j = 0; j < 6; ++j) {
        reference(joint_q_index_(j)) = q[j];
      }
    }
  }
  // A target reset while ACTIVE keeps the current joint motion: the OInK
  // acceleration limit then decelerates towards the new (current) target
  // instead of stopping the reference within one cycle. In HOLD a running
  // braking ramp is not interrupted either.
  const bool keep_motion = request == static_cast<int>(SyncKind::kTargetOnly) &&
                           (current == ServoState::kActive || braking_.load());
  if (!keep_motion) {
    braking_ = false;
    segment_start_ = q_cmd_full_;
    segment_end_ = q_cmd_full_;
    segment_tick_ = solver_divider_;
  }

  const pinocchio::SE3 pose(scene_->forwardKinematics(reference, tip_frame_, base_frame_));
  reset_pose_ = pose;
  raw_target_ = pose;
  limited_target_ = pose;
  filter_->reset(pose);
  if (!keep_motion) {
    previous_delta_.setZero();
    if (acceleration_limit_) {
      acceleration_limit_->reset();
    }
  }
  target_acquired_ = false;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    last_target_seq_ = target_seq_;
  }
  {
    std::lock_guard<std::mutex> lock(reset_pose_mutex_);
    reset_pose_out_ = pose;
  }
  reset_pose_pending_ = true;
  sync_result_ = 1;
}

bool CartesianServoNode::solveStep(Clock::time_point now) {
  const double dt = 1.0 / solver_frequency_;
  const auto solve_start = Clock::now();

  // Latest Viser target. After a synchronization a target is accepted only
  // once it is close to the reset pose, so a stale gizmo can never pull the
  // robot towards an old position.
  pinocchio::SE3 sample;
  Clock::time_point stamp;
  uint64_t seq = 0;
  bool received = false;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    sample = target_pose_;
    stamp = target_stamp_;
    seq = target_seq_;
    received = target_received_;
  }
  if (received && seq != last_target_seq_) {
    if (target_acquired_) {
      raw_target_ = sample;
    } else {
      double translation = 0.0;
      double rotation = 0.0;
      poseDistance(sample, reset_pose_, translation, rotation);
      if (translation <= reacquire_translation_ && rotation <= reacquire_rotation_) {
        target_acquired_ = true;
        raw_target_ = sample;
      }
    }
    last_target_seq_ = seq;
  }
  if (target_acquired_ && secondsSince(now, stamp) > target_timeout_) {
    transitionIf(ServoState::kActive, ServoState::kHold,
                 "Viser target stale for " + formatDouble(secondsSince(now, stamp), 2) + " s");
    return false;
  }

  // SE(3) low-pass filter and explicit Cartesian rate limit.
  const auto filter_start = Clock::now();
  const pinocchio::SE3 filtered = filter_->update(raw_target_, dt);
  const auto limiter_start = Clock::now();
  limited_target_ = rateLimitPose(limited_target_, filtered, max_linear_speed_, max_angular_speed_, dt);
  const auto oink_start = Clock::now();
  timing_filter_->record(millisecondsBetween(limiter_start, filter_start));
  timing_rate_limiter_->record(millisecondsBetween(oink_start, limiter_start));

  // RoboPlan OInK: FrameTask base_link -> wrist3_link with position, velocity
  // and acceleration limits (and the optional self-collision barrier).
  frame_task_->setTargetFrameTransform(limited_target_.toHomogeneousMatrix());
  if (acceleration_limit_) {
    acceleration_limit_->setLastVelocity(previous_delta_ / dt);
    delta_q_target_.setZero();
    const auto target_result =
        oink_->solveIk(segment_end_, tasks_, no_constraints_, no_barriers_, delta_q_target_, regularization_);
    if (target_result) {
      acceleration_limit_->setTargetDisplacement(delta_q_target_);
    } else {
      acceleration_limit_->clearTargetDisplacement();
    }
  }
  delta_q_.setZero();
  auto result = oink_->solveIk(segment_end_, tasks_, constraints_, barriers_, delta_q_, regularization_);
  if (result && acceleration_limit_) {
    // ProxQP meets the bounds only to eps_abs (1e-6 rad), i.e. ~1 rad/s^2 at
    // 1 kHz where a*dt^2 is ~1e-5 rad: clamp to the exact acceleration and
    // velocity boxes (always non-empty because previous_delta_ obeys them).
    for (int j = 0; j < 6; ++j) {
      const int i = joint_v_group_index_(j);
      const double a_step = max_joint_acceleration_[j] * dt * dt;
      const double v_step = max_joint_velocity_[j] * dt;
      delta_q_(i) = std::clamp(delta_q_(i), std::max(previous_delta_(i) - a_step, -v_step),
                               std::min(previous_delta_(i) + a_step, v_step));
    }
  }
  if (result && !barriers_.empty()) {
    // Post-solve check of the nonlinear barrier values (zeroes violating joints).
    result = oink_->enforceBarriers(segment_end_, barriers_, delta_q_);
  }
  const auto oink_end = Clock::now();
  timing_oink_->record(millisecondsBetween(oink_end, oink_start));
  if (!result) {
    const Fault fault = servoFault("OInK solve failed: " + result.error());
    transitionIf(ServoState::kActive, ServoState::kFault, fault.description, &fault);
    return false;
  }

  delta_full_.setZero();
  for (int i = 0; i < delta_q_.size(); ++i) {
    delta_full_(oink_->v_indices(i)) = delta_q_(i);
  }
  pinocchio::integrate(scene_->getModel(), segment_end_, delta_full_, q_next_);

  // Validate the joint result before it can reach the controller.
  for (int j = 0; j < 6; ++j) {
    const double value = q_next_(joint_q_index_(j));
    const double step = std::abs(delta_q_(joint_v_group_index_(j)));
    const double max_step = max_joint_velocity_[j] * dt * 1.01 + 1e-6;
    std::string problem;
    if (!std::isfinite(value)) {
      problem = "non-finite joint result";
    } else if (value < q_min_(j) - 1e-6 || value > q_max_(j) + 1e-6) {
      problem = "joint result outside limits";
    } else if (step > max_step) {
      problem = "joint step " + formatDouble(step, 6) + " rad exceeds " + formatDouble(max_step, 6) + " rad";
    }
    if (!problem.empty()) {
      const Fault fault = servoFault("OInK result rejected on " + joint_names_[j] + ": " + problem);
      transitionIf(ServoState::kActive, ServoState::kFault, fault.description, &fault);
      return false;
    }
  }
  segment_start_ = segment_end_;
  segment_end_ = q_next_;
  segment_tick_ = 0;
  previous_delta_ = delta_q_;
  timing_solver_->record(millisecondsBetween(Clock::now(), solve_start));
  return true;
}

void CartesianServoNode::publishCommand() {
  for (int j = 0; j < 6; ++j) {
    command_msg_.data[static_cast<std::size_t>(j)] = q_cmd_full_(joint_q_index_(j));
  }
  command_pub_->publish(command_msg_);
}

}  // namespace fairino_cartesian_servo
