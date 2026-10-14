#pragma once

#include <string>
#include <vector>

namespace fairino_cartesian_servo {

/// How bad a fault is for the operator.
enum class Severity { kNone = 0, kInfo, kWarning, kError, kCritical };

/// What the servo state machine is allowed to do about a fault.
///  - kAutoRecover:   whitelisted; the controlled recovery may run without an
///                    operator (when auto_recovery is enabled). Never ends in
///                    motion: HOLD in servo mode, DISABLED with the trajectory
///                    controller holding the measured position in plan mode.
///  - kOperatorReset: FAULT; recovery runs only after an explicit operator action.
///  - kLockout:       LOCKOUT; no software recovery. Manual intervention first.
enum class Policy { kNone = 0, kAutoRecover, kOperatorReset, kLockout };

const char* toString(Severity severity);
const char* toString(Policy policy);

/// Row of the controller main/sub fault catalog (FAIRINO manual 3.9.9,
/// Appendix 1 "Motion controller errors").
struct ControllerFaultEntry {
  int main_code;
  int sub_min;
  int sub_max;
  const char* description;
  bool resettable;  // as stated by FAIRINO
  Severity severity;
  Policy policy;
  const char* operator_action;
  const char* simmachine_test;
  const char* real_robot_test;
};

/// Row of the SDK return-code catalog (robot_error.h and the FAIRINO SDK
/// error code table).
struct SdkCodeEntry {
  int code;
  const char* description;
  Severity severity;
  Policy policy;
  const char* operator_action;
  const char* simmachine_test;
  const char* real_robot_test;
};

/// Safety signal rows (realtime state package booleans).
struct SignalEntry {
  const char* signal;
  const char* description;
  Severity severity;
  Policy policy;
  const char* operator_action;
  const char* simmachine_test;
  const char* real_robot_test;
};

/// Robot status in a ROS independent form so the classifier is unit testable.
struct RobotSignals {
  bool status_available = false;
  bool connected = false;
  double feedback_age_s = 0.0;
  bool robot_enabled = true;
  int main_code = 0;
  int sub_code = 0;
  bool emergency_stop = false;
  bool button_box_stop = false;
  bool safety_stop0 = false;
  bool safety_stop1 = false;
  bool collision = false;
  bool safety_door = false;
  bool safety_plane = false;
  bool safety_data_abnormal = false;
  bool out_soft_limit = false;
  bool abnormal_stop = false;
  bool alarm_check_estop = false;
  int alarm_reboot = 0;
  int driver_alarm_axis = 0;
  int driver_error_axis = 0;
  bool driver_com_error = false;
  int ctrl_box_error = 0;
  /// ServoJ returned a non-zero code and the hardware stopped streaming.
  bool servoj_failed = false;
  int servoj_code = 0;
  /// Hardware discontinuity guard or non-finite command stopped streaming.
  bool command_rejected = false;
  std::string command_rejected_reason;
};

/// One classified fault.
struct Fault {
  std::string source = "none";  // none | controller | sdk | signal | servo
  int main_code = 0;
  int sub_code = 0;
  int sdk_code = 0;
  std::string description;
  Severity severity = Severity::kNone;
  Policy policy = Policy::kNone;
  std::string action;
  bool known = true;

  bool active() const { return policy != Policy::kNone; }
  /// Identity used to detect a repeated fault after recovery.
  std::string signature() const;
};

const std::vector<ControllerFaultEntry>& controllerFaultTable();
const std::vector<SdkCodeEntry>& sdkCodeTable();
const std::vector<SignalEntry>& signalTable();

/// Classifies a controller main/sub code. main_code == 0 yields no fault;
/// codes missing from the catalog are reported raw as an unknown FAULT.
Fault classifyControllerFault(int main_code, int sub_code);

/// Classifies an SDK return code; unknown codes become an unknown FAULT.
Fault classifySdkCode(int code);

/// Builds a servo-side fault (tracking error, watchdog, solver...).
Fault servoFault(const std::string& description, Policy policy = Policy::kOperatorReset,
                 Severity severity = Severity::kError);

/// Worst active fault among all robot signals, controller codes and ServoJ
/// results. feedback_timeout_s bounds the acceptable realtime-state age.
Fault classifyRobot(const RobotSignals& signals, double feedback_timeout_s);

/// Returns the fault with the stricter policy (then the higher severity).
const Fault& worst(const Fault& a, const Fault& b);

/// Recovery procedure text for documentation and the UI.
std::string recoverySequence(Policy policy);

/// Markdown tables of the complete catalog (used to generate the docs).
std::string catalogMarkdown();

}  // namespace fairino_cartesian_servo
