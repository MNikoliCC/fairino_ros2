#include "fairino_cartesian_servo/fault_catalog.hpp"

#include <sstream>

namespace fairino_cartesian_servo {

namespace {

constexpr const char* kNotTested = "not tested";
constexpr const char* kControllerSource =
    "[FAIRINO manual 3.9.9, Appendix 1](https://fairino-doc-en.readthedocs.io/3.9.9/CobotsManual/appendix.html)";
constexpr const char* kSdkSource =
    "[FAIRINO SDK error codes 3.9.9](https://fairino-doc-en.readthedocs.io/3.9.9/SDKManual/errcode.html), "
    "`libfairino/include/robot_error.h`";
constexpr const char* kSignalSource =
    "`ROBOT_STATE_PKG` in `libfairino/include/robot_types.h` (SDK 3.9.9)";

constexpr auto A = Policy::kAutoRecover;
constexpr auto O = Policy::kOperatorReset;
constexpr auto L = Policy::kLockout;
constexpr auto W = Severity::kWarning;
constexpr auto E = Severity::kError;
constexpr auto C = Severity::kCritical;

int policyRank(Policy policy) { return static_cast<int>(policy); }

}  // namespace

const char* toString(Severity severity) {
  switch (severity) {
    case Severity::kNone: return "none";
    case Severity::kInfo: return "info";
    case Severity::kWarning: return "warning";
    case Severity::kError: return "error";
    case Severity::kCritical: return "critical";
  }
  return "unknown";
}

const char* toString(Policy policy) {
  switch (policy) {
    case Policy::kNone: return "none";
    case Policy::kAutoRecover: return "auto_recover";
    case Policy::kOperatorReset: return "operator_reset";
    case Policy::kLockout: return "lockout";
  }
  return "unknown";
}

std::string Fault::signature() const {
  std::ostringstream text;
  text << source << ':' << main_code << '/' << sub_code << '/' << sdk_code;
  if (source == "servo" || source == "signal") {
    text << ':' << description;
  }
  return text.str();
}

const std::vector<ControllerFaultEntry>& controllerFaultTable() {
  // Policies: auto recovery only for resettable faults caused by the command
  // stream itself (speed, reachability, singularity) after which the
  // controller has stopped the robot. Physical events (collision, limits,
  // IO, drives) always need an operator; non-resettable faults lock out.
  static const std::vector<ControllerFaultEntry> table = {
      {1, 1, 1, "Joint command point error", true, E, O,
       "Check the commanded joint target, then Reset Fault.", kNotTested, kNotTested},
      {1, 2, 15, "Linear/arc/circle/helix/TPD/spline command point or tool mismatch errors (sub 2-15)", true, E, O,
       "Planned-motion command error; check the program/tool, then Reset Fault.", kNotTested, kNotTested},
      {1, 17, 19, "PTP/TPD/LIN-ARC joint command limit exceeded", true, E, O,
       "Target exceeds joint limits; move the target inside the limits, then Reset Fault.", "observed 1/17 (SDK MoveJ beyond the J3 limit); ResetAllError cleared it", kNotTested},
      {1, 20, 20, "Cartesian space command overspeed, not resettable", false, C, L,
       "Not resettable: inspect the motion, restart the controller per FAIRINO, review speed limits.", kNotTested,
       kNotTested},
      {1, 21, 21, "Joint space torque command limit exceeded", true, E, O,
       "Check payload/dynamics settings in the WebApp, then Reset Fault.", kNotTested, kNotTested},
      {1, 22, 22, "JOG joint command limit exceeded", true, W, O, "Jog back inside the limits, then Reset Fault.",
       kNotTested, kNotTested},
      {1, 23, 28, "Axis 1-6 joint space command speed limit exceeded (sub 23 = J1 ... 28 = J6)", true, E, A,
       "Whitelisted: controlled recovery to HOLD. If it repeats, lower OInK/Cartesian limits or the target speed.",
       "observed 1/23 during ServoJ (simulator stall + step); auto recovery 0.06 s -> HOLD", kNotTested},
      {1, 29, 29, "Joint feedback speed limit exceeded, not resettable", false, C, L,
       "Not resettable: stop, inspect the robot, restart the controller per FAIRINO.", kNotTested, kNotTested},
      {1, 30, 30, "Joint command and feedback deviation too large, not resettable, requires restart", false, C, L,
       "Power cycle / restart the controller; check payload, collisions and command continuity.", "not reproducible; mock state-machine test S5 -> LOCKOUT",
       kNotTested},
      {1, 31, 31, "DMP target point error (including tool mismatch)", true, E, O, "Check the DMP target, Reset Fault.",
       kNotTested, kNotTested},
      {1, 33, 34, "Joint configuration changed between commands (singular pose on the path)", true, E, O,
       "Use a joint-space motion or another target, then Reset Fault.", kNotTested, kNotTested},
      {1, 35, 36, "LIN command joint speed / adaptive speed threshold exceeded", true, E, O,
       "Planned LIN motion too fast; reduce speed, then Reset Fault.", kNotTested, kNotTested},
      {1, 37, 37, "Unreachable point in trajectory", true, E, A,
       "Whitelisted: recovery to HOLD; move the gizmo to a reachable pose before resuming.", kNotTested, kNotTested},
      {1, 38, 38, "Unreachable point in trajectory - singular pose", true, E, A,
       "Whitelisted: recovery to HOLD; move the gizmo away from the singularity before resuming.", kNotTested,
       kNotTested},
      {1, 49, 56, "Arc/weave command sequence or weaving trajectory errors (sub 49-56)", true, E, O,
       "Welding/weaving program error; fix the program, then Reset Fault.", kNotTested, kNotTested},
      {1, 65, 66, "Laser sensor command deviation / seam tracking interrupted", true, E, O,
       "Check the laser sensor, then Reset Fault.", kNotTested, kNotTested},
      {1, 81, 81, "External axis command speed limit exceeded", true, E, O,
       "Check the external axis motion, then Reset Fault.", kNotTested, kNotTested},
      {1, 82, 82, "External axis command and feedback deviation too large, not resettable", false, C, L,
       "Home the external axis or restart the controller per FAIRINO.", kNotTested, kNotTested},
      {1, 83, 84, "Extended peripheral (external axis/IO) communication error or packet loss", true, E, O,
       "Check peripheral wiring/communication, then Reset Fault.", kNotTested, kNotTested},
      {1, 97, 97, "Conveyor tracking start/reference pose change too large", true, E, O,
       "Check conveyor tracking setup, then Reset Fault.", kNotTested, kNotTested},
      {1, 113, 118, "Constant force control exceeded max adjustment (X/Y/Z/RX/RY/RZ)", true, E, O,
       "Check the force control task, then Reset Fault.", kNotTested, kNotTested},
      {1, 119, 123, "External sensor data error or force-guided motion failed (sub 119-123)", true, E, O,
       "Check the sensor/force task, then Reset Fault.", kNotTested, kNotTested},
      {1, 129, 130, "Max torque record points exceeded / speed switching error", true, E, O,
       "Check the program, then Reset Fault.", kNotTested, kNotTested},
      {1, 147, 149, "Focus following error / attitude speed limit / joint status word abnormal", true, E, O,
       "Inspect the reported condition, then Reset Fault.", kNotTested, kNotTested},
      {2, 1, 6, "Axis 1-6 drive fault, not resettable", false, C, L,
       "Drive fault: inspect drives/cabling, power cycle per FAIRINO; WebApp alarm log.", kNotTested, kNotTested},
      {3, 1, 6, "Axis 1-6 soft limit exceeded", true, E, O,
       "Joint outside the safe range: no automatic recovery. Reset, then jog/drag the joint back inside the "
       "soft limits in the WebApp.",
       kNotTested, kNotTested},
      {4, 1, 6, "Axis 1-6 collision fault", true, E, O,
       "Collision: inspect the cell and people first. Never lower collision sensitivity to clear it. Then Recover.",
       "not reproducible; mock test S7 (collision signal) -> FAULT, operator Recover", kNotTested},
      {4, 7, 7, "End effector collision fault", true, E, O,
       "Collision: inspect the cell and tool first, then Recover.", kNotTested, kNotTested},
      {5, 1, 1, "Active slave count error, not resettable", false, C, L,
       "EtherCAT slave count wrong: power cycle and check the bus.", kNotTested, kNotTested},
      {6, 1, 6, "Slave offline/state/configuration/initialization/mailbox error, not resettable", false, C, L,
       "EtherCAT slave fault: power cycle, check the bus and configuration.", kNotTested, kNotTested},
      {7, 1, 14, "IO error (channel, value, WaitDI/AI/Axle/Aux timeouts, arc/search/conveyor timeouts)", true, W, O,
       "Check the IO configuration/signals, then Reset Fault.", kNotTested, kNotTested},
      {8, 1, 1, "Gripper motion timeout error", true, W, O, "Check the gripper, then Reset Fault.", kNotTested,
       kNotTested},
      {9, 1, 14, "Configuration file or robot model error, not resettable", false, C, L,
       "Controller configuration invalid: fix in the WebApp (robot model / config files), restart.", kNotTested,
       kNotTested},
      {10, 1, 1, "Singular pose", true, E, A,
       "Whitelisted: recovery to HOLD; move the gizmo away from the singularity before resuming.", kNotTested,
       kNotTested},
      {11, 1, 6, "Axis 1-6 drive communication error, not resettable", false, C, L,
       "Drive communication fault: power cycle, check cabling per FAIRINO.", kNotTested, kNotTested},
      {12, 1, 4, "External axis 1-4 soft limit exceeded", true, E, O,
       "Move the external axis inside its limits, then Reset Fault.", kNotTested, kNotTested},
      {13, 1, 17, "Parameter setting error (tool/workpiece numbers, collision level, load, filters, IO ranges)", true, E,
       O, "Correct the parameter in the WebApp, then Reset Fault.", kNotTested, kNotTested},
  };
  return table;
}

const std::vector<SdkCodeEntry>& sdkCodeTable() {
  static const std::vector<SdkCodeEntry> table = {
      {-16, "Socket receive failed", C, L, "Communication with the controller lost; robot state unknown.", kNotTested,
       kNotTested},
      {-15, "Socket send failed", C, L, "Communication with the controller lost; robot state unknown.", kNotTested,
       kNotTested},
      {-4, "XML-RPC interface execution failed", E, O, "Check the controller log; Reset Fault after inspection.",
       kNotTested, kNotTested},
      {-3, "XML-RPC communication failed", C, L, "Check network and controller IP; robot state unknown.", kNotTested,
       kNotTested},
      {-2, "Abnormal communication with the controller", C, L,
       "Check the connection; robot state unknown, no automatic recovery.", kNotTested, kNotTested},
      {-1, "Other error", E, O, "Check the controller log.", kNotTested, kNotTested},
      {3, "Interface parameter count mismatch", E, O, "Software error in the caller.", kNotTested, kNotTested},
      {4, "Interface parameter value out of range", E, O, "Software error in the caller (command out of range).",
       kNotTested, kNotTested},
      {14, "Interface execution failed", E, O,
       "Controller rejected the command; the WebApp/status main-sub code gives the cause.", "observed: calls rejected with 14 while 1/17 was active", kNotTested},
      {18, "Robot program is running", W, O, "Stop the WebApp program before servoing.", kNotTested, kNotTested},
      {28, "Inverse kinematics calculation failed", E, A,
       "Whitelisted: recovery to HOLD; choose a reachable pose.", kNotTested, kNotTested},
      {29, "ServoJ joint overrun (joint value out of range)", E, A,
       "Whitelisted only while measured joints are inside their limits: recovery to HOLD. Not a joint-speed "
       "overspeed code.",
       "not reproducible; mock test S10 -> auto recovery -> HOLD", kNotTested},
      {30, "Non-resettable fault, power off and restart the control box", C, L, "Power cycle the control box.",
       kNotTested, kNotTested},
      {31, "Emergency stop released, power off and restart the control box", C, L, "Power cycle the control box.",
       kNotTested, kNotTested},
      {32, "Joint over limit", E, O, "Drag the joint back inside the soft limits, then Reset Fault.", "observed: MoveJ target beyond the J3 limit returned 32",
       kNotTested},
      {38, "Singular pose, calculation failed", E, A, "Whitelisted: recovery to HOLD; change the pose.", kNotTested,
       kNotTested},
      {40, "Speed percentage exceeds limit", W, O, "Use a speed within 0-100 %.", kNotTested, kNotTested},
      {42, "Posture changes too much", E, O, "Insert an intermediate pose.", kNotTested, kNotTested},
      {44, "Robot attitude angle exceeds the limit", E, O, "Check the target attitude.", kNotTested, kNotTested},
      {64, "Not added to the instruction queue", E, O, "Check the controller log.", kNotTested, kNotTested},
      {75, "Channel error", W, O, "IO number out of range; check the IO configuration.", kNotTested, kNotTested},
      {99, "Safety stop triggered", C, L, "Check the safety stop inputs; manual intervention required.", "not reproducible; safety signals covered by mock tests S4/S8",
       kNotTested},
      {101, "Robot is not enabled", W, O, "Recover (enables the robot) after checking the cell.", kNotTested,
       kNotTested},
      {112, "Target pose cannot be reached", E, A, "Whitelisted: recovery to HOLD; choose a reachable pose.",
       kNotTested, kNotTested},
      {171, "Joint soft limit exceeded, soft limit protection failed to start", E, O,
       "Move all joints inside the soft limits.", kNotTested, kNotTested},
      {182, "Given physical speed exceeds limit", E, O, "Reduce the commanded speed.", kNotTested, kNotTested},
      {183, "Given physical acceleration exceeds limit", E, O, "Reduce the commanded acceleration.", kNotTested,
       kNotTested},
      {185, "Fault signal triggered, motion stopped", C, L, "Check the fault input signal.", kNotTested, kNotTested},
      {186, "Emergency stop signal triggered, motion stopped", C, L, "Release E-stop only when safe; manual reset.",
       kNotTested, kNotTested},
      {187, "Current collision level too low", W, O, "Collision level is a safety setting: change only in the WebApp.",
       kNotTested, kNotTested},
      {204, "Command queue is full", E, O, "Command rate too high; check cmdT and the stream.", kNotTested,
       kNotTested},
      {207, "Automatic mode safety stop requires teach pendant three-position enable", C, L,
       "Use the teach pendant enabling device.", kNotTested, kNotTested},
      {211, "Safety parameter operation password error", W, O, "Safety configuration only: check the password.",
       kNotTested, kNotTested},
      {212, "Safety operation password verification required first", W, O,
       "Safety configuration only: SafetyOPPasswordCheck first.", kNotTested, kNotTested},
      {215, "Safety configuration parameter setting failed", E, O, "Use the WebApp Safety page.", kNotTested,
       kNotTested},
      {216, "Safety configuration checksum update failed", E, O, "Use the WebApp Safety page.", kNotTested,
       kNotTested},
      {220, "Level 3 reduced mode has been triggered", C, L, "Check the reduced-mode safety inputs.", kNotTested,
       kNotTested},
  };
  return table;
}

const std::vector<SignalEntry>& signalTable() {
  static const std::vector<SignalEntry> table = {
      {"status_lost", "Robot status missing/stale or SDK communication not normal (robot state unknown)", C, L,
       "Restore communication; the hardware must be restarted before servoing.", "observed: SimMachine paused 1.5 s -> read() ERROR after 0.5 s, LOCKOUT; manual hardware reactivation + Reset Fault (also mock S9)", kNotTested},
      {"emergency_stop", "Emergency stop pressed (EmergencyStop / button box)", C, L,
       "Release E-stop only when safe; Reset Fault; power cycle if SDK code 31 / alarm_reboot_robot.", "mock test S4: LOCKOUT, reset refused while pressed",
       kNotTested},
      {"safety_stop", "Safety stop SI0/SI1 active", C, L, "Clear the safety stop input, then Reset Fault.", kNotTested,
       kNotTested},
      {"safety_door", "Safety door open", C, L, "Close the door, then Reset Fault.", "mock test S8 -> LOCKOUT", kNotTested},
      {"safety_plane", "Safety plane (safety wall) entered", C, L,
       "Move out of the safety plane with the WebApp/drag, then Reset Fault.", kNotTested, kNotTested},
      {"safety_data_abnormal", "Safety data state abnormal", C, L,
       "Safety configuration invalid: check WebApp Safety pages/checksum.", kNotTested, kNotTested},
      {"alarm_reboot", "Controller requests power cycle (E-stop release / joint communication)", C, L,
       "Power cycle the control box.", kNotTested, kNotTested},
      {"alarm_check_estop", "E-stop button communication abnormal", C, L, "Check the E-stop button and cabling.",
       kNotTested, kNotTested},
      {"driver_fault", "Drive alarm, drive communication error", C, L, "Inspect drives; power cycle per FAIRINO.",
       kNotTested, kNotTested},
      {"ctrl_box_error", "Control box error (ctrlBoxError) reported", E, O,
       "Read the control box error in the WebApp; Reset Fault after the cause is cleared.", kNotTested,
       kNotTested},
      {"collision", "Collision detected (collisionState)", E, O,
       "Inspect the cell; never change collision sensitivity automatically; then Recover.", "mock test S7", kNotTested},
      {"out_soft_limit", "Joint outside the soft limit", E, O, "Jog/drag back inside the limits, then Reset Fault.",
       kNotTested, kNotTested},
      {"abnormal_stop", "Abnormal stop flag (abnomalStop): informational, shown in the UI only", Severity::kInfo,
       Policy::kNone,
       "Stays 1 after a successful ResetAllError() on SimMachine 3.9.9; the main/sub code decides.",
       "yes: set with main=1 sub=23 and still set after reset (2026-10-14)", kNotTested},
      {"robot_disabled", "Robot disabled (rbtEnableState = 0) without a main/sub code", W, O,
       "Recover enables the robot after a check. With a main/sub code the code decides.", "mock test S11: Recover enables",
       kNotTested},
      {"command_rejected", "Hardware stopped ServoJ: discontinuity guard or non-finite command", E, O,
       "Software fault in the command stream; Reset Fault re-synchronizes command and target.", "observed: jump guard trip under CPU load (before the gap-aware guard)",
       kNotTested},
  };
  return table;
}

Fault classifyControllerFault(int main_code, int sub_code) {
  Fault fault;
  if (main_code == 0) {
    return fault;
  }
  fault.source = "controller";
  fault.main_code = main_code;
  fault.sub_code = sub_code;
  for (const auto& entry : controllerFaultTable()) {
    if (entry.main_code == main_code && sub_code >= entry.sub_min && sub_code <= entry.sub_max) {
      fault.description = entry.description;
      fault.severity = entry.severity;
      fault.policy = entry.policy;
      fault.action = entry.operator_action;
      return fault;
    }
  }
  fault.known = false;
  fault.description = "Unknown FAIRINO controller fault (raw main=" + std::to_string(main_code) +
                      ", sub=" + std::to_string(sub_code) + ")";
  fault.severity = Severity::kError;
  fault.policy = Policy::kOperatorReset;
  fault.action =
      "Not in the catalog: read the WebApp alarm text and the FAIRINO manual. No automatic recovery; "
      "Reset Fault only after the cause is understood.";
  return fault;
}

Fault classifySdkCode(int code) {
  Fault fault;
  if (code == 0) {
    return fault;
  }
  fault.source = "sdk";
  fault.sdk_code = code;
  for (const auto& entry : sdkCodeTable()) {
    if (entry.code == code) {
      fault.description = entry.description;
      fault.severity = entry.severity;
      fault.policy = entry.policy;
      fault.action = entry.operator_action;
      return fault;
    }
  }
  fault.known = false;
  fault.description = "Unknown FAIRINO SDK return code " + std::to_string(code);
  fault.severity = Severity::kError;
  fault.policy = Policy::kOperatorReset;
  fault.action = "Not in the catalog: check the FAIRINO SDK error table; no automatic recovery.";
  return fault;
}

Fault servoFault(const std::string& description, Policy policy, Severity severity) {
  Fault fault;
  fault.source = "servo";
  fault.description = description;
  fault.policy = policy;
  fault.severity = severity;
  fault.action = policy == Policy::kLockout ? "Manual intervention required."
                                            : "Check the cause, then Reset Fault (re-synchronizes command and target).";
  return fault;
}

const Fault& worst(const Fault& a, const Fault& b) {
  if (policyRank(b.policy) != policyRank(a.policy)) {
    return policyRank(b.policy) > policyRank(a.policy) ? b : a;
  }
  return static_cast<int>(b.severity) > static_cast<int>(a.severity) ? b : a;
}

namespace {

Fault signalFault(const char* signal) {
  for (const auto& entry : signalTable()) {
    if (std::string(entry.signal) == signal) {
      Fault fault;
      fault.source = "signal";
      fault.description = entry.description;
      fault.severity = entry.severity;
      fault.policy = entry.policy;
      fault.action = entry.operator_action;
      return fault;
    }
  }
  return servoFault(std::string("unknown signal ") + signal);
}

}  // namespace

Fault classifyRobot(const RobotSignals& s, double feedback_timeout_s) {
  Fault result;
  if (!s.status_available || !s.connected || s.feedback_age_s > feedback_timeout_s) {
    return signalFault("status_lost");
  }
  auto consider = [&result](const Fault& candidate) { result = worst(result, candidate); };

  if (s.emergency_stop || s.button_box_stop) consider(signalFault("emergency_stop"));
  if (s.safety_stop0 || s.safety_stop1) consider(signalFault("safety_stop"));
  if (s.safety_door) consider(signalFault("safety_door"));
  if (s.safety_plane) consider(signalFault("safety_plane"));
  if (s.safety_data_abnormal) consider(signalFault("safety_data_abnormal"));
  if (s.alarm_reboot != 0) consider(signalFault("alarm_reboot"));
  if (s.alarm_check_estop) consider(signalFault("alarm_check_estop"));
  if (s.driver_alarm_axis != 0 || s.driver_error_axis != 0 || s.driver_com_error) {
    consider(signalFault("driver_fault"));
  }
  if (s.ctrl_box_error != 0) {
    Fault fault = signalFault("ctrl_box_error");
    fault.description += " (code " + std::to_string(s.ctrl_box_error) + ")";
    consider(fault);
  }
  if (s.collision) consider(signalFault("collision"));
  if (s.out_soft_limit) consider(signalFault("out_soft_limit"));
  // The controller raises abnomalStop (and may disable the robot) together
  // with a main/sub fault (seen on SimMachine with main=1 sub=23). Then the
  // main/sub code decides the policy; on their own they need an operator.
  // abnomalStop is informational only: SimMachine 3.9.9 keeps it at 1 after
  // a successful ResetAllError() (main/sub = 0).
  if (s.main_code == 0 && !s.robot_enabled) {
    consider(signalFault("robot_disabled"));
  }
  if (s.command_rejected) {
    Fault fault = signalFault("command_rejected");
    if (!s.command_rejected_reason.empty()) {
      fault.description += ": " + s.command_rejected_reason;
    }
    consider(fault);
  }
  consider(classifyControllerFault(s.main_code, s.sub_code));
  if (s.servoj_failed) consider(classifySdkCode(s.servoj_code));
  return result;
}

std::string recoverySequence(Policy policy) {
  switch (policy) {
    case Policy::kAutoRecover:
      return "R1 (automatic, whitelisted): ACTIVE->HOLD, stop new targets, ServoMoveEnd(UDP), StopMotion(), "
             "wait standstill, ResetAllError(), GetRobotErrorCode()==0/0, RobotEnable(1) if disabled, "
             "ServoMoveStart(UDP), q_cmd=q_measured, target=FK(q_measured), reset filter/OInK, HOLD; "
             "operator resumes. Max 2 attempts, 3 s cooldown, repeat -> LOCKOUT.";
    case Policy::kOperatorReset:
      return "R2 (operator): same steps as R1, started only by the operator. Recover = R1 incl. RobotEnable(1); "
             "Reset Fault = acknowledgement without RobotEnable.";
    case Policy::kLockout:
      return "LOCKOUT (no software recovery): remove the cause manually, then the operator may Reset Fault "
             "once no lockout signal is active (power cycle where stated).";
    case Policy::kNone:
      return "-";
  }
  return "-";
}

std::string catalogMarkdown() {
  auto escape = [](std::string text) {
    std::string out;
    for (char c : text) {
      if (c == '|') {
        out += "\\|";
      } else {
        out += c;
      }
    }
    return out;
  };
  auto sequence_id = [](Policy policy) {
    switch (policy) {
      case Policy::kAutoRecover: return "R1";
      case Policy::kOperatorReset: return "R2";
      case Policy::kLockout: return "none (LOCKOUT)";
      default: return "-";
    }
  };
  std::ostringstream md;
  md << "### Controller main/sub fault codes\n\n"
        "| code | subcode | description | severity | automatic recovery allowed | recovery sequence | "
        "manual/WebApp action | source | tested in SimMachine | tested on real robot |\n"
        "|---:|---|---|---|---|---|---|---|---|---|\n";
  for (const auto& e : controllerFaultTable()) {
    md << "| " << e.main_code << " | "
       << (e.sub_min == e.sub_max ? std::to_string(e.sub_min)
                                  : std::to_string(e.sub_min) + "-" + std::to_string(e.sub_max))
       << " | " << escape(e.description) << (e.resettable ? "" : " (FAIRINO: not resettable)") << " | "
       << toString(e.severity) << " | " << (e.policy == Policy::kAutoRecover ? "yes" : "no") << " | "
       << sequence_id(e.policy) << " | " << escape(e.operator_action) << " | " << kControllerSource << " | "
       << e.simmachine_test << " | " << e.real_robot_test << " |\n";
  }
  md << "| other | any | Unknown code: shown raw, FAULT, no guessed recovery | error | no | R2 (Reset Fault only) "
        "| Read the WebApp alarm text and manual | - | mock test S6 | not tested |\n";

  md << "\n### SDK return codes (ServoJ and API calls)\n\n"
        "| code | subcode | description | severity | automatic recovery allowed | recovery sequence | "
        "manual/WebApp action | source | tested in SimMachine | tested on real robot |\n"
        "|---:|---|---|---|---|---|---|---|---|---|\n";
  md << "| 0 | - | ERR_SUCCESS | none | - | - | - | " << kSdkSource << " | yes (every ServoJ call) | not tested |\n";
  for (const auto& e : sdkCodeTable()) {
    md << "| " << e.code << " | - | " << escape(e.description) << " | " << toString(e.severity) << " | "
       << (e.policy == Policy::kAutoRecover ? "yes" : "no") << " | " << sequence_id(e.policy) << " | "
       << escape(e.operator_action) << " | " << kSdkSource << " | " << e.simmachine_test << " | "
       << e.real_robot_test << " |\n";
  }
  md << "| other | - | Unknown code: shown raw, FAULT | error | no | R2 (Reset Fault only) | Check the SDK table "
        "| - | - | - |\n";

  md << "\n### Safety and status signals\n\n"
        "| signal | description | severity | automatic recovery allowed | recovery sequence | "
        "manual/WebApp action | source | tested in SimMachine | tested on real robot |\n"
        "|---|---|---|---|---|---|---|---|---|\n";
  for (const auto& e : signalTable()) {
    md << "| `" << e.signal << "` | " << escape(e.description) << " | " << toString(e.severity) << " | "
       << (e.policy == Policy::kAutoRecover ? "yes" : "no") << " | " << sequence_id(e.policy) << " | "
       << escape(e.operator_action) << " | " << kSignalSource << " | " << e.simmachine_test << " | "
       << e.real_robot_test << " |\n";
  }
  md << "\n### Recovery sequences\n\n";
  md << "- " << recoverySequence(Policy::kAutoRecover) << "\n";
  md << "- " << recoverySequence(Policy::kOperatorReset) << "\n";
  md << "- " << recoverySequence(Policy::kLockout) << "\n";
  return md.str();
}

}  // namespace fairino_cartesian_servo
