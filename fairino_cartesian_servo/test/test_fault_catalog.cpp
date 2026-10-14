#include <gtest/gtest.h>

#include "fairino_cartesian_servo/fault_catalog.hpp"

using fairino_cartesian_servo::classifyControllerFault;
using fairino_cartesian_servo::classifyRobot;
using fairino_cartesian_servo::classifySdkCode;
using fairino_cartesian_servo::Policy;
using fairino_cartesian_servo::RobotSignals;

namespace {
RobotSignals healthy() {
  RobotSignals s;
  s.status_available = true;
  s.connected = true;
  s.feedback_age_s = 0.004;
  return s;
}
}  // namespace

TEST(FaultCatalog, NoFaultWhenHealthy) {
  EXPECT_FALSE(classifyRobot(healthy(), 0.1).active());
  EXPECT_FALSE(classifyControllerFault(0, 0).active());
  EXPECT_FALSE(classifySdkCode(0).active());
}

TEST(FaultCatalog, MinimumSdkCodes) {
  EXPECT_EQ(classifySdkCode(28).policy, Policy::kAutoRecover);
  EXPECT_EQ(classifySdkCode(29).policy, Policy::kAutoRecover);
  EXPECT_EQ(classifySdkCode(30).policy, Policy::kLockout);
  EXPECT_EQ(classifySdkCode(38).policy, Policy::kAutoRecover);
  EXPECT_EQ(classifySdkCode(75).policy, Policy::kOperatorReset);
  EXPECT_EQ(classifySdkCode(99).policy, Policy::kLockout);
  EXPECT_EQ(classifySdkCode(112).policy, Policy::kAutoRecover);
  // Code 29 is a joint position overrun, not a joint speed overspeed.
  EXPECT_NE(classifySdkCode(29).description.find("joint overrun"), std::string::npos);
}

TEST(FaultCatalog, UnknownCodesAreRawFaultsWithoutAutoRecovery) {
  const auto controller = classifyControllerFault(42, 7);
  EXPECT_FALSE(controller.known);
  EXPECT_EQ(controller.policy, Policy::kOperatorReset);
  EXPECT_NE(controller.description.find("main=42"), std::string::npos);
  EXPECT_NE(controller.description.find("sub=7"), std::string::npos);
  const auto sdk = classifySdkCode(12345);
  EXPECT_FALSE(sdk.known);
  EXPECT_EQ(sdk.policy, Policy::kOperatorReset);
}

TEST(FaultCatalog, ControllerPolicies) {
  // Joint command speed limit (overspeed) is whitelisted.
  for (int sub = 23; sub <= 28; ++sub) {
    EXPECT_EQ(classifyControllerFault(1, sub).policy, Policy::kAutoRecover) << sub;
  }
  // Not resettable codes lock out.
  EXPECT_EQ(classifyControllerFault(1, 20).policy, Policy::kLockout);
  EXPECT_EQ(classifyControllerFault(1, 29).policy, Policy::kLockout);
  EXPECT_EQ(classifyControllerFault(1, 30).policy, Policy::kLockout);
  EXPECT_EQ(classifyControllerFault(2, 3).policy, Policy::kLockout);
  EXPECT_EQ(classifyControllerFault(11, 6).policy, Policy::kLockout);
  // Collision and soft limit always need an operator.
  EXPECT_EQ(classifyControllerFault(4, 2).policy, Policy::kOperatorReset);
  EXPECT_EQ(classifyControllerFault(3, 1).policy, Policy::kOperatorReset);
  EXPECT_EQ(classifyControllerFault(10, 1).policy, Policy::kAutoRecover);
}

TEST(FaultCatalog, SafetySignalsLockOut) {
  auto s = healthy();
  s.emergency_stop = true;
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kLockout);
  s = healthy();
  s.safety_stop1 = true;
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kLockout);
  s = healthy();
  s.safety_door = true;
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kLockout);
  s = healthy();
  s.safety_plane = true;
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kLockout);
  s = healthy();
  s.safety_data_abnormal = true;
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kLockout);
  s = healthy();
  s.connected = false;
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kLockout);
  s = healthy();
  s.feedback_age_s = 0.5;
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kLockout);
}

TEST(FaultCatalog, WorstFaultWins) {
  auto s = healthy();
  s.main_code = 1;
  s.sub_code = 25;  // whitelisted
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kAutoRecover);
  s.collision = true;  // operator reset
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kOperatorReset);
  s.emergency_stop = true;  // lockout
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kLockout);
}

TEST(FaultCatalog, AbnormalStopFollowsTheMainCode) {
  // SimMachine 3.9.9 reports abnomalStop together with main=1 sub=23.
  auto s = healthy();
  s.abnormal_stop = true;
  s.main_code = 1;
  s.sub_code = 23;
  const auto fault = classifyRobot(s, 0.1);
  EXPECT_EQ(fault.source, "controller");
  EXPECT_EQ(fault.policy, Policy::kAutoRecover);
  s.robot_enabled = false;  // "stop, alarm and disable" strategies
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kAutoRecover);
  // The flag alone is informational (it stays set after a reset); a disabled
  // robot without a code needs an operator.
  s.main_code = 0;
  s.sub_code = 0;
  EXPECT_EQ(classifyRobot(s, 0.1).policy, Policy::kOperatorReset);
  s.robot_enabled = true;
  EXPECT_FALSE(classifyRobot(s, 0.1).active());
}

TEST(FaultCatalog, ServoJFailureUsesSdkTable) {
  auto s = healthy();
  s.servoj_failed = true;
  s.servoj_code = 29;
  const auto fault = classifyRobot(s, 0.1);
  EXPECT_EQ(fault.source, "sdk");
  EXPECT_EQ(fault.sdk_code, 29);
  EXPECT_EQ(fault.policy, Policy::kAutoRecover);
}

TEST(FaultCatalog, MarkdownContainsAllColumns) {
  const auto md = fairino_cartesian_servo::catalogMarkdown();
  EXPECT_NE(md.find("automatic recovery allowed"), std::string::npos);
  EXPECT_NE(md.find("tested on real robot"), std::string::npos);
  EXPECT_NE(md.find("| 30 | - |"), std::string::npos);
}
