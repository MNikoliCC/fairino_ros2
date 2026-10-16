#include <gtest/gtest.h>

#include <pinocchio/spatial/explog.hpp>

#include "fairino_cartesian_servo/servo_math.hpp"

using fairino_cartesian_servo::alignToolAxis;
using fairino_cartesian_servo::poseDistance;
using fairino_cartesian_servo::rateLimitPose;
using fairino_cartesian_servo::TimingRecorder;

TEST(RateLimiter, LimitsTranslationAndRotation) {
  const pinocchio::SE3 current = pinocchio::SE3::Identity();
  const pinocchio::SE3 target(pinocchio::exp3(Eigen::Vector3d(0.0, 0.0, 1.0)), Eigen::Vector3d(1.0, 0.0, 0.0));
  const double dt = 0.001;
  const auto limited = rateLimitPose(current, target, 0.5, 2.0, dt);
  double translation = 0.0;
  double rotation = 0.0;
  poseDistance(current, limited, translation, rotation);
  EXPECT_NEAR(translation, 0.5 * dt, 1e-12);
  EXPECT_NEAR(rotation, 2.0 * dt, 1e-9);
}

TEST(RateLimiter, ReachesCloseTargetsAndDisabledLimits) {
  const pinocchio::SE3 current = pinocchio::SE3::Identity();
  const pinocchio::SE3 target(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1e-5, 0.0, 0.0));
  const auto limited = rateLimitPose(current, target, 1.0, 1.0, 0.001);
  EXPECT_TRUE(limited.isApprox(target));
  const pinocchio::SE3 far(pinocchio::exp3(Eigen::Vector3d(0.5, 0.0, 0.0)), Eigen::Vector3d(2.0, 0.0, 0.0));
  EXPECT_TRUE(rateLimitPose(current, far, 0.0, 0.0, 0.001).isApprox(far));
}

TEST(TimingRecorder, PercentilesAndOverruns) {
  TimingRecorder recorder("test", 1.0, 1024);
  for (int i = 1; i <= 100; ++i) {
    recorder.record(i * 0.02);  // 0.02 .. 2.0 ms
  }
  const auto summary = recorder.summarize();
  EXPECT_EQ(summary.samples, 100u);
  EXPECT_NEAR(summary.mean_ms, 1.01, 1e-6);
  EXPECT_NEAR(summary.max_ms, 2.0, 1e-6);
  EXPECT_NEAR(summary.p99_ms, 1.98, 1e-6);
  EXPECT_NEAR(summary.p95_ms, 1.90, 1e-6);
  EXPECT_EQ(summary.overruns_window, 50u);
  // A second window only contains new samples.
  recorder.record(0.5);
  const auto second = recorder.summarize();
  EXPECT_EQ(second.samples, 1u);
  EXPECT_EQ(second.overruns_window, 0u);
  EXPECT_EQ(second.overruns_total, 50u);
}

TEST(TimingRecorder, ResetIsAppliedByTheProducer) {
  TimingRecorder recorder("test", 1.0, 1024);
  for (int i = 0; i < 10; ++i) {
    recorder.record(2.0);
  }
  recorder.reset();
  // Until the producer records again, the totals report nothing.
  EXPECT_EQ(recorder.summarizeTotal().samples, 0u);
  recorder.record(0.5);
  recorder.record(0.7);
  const auto total = recorder.summarizeTotal();
  EXPECT_EQ(total.samples, 2u);
  EXPECT_EQ(total.overruns_total, 0u);
  EXPECT_NEAR(total.max_ms, 0.7, 1e-6);
  EXPECT_NEAR(total.p99_ms, 0.7, 1e-6);  // percentiles exclude pre-reset samples
}

TEST(AlignToolAxis, PointsZAndKeepsTheRoll) {
  const Eigen::Matrix3d rotation = pinocchio::exp3(Eigen::Vector3d(0.3, -0.2, 1.1));
  const Eigen::Vector3d axis = Eigen::Vector3d(0.2, 0.9, -0.4).normalized();
  const Eigen::Matrix3d aligned = alignToolAxis(rotation, axis);
  EXPECT_TRUE(aligned.col(2).isApprox(axis, 1e-12));
  // Smallest rotation: the turn axis is perpendicular to both z axes, so
  // there is no extra rotation about the tool axis.
  const Eigen::Vector3d turn = pinocchio::log3(aligned * rotation.transpose());
  EXPECT_NEAR(turn.dot(rotation.col(2)), 0.0, 1e-9);
  EXPECT_NEAR(turn.dot(axis), 0.0, 1e-9);
  EXPECT_TRUE(alignToolAxis(rotation, rotation.col(2)).isApprox(rotation));
  EXPECT_TRUE(alignToolAxis(rotation, -rotation.col(2)).col(2).isApprox(-rotation.col(2), 1e-12));
}
