#pragma once

#include <atomic>
#include <mutex>
#include <cstdint>
#include <string>
#include <vector>

#include <pinocchio/spatial/se3.hpp>

namespace fairino_cartesian_servo {

/// Limits one SE(3) reference step to max_linear_speed [m/s] and
/// max_angular_speed [rad/s]. Non-positive limits disable the respective cap.
/// The rotation step is taken along the geodesic (log3) towards the target.
pinocchio::SE3 rateLimitPose(const pinocchio::SE3& current, const pinocchio::SE3& target, double max_linear_speed,
                             double max_angular_speed, double dt);

/// Translation [m] and rotation angle [rad] between two poses.
void poseDistance(const pinocchio::SE3& a, const pinocchio::SE3& b, double& translation, double& rotation);

/// Summary of one timing window.
struct TimingSummary {
  std::string name;
  uint64_t samples = 0;
  double mean_ms = 0.0;
  double p95_ms = 0.0;
  double p99_ms = 0.0;
  double max_ms = 0.0;
  double budget_ms = 0.0;
  uint64_t overruns_window = 0;
  uint64_t overruns_total = 0;
};

/// Single-producer timing recorder. record() is called from the real-time
/// loop and never allocates or locks; summarize() is called from a
/// non-real-time thread and evaluates the samples recorded since its previous
/// call (up to the ring capacity). Every cycle is measured, none is logged.
class TimingRecorder {
public:
  TimingRecorder(std::string name, double budget_ms, std::size_t capacity = 1 << 16);

  void record(double milliseconds);
  TimingSummary summarize();
  /// Summary over everything since construction/reset (mean/max only exact,
  /// percentiles over the last ring capacity).
  TimingSummary summarizeTotal() const;
  /// Requests a reset; the producer applies it at its next record() so the
  /// real-time thread never races a non-atomic update of the totals.
  void reset();

  const std::string& name() const { return name_; }
  double budget() const { return budget_ms_; }

private:
  std::string name_;
  double budget_ms_;
  std::vector<float> ring_;
  std::atomic<uint64_t> write_index_{0};
  uint64_t read_index_ = 0;
  std::atomic<uint64_t> overruns_total_{0};
  std::atomic<uint64_t> total_samples_{0};
  std::atomic<double> total_sum_ms_{0.0};
  std::atomic<double> total_max_ms_{0.0};
  std::atomic<bool> reset_requested_{false};
  std::atomic<uint64_t> base_index_{0};  // first sample of the current totals
  std::mutex consumer_mutex_;            // summarize() vs reset()
  uint64_t overruns_at_last_summary_ = 0;
  std::vector<float> scratch_;
};

}  // namespace fairino_cartesian_servo
