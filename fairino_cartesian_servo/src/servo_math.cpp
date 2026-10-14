#include "fairino_cartesian_servo/servo_math.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <pinocchio/spatial/explog.hpp>

namespace fairino_cartesian_servo {

pinocchio::SE3 rateLimitPose(const pinocchio::SE3& current, const pinocchio::SE3& target, double max_linear_speed,
                             double max_angular_speed, double dt) {
  pinocchio::SE3 result = current;

  Eigen::Vector3d translation_step = target.translation() - current.translation();
  const double distance = translation_step.norm();
  const double max_distance = max_linear_speed * dt;
  if (max_linear_speed > 0.0 && distance > max_distance) {
    translation_step *= max_distance / distance;
  }
  result.translation() = current.translation() + translation_step;

  const Eigen::Matrix3d relative = current.rotation().transpose() * target.rotation();
  Eigen::Vector3d rotation_step = pinocchio::log3(relative);
  const double angle = rotation_step.norm();
  const double max_angle = max_angular_speed * dt;
  if (max_angular_speed > 0.0 && angle > max_angle) {
    rotation_step *= max_angle / angle;
  }
  result.rotation() = current.rotation() * pinocchio::exp3(rotation_step);
  return result;
}

void poseDistance(const pinocchio::SE3& a, const pinocchio::SE3& b, double& translation, double& rotation) {
  translation = (a.translation() - b.translation()).norm();
  rotation = pinocchio::log3(a.rotation().transpose() * b.rotation()).norm();
}

TimingRecorder::TimingRecorder(std::string name, double budget_ms, std::size_t capacity)
    : name_(std::move(name)), budget_ms_(budget_ms), ring_(capacity, 0.0F) {
  scratch_.reserve(capacity);
}

void TimingRecorder::record(double milliseconds) {
  const uint64_t index = write_index_.load(std::memory_order_relaxed);
  if (reset_requested_.load(std::memory_order_acquire)) {
    overruns_total_.store(0, std::memory_order_relaxed);
    total_samples_.store(0, std::memory_order_relaxed);
    total_sum_ms_.store(0.0, std::memory_order_relaxed);
    total_max_ms_.store(0.0, std::memory_order_relaxed);
    base_index_.store(index, std::memory_order_relaxed);
    reset_requested_.store(false, std::memory_order_release);
  }
  ring_[index % ring_.size()] = static_cast<float>(milliseconds);
  write_index_.store(index + 1, std::memory_order_release);
  if (milliseconds > budget_ms_) {
    overruns_total_.fetch_add(1, std::memory_order_relaxed);
  }
  total_samples_.fetch_add(1, std::memory_order_relaxed);
  // Single producer: plain read-modify-write of the atomics is race free.
  total_sum_ms_.store(total_sum_ms_.load(std::memory_order_relaxed) + milliseconds, std::memory_order_relaxed);
  if (milliseconds > total_max_ms_.load(std::memory_order_relaxed)) {
    total_max_ms_.store(milliseconds, std::memory_order_relaxed);
  }
}

namespace {

void fillPercentiles(std::vector<float>& samples, TimingSummary& summary) {
  if (samples.empty()) {
    return;
  }
  const double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
  summary.mean_ms = sum / static_cast<double>(samples.size());
  auto percentile = [&samples](double q) {
    const std::size_t k = std::min(samples.size() - 1, static_cast<std::size_t>(std::ceil(q * samples.size())) - 1);
    std::nth_element(samples.begin(), samples.begin() + static_cast<long>(k), samples.end());
    return static_cast<double>(samples[k]);
  };
  summary.p95_ms = percentile(0.95);
  summary.p99_ms = percentile(0.99);
  summary.max_ms = *std::max_element(samples.begin(), samples.end());
}

}  // namespace

TimingSummary TimingRecorder::summarize() {
  std::lock_guard<std::mutex> lock(consumer_mutex_);
  TimingSummary summary;
  summary.name = name_;
  summary.budget_ms = budget_ms_;
  const uint64_t end = write_index_.load(std::memory_order_acquire);
  uint64_t begin = read_index_;
  if (end - begin > ring_.size()) {
    begin = end - ring_.size();
  }
  scratch_.clear();
  for (uint64_t i = begin; i < end; ++i) {
    scratch_.push_back(ring_[i % ring_.size()]);
  }
  read_index_ = end;
  summary.samples = scratch_.size();
  fillPercentiles(scratch_, summary);
  const uint64_t overruns = overruns_total_.load(std::memory_order_relaxed);
  summary.overruns_total = overruns;
  // The totals may have been reset by the producer since the last summary.
  summary.overruns_window = overruns >= overruns_at_last_summary_ ? overruns - overruns_at_last_summary_ : overruns;
  overruns_at_last_summary_ = overruns;
  return summary;
}

TimingSummary TimingRecorder::summarizeTotal() const {
  TimingSummary summary;
  summary.name = name_;
  summary.budget_ms = budget_ms_;
  if (reset_requested_.load(std::memory_order_acquire)) {
    return summary;  // reset not applied yet: no samples since the reset
  }
  const uint64_t end = write_index_.load(std::memory_order_acquire);
  const uint64_t base = base_index_.load(std::memory_order_relaxed);
  const uint64_t begin = std::max(base, end > ring_.size() ? end - ring_.size() : uint64_t{0});
  std::vector<float> samples;
  samples.reserve(end - begin);
  for (uint64_t i = begin; i < end; ++i) {
    samples.push_back(ring_[i % ring_.size()]);
  }
  fillPercentiles(samples, summary);
  summary.samples = total_samples_.load();
  if (summary.samples > 0) {
    summary.mean_ms = total_sum_ms_.load() / static_cast<double>(summary.samples);
    summary.max_ms = total_max_ms_.load();
  }
  summary.overruns_total = overruns_total_.load();
  summary.overruns_window = summary.overruns_total;
  return summary;
}

void TimingRecorder::reset() {
  std::lock_guard<std::mutex> lock(consumer_mutex_);
  read_index_ = write_index_.load();
  overruns_at_last_summary_ = 0;
  reset_requested_.store(true, std::memory_order_release);
}

}  // namespace fairino_cartesian_servo
