#pragma once

#include <optional>
#include <string>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/scene_context.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/frame.hpp>

namespace fairino_cartesian_servo {

/// OInK task like roboplan::FrameTask, but only the tool position and the
/// direction of the tool z axis are tracked: the rotation about the tool z
/// axis (on the FR20 exactly joint 6) has no cost, so the solver picks it
/// with the least joint motion and within the joint limits. Used for point
/// tracking, where the roll about the viewing axis is free.
///
/// Rows: 3 position (world, as FrameTask) and 3 orientation expressed in the
/// current tool frame; the tool-z orientation row has weight 0.
struct PointingTask : public roboplan::Task {
  PointingTask(const roboplan::Oink& oink, const roboplan::Scene& scene,
               const roboplan::CartesianConfiguration& target_pose, const roboplan::FrameTaskOptions& options = {});

  tl::expected<void, std::string> computeError(const roboplan::SceneContext& context) override;
  tl::expected<void, std::string> computeJacobian(const roboplan::SceneContext& context) override;

  /// Target in the base frame (as FrameTask); only its position and z axis are used.
  void setTargetFrameTransform(const Eigen::Matrix4d& tform) { target_pose.tform = tform; }

  /// diag(sqrt(position_cost) I3, sqrt(orientation_cost), sqrt(orientation_cost), 0)
  static Eigen::MatrixXd weightMatrix(double position_cost, double orientation_cost);

  pinocchio::FrameIndex frame_id = 0;
  std::optional<pinocchio::FrameIndex> base_frame_id;
  Eigen::VectorXi v_indices;
  roboplan::CartesianConfiguration target_pose;
  Eigen::MatrixXd full_jacobian;
};

}  // namespace fairino_cartesian_servo
