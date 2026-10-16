#include "fairino_cartesian_servo/pointing_task.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pinocchio/spatial/explog.hpp>

namespace fairino_cartesian_servo {

PointingTask::PointingTask(const roboplan::Oink& oink, const roboplan::Scene& scene,
                           const roboplan::CartesianConfiguration& target, const roboplan::FrameTaskOptions& options)
    : roboplan::Task(options.priority, weightMatrix(options.position_cost, options.orientation_cost), options.task_gain,
                     options.lm_damping),
      target_pose(target) {
  const auto frame = scene.getFrameId(target.tip_frame);
  if (!frame) {
    throw std::runtime_error("Frame '" + target.tip_frame + "' not found: " + frame.error());
  }
  frame_id = frame.value();
  if (!target.base_frame.empty()) {
    const auto base = scene.getFrameId(target.base_frame);
    if (!base) {
      throw std::runtime_error("Base frame '" + target.base_frame + "' not found: " + base.error());
    }
    base_frame_id = base.value();
  }
  v_indices = oink.v_indices;
  initializeStorage(roboplan::kSpatialDimension, v_indices.size());
  full_jacobian = Eigen::MatrixXd::Zero(roboplan::kSpatialDimension, scene.getModel().nv);
}

tl::expected<void, std::string> PointingTask::computeError(const roboplan::SceneContext& context) {
  const auto& data = context.getData();
  const pinocchio::SE3& frame = data.oMf.at(frame_id);
  const pinocchio::SE3 target = base_frame_id.has_value()
                                    ? data.oMf.at(base_frame_id.value()) * pinocchio::SE3(target_pose.tform)
                                    : pinocchio::SE3(target_pose.tform);
  error_container.head<3>() = target.translation() - frame.translation();
  // Smallest rotation (world axis-angle) that turns the tool z axis onto the
  // target's z axis; it has no component about the tool z axis.
  const Eigen::Vector3d z = frame.rotation().col(2);
  const Eigen::Vector3d d = target.rotation().col(2);
  const Eigen::Vector3d cross = z.cross(d);
  const double sin = cross.norm();
  const double cos = std::clamp(z.dot(d), -1.0, 1.0);
  Eigen::Vector3d turn = Eigen::Vector3d::Zero();
  if (sin > 1e-12) {
    turn = cross / sin * std::atan2(sin, cos);
  } else if (cos < 0.0) {
    turn = frame.rotation().col(0) * M_PI;
  }
  // In the tool frame, so the third component is the (weight 0) roll.
  error_container.tail<3>() = frame.rotation().transpose() * turn;
  return {};
}

tl::expected<void, std::string> PointingTask::computeJacobian(const roboplan::SceneContext& context) {
  const Eigen::VectorXd& q = context.getJointPositions();
  full_jacobian.setZero();
  if (base_frame_id.has_value()) {
    context.computeRelativeFrameJacobian(q, frame_id, target_pose.base_frame,
                                         pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, full_jacobian);
  } else {
    context.computeFrameJacobian(q, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, full_jacobian);
  }
  // Same sign convention as FrameTask; angular rows rotated into the tool
  // frame to match the error.
  const Eigen::Matrix3d rotation_t = context.getData().oMf.at(frame_id).rotation().transpose();
  full_jacobian.bottomRows<3>() = rotation_t * full_jacobian.bottomRows<3>();
  jacobian_container.noalias() = -full_jacobian(Eigen::placeholders::all, v_indices);
  return {};
}

Eigen::MatrixXd PointingTask::weightMatrix(double position_cost, double orientation_cost) {
  Eigen::MatrixXd weight = Eigen::MatrixXd::Zero(roboplan::kSpatialDimension, roboplan::kSpatialDimension);
  weight.diagonal() << std::sqrt(position_cost), std::sqrt(position_cost), std::sqrt(position_cost),
      std::sqrt(orientation_cost), std::sqrt(orientation_cost), 0.0;
  return weight;
}

}  // namespace fairino_cartesian_servo
