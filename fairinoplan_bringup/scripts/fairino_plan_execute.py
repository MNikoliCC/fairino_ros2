#!/usr/bin/env python3

import json
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_path
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from roboplan.core import (
    CartesianConfiguration,
    JointConfiguration,
    PathShortcuttingOptions,
    PathShortcutter,
    Scene,
)
from roboplan.rrt import RRT, RRTOptions
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from roboplan.toppra import (
    PathParameterizerTOPPRA,
    SplineFittingMode,
    TOPPRAOptions,
)

from roboplan_ros.cpp import (
    buildConversionMap,
    fromJointState,
    poseToSE3,
    toJointTrajectory,
)

import threading

import pinocchio as pin

from fairino_trajectory_tools import (
    PathError,
    look_at_from_payload,
    PathIk,
    fit_recorded_timing,
    parameterize_path,
    path_from_payload,
    quat_wxyz_to_matrix,
    sample_path,
    smooth_recording,
    tracking_report,
)
from fairino_msgs.msg import RobotStatus

from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

def spin_executor(executor):
    """Helper function to spin an executor."""
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except Exception:
        if executor.context.ok():
            raise


VOLATILE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
)

class JointStateSubscriber:
    """
    Subscribes and manages a spinner to the specified joint states topic.

    JointStates are often published at the control loop rate, and a subscriber's
    callback functions can eat time in an executor's spinner. This class handles
    standing up the required infrastructure to subscribe to a JointState topic,
    spinning the subscriber, and providing access to the latest message on the
    topic.
    """

    def __init__(self, node_name="joint_state_listener", topic="/joint_states"):
        self.last_joint_state = None
        # While a trajectory executes: (time, msg) of every joint state and
        # the FAIRINO ServoJ counters, for the tracking report.
        self.recording = None
        self.servoj = None

        def joint_state_cb(msg):
            self.last_joint_state = msg
            if self.recording is not None:
                self.recording.append((time.monotonic(), msg))

        def robot_status_cb(msg):
            if self.servoj is not None:
                self.servoj.append((time.monotonic(), msg.servoj_cmd_num, msg.servoj_flow_skips,
                                    msg.servoj_point_limited, msg.servoj_cmd_period_s))

        self._js_node = Node(node_name)
        self._js_sub = self._js_node.create_subscription(
            JointState,
            topic,
            joint_state_cb,
            VOLATILE_QOS,
        )
        self._status_sub = self._js_node.create_subscription(
            RobotStatus, "/fairino_robot/status", robot_status_cb, 10
        )

        self._js_executor = SingleThreadedExecutor()
        self._js_executor.add_node(self._js_node)
        self._js_thread = threading.Thread(
            target=spin_executor, daemon=True, args=(self._js_executor,)
        )
        self._js_thread.start()

    def shutdown(self):
        self._js_node.destroy_node()
        self._js_executor.shutdown()
        self._js_thread.join()
        self._js_node = None

class FairinoPlanExecute(Node):

    def __init__(self):
        super().__init__("fairinoplan")

        # -------------------------------------------------------------
        # Parameters
        # -------------------------------------------------------------

        self.declare_parameter(
            "joint_state_topic",
            "/joint_states",
        )

        self.declare_parameter(
            "action_server_name",
            "/fairino20_controller/follow_joint_trajectory",
        )

        self.declare_parameter(
            "joint_group",
            "fairino20_v6_group",
        )

        self.declare_parameter(
            "base_link",
            "base_link",
        )

        self.declare_parameter(
            "tip_link",
            "flange",
        )

        self.declare_parameter(
            "auto_execute",
            False,
        )

        self.declare_parameter(
            "include_shortcutting",
            True,
        )

        # Sample period of recorded/drawn path trajectories (the trajectory
        # controller interpolates between points at 1 kHz).
        self.declare_parameter("path_trajectory_dt", 0.004)
        # Spacing of the Cartesian path samples used for IK and timing [m].
        self.declare_parameter("path_sample_step", 0.002)

        joint_state_topic = self.get_parameter(
            "joint_state_topic"
        ).value

        action_server_name = self.get_parameter(
            "action_server_name"
        ).value

        self._joint_group = self.get_parameter(
            "joint_group"
        ).value

        self._base_link = self.get_parameter(
            "base_link"
        ).value

        self._tip_link = self.get_parameter(
            "tip_link"
        ).value

        self._auto_execute = self.get_parameter(
            "auto_execute"
        ).value

        self._include_shortcutting = self.get_parameter(
            "include_shortcutting"
        ).value

        # -------------------------------------------------------------
        # RoboPlan Scene
        # -------------------------------------------------------------

        bringup_share = get_package_share_path(
            "fairinoplan_bringup"
        )

        description_share = get_package_share_path(
            "fairino_description"
        )

        urdf_path = (
            description_share
            / "urdf"
            / "fairino20_v6.urdf"
        )

        srdf_path = (
            bringup_share
            / "config"
            / "fairino20_v6_robot.srdf"
        )

        roboplan_config_path = (
            bringup_share
            / "config"
            / "roboplan.yaml"
        )

        urdf_xml = urdf_path.read_text()
        srdf_xml = srdf_path.read_text()

        package_paths = [
            description_share.as_posix(),
            description_share.parent.as_posix(),
        ]

        self._scene = Scene(
            name="fairino20_scene",
            urdf=urdf_xml,
            srdf=srdf_xml,
            package_paths=package_paths,
            yaml_config_path=roboplan_config_path.as_posix(),
        )

        joint_group_info = self._scene.getJointGroupInfo(
            self._joint_group
        )

        self._q_indices = joint_group_info.q_indices
        self._joint_names = list(
            joint_group_info.joint_names
        )

        self.get_logger().info(
            f"Planning group: {self._joint_group}"
        )

        self.get_logger().info(
            f"Joints: {self._joint_names}"
        )

        # Joint limits for recorded/drawn paths: the same RoboPlan limits as
        # TOPP-RA (roboplan.yaml).
        self._qd_max = np.asarray(self._scene.getVelocityLimitVectors(self._joint_group)[1], dtype=float)
        self._qdd_max = np.asarray(self._scene.getAccelerationLimitVectors(self._joint_group)[1], dtype=float)
        q_lower, q_upper = self._scene.getPositionLimitVectors(self._joint_group)
        self._q_lower = np.asarray(q_lower, dtype=float)
        self._q_upper = np.asarray(q_upper, dtype=float)
        self._path_dt = float(self.get_parameter("path_trajectory_dt").value)
        self._path_step = float(self.get_parameter("path_sample_step").value)
        self._path_ik = PathIk(urdf_xml, self._joint_names, self._tip_link)

        # -------------------------------------------------------------
        # Joint states
        #
        # This wrapper creates its own ROS node + executor + thread.
        # -------------------------------------------------------------

        self._js_subscriber = JointStateSubscriber(
            topic=joint_state_topic
        )

        while (
            rclpy.ok()
            and self._js_subscriber.last_joint_state is None
        ):
            self.get_logger().info(
                f"Waiting for joint positions on "
                f"{joint_state_topic}..."
            )
            time.sleep(1.0)

        if self._js_subscriber.last_joint_state is None:
            raise RuntimeError(
                "No JointState received."
            )

        # Build ROS JointState -> RoboPlan configuration mapping
        self._conversion_map = buildConversionMap(
            self._scene,
            self._js_subscriber.last_joint_state,
        )

        # Sync RoboPlan scene with real robot
        self._latest_joint_positions = (
            self._get_hardware_positions()
        )

        self._scene.setJointPositions(
            self._latest_joint_positions
        )

        self.get_logger().info(
            "Joint states received. RoboPlan ready."
        )

        # -------------------------------------------------------------
        # IK
        # -------------------------------------------------------------

        ik_options = SimpleIkOptions()

        ik_options.group_name = self._joint_group
        ik_options.max_iters = 100
        ik_options.step_size = 0.25
        ik_options.max_time = 0.10
        ik_options.check_collisions = True
        ik_options.fast_return = False

        self._ik = SimpleIk(
            self._scene,
            ik_options,
        )

        # -------------------------------------------------------------
        # RRT
        # -------------------------------------------------------------

        self._rrt_options = RRTOptions()

        self._rrt_options.group_name = (
            self._joint_group
        )

        self._rrt_options.max_connection_distance = 3.0

        self._rrt_options.collision_check_step_size = 0.05

        self._rrt_options.max_planning_time = 5.0

        self._rrt_options.rrt_connect = True

        self._rrt_options.max_nodes = 1000

        self._rrt_options.goal_biasing_probability = 0.05

        self._rrt_options.collision_check_use_bisection = True

        self._rrt = RRT(
            self._scene,
            self._rrt_options,
        )

        # -------------------------------------------------------------
        # Path shortcutting
        # -------------------------------------------------------------

        shortcut_options = PathShortcuttingOptions(
            group_name=self._joint_group,
            max_step_size=(
                self._rrt_options.collision_check_step_size
            ),
            max_iters=250,
        )

        self._shortcutter = PathShortcutter(
            self._scene,
            shortcut_options,
        )

        # -------------------------------------------------------------
        # TOPP-RA
        # -------------------------------------------------------------

        self._toppra = PathParameterizerTOPPRA(
            self._scene,
            self._joint_group,
        )

        # Match the 1 kHz ros2_control / FAIRINO ServoJ command period.
        self._traj_dt = 0.001

        # -------------------------------------------------------------
        # Goal inputs
        # -------------------------------------------------------------

        self._joint_goal_sub = self.create_subscription(
            JointState,
            "~/joint_goal",
            self._on_joint_goal,
            10,
        )

        self._pose_goal_sub = self.create_subscription(
            PoseStamped,
            "~/pose_goal",
            self._on_pose_goal,
            10,
        )

        # Recorded trajectories and drawn Cartesian paths (JSON, see
        # fairino_trajectory_tools and docs/trajectory_recording_and_paths.md).
        self._path_goal_sub = self.create_subscription(
            String,
            "~/path_goal",
            self._on_path_goal,
            10,
        )
        # Check while drawing: reachability (failing segment) and the fastest
        # time of every segment; does not create or change a plan.
        self._path_check_sub = self.create_subscription(
            String,
            "~/path_check",
            self._on_path_check,
            10,
        )
        self._path_check_pub = self.create_publisher(String, "~/path_check_result", 10)

        # -------------------------------------------------------------
        # Execution
        # -------------------------------------------------------------

        self._execute_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_server_name,
        )

        self._execute_service = self.create_service(
            Trigger,
            "~/execute",
            self._on_execute_service,
        )

        # The plan that Execute sends; cleared by every new request so a stale
        # plan can never be executed. kind = its source (joint_goal,
        # joint_path, cartesian_path) for the execution status.
        self._planned_msg = None
        self._planned_kind = ""

        # Latched plan/execution status (JSON) and the planned trajectory, so
        # a UI (Viser) can show the result of Plan and Execute.
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(String, "~/status", latched)
        self._path_pub = self.create_publisher(
            JointTrajectory, "~/planned_path", latched
        )
        self._publish_status("ready", "waiting for a goal")

        self.get_logger().info(
            "Ready."
        )

        self.get_logger().info(
            "Publish ~/joint_goal or ~/pose_goal to plan."
        )

        self.get_logger().info(
            "Call ~/execute to execute."
        )

    def _publish_status(self, state, message, **extra):
        payload = {"state": state, "message": message, "stamp": time.time()}
        payload.update(extra)
        self._status_pub.publish(String(data=json.dumps(payload)))

    # =================================================================
    # Hardware state
    # =================================================================

    def _get_hardware_positions(self):
        """
        Get latest real FAIRINO joint positions and convert them
        into RoboPlan's full robot configuration.
        """

        if self._js_subscriber.last_joint_state is None:
            raise RuntimeError(
                "No joint states available."
            )

        joint_config = fromJointState(
            self._js_subscriber.last_joint_state,
            self._scene,
            self._conversion_map,
        )

        return self._scene.clampToValidConfiguration(
            joint_config.positions
        )

    # =================================================================
    # Joint-space planning
    # =================================================================

    def _on_joint_goal(self, msg):

        self._planned_msg = None
        try:
            goal_by_name = dict(
                zip(
                    msg.name,
                    msg.position,
                )
            )

            missing = [
                joint
                for joint in self._joint_names
                if joint not in goal_by_name
            ]

            if missing:
                raise ValueError(
                    f"Missing joints: {missing}"
                )

            goal_positions = [
                float(goal_by_name[joint])
                for joint in self._joint_names
            ]

            self._plan_to_joint_positions(
                goal_positions
            )

        except Exception as e:
            self.get_logger().error(
                f"Joint goal failed: {e}"
            )
            self._publish_status("failed", f"joint goal failed: {e}")

    # =================================================================
    # Cartesian planning
    # =================================================================

    def _on_pose_goal(self, msg):

        self._planned_msg = None
        try:
            if msg.header.frame_id not in (
                "",
                self._base_link,
            ):
                raise ValueError(
                    f"Pose must be expressed in "
                    f"'{self._base_link}'."
                )

            # Get latest REAL robot state
            self._latest_joint_positions = (
                self._get_hardware_positions()
            )

            self._scene.setJointPositions(
                self._latest_joint_positions
            )

            # Seed IK from current hardware position
            seed = JointConfiguration()

            seed.positions = (
                self._latest_joint_positions[
                    self._q_indices
                ]
            )

            goal = CartesianConfiguration()

            goal.base_frame = self._base_link
            goal.tip_frame = self._tip_link

            goal.tform = poseToSE3(
                msg.pose
            )

            solution = JointConfiguration()

            success = self._ik.solveIk(
                goal,
                seed,
                solution,
            )

            if not success:
                raise RuntimeError(
                    "IK failed."
                )

            self.get_logger().info(
                f"IK solution: "
                f"{np.round(solution.positions, 3).tolist()}"
            )

            self._plan_to_joint_positions(
                solution.positions
            )

        except Exception as e:
            self.get_logger().error(
                f"Pose goal failed: {e}"
            )
            self._publish_status("failed", f"pose goal failed: {e}")

    # =================================================================
    # Planning pipeline
    # =================================================================

    def _plan_to_joint_positions(
        self,
        goal_positions,
    ):
        """
        Hardware state
             ↓
        RRT
             ↓
        Path shortcutting
             ↓
        TOPP-RA
             ↓
        JointTrajectory
        """

        # Always start from latest REAL robot state
        self._latest_joint_positions = (
            self._get_hardware_positions()
        )

        self._scene.setJointPositions(
            self._latest_joint_positions
        )

        start = JointConfiguration()

        start.positions = (
            self._latest_joint_positions[
                self._q_indices
            ]
        )

        goal = JointConfiguration()

        goal.positions = np.asarray(
            goal_positions,
            dtype=float,
        )

        if len(goal.positions) != len(
            self._q_indices
        ):
            raise ValueError(
                f"Expected {len(self._q_indices)} joints, "
                f"got {len(goal.positions)}."
            )

        self.get_logger().info(
            "Planning..."
        )
        plan_started = time.monotonic()
        self._publish_status("planning", "RRT + shortcutting + TOPP-RA", kind="joint_goal")

        self.get_logger().info(
            f"Start: "
            f"{np.round(start.positions, 3).tolist()}"
        )

        self.get_logger().info(
            f"Goal: "
            f"{np.round(goal.positions, 3).tolist()}"
        )

        times, positions, velocities = self._rrt_arrays(start.positions, goal.positions)
        self._publish_plan(times, positions, velocities, plan_started, kind="joint_goal")

        if self._auto_execute:
            self._execute_latest()

    def _rrt_arrays(self, start_positions, goal_positions):
        """RRT + shortcutting + TOPP-RA between two configurations."""
        start = JointConfiguration()
        start.positions = np.asarray(start_positions, dtype=float)
        goal = JointConfiguration()
        goal.positions = np.asarray(goal_positions, dtype=float)
        path = self._rrt.plan(start, goal)
        if path is None:
            raise RuntimeError("RRT failed to find a path.")
        if self._include_shortcutting:
            path = self._shortcutter.shortcut(path)
        trajectory = self._toppra.generate(
            path,
            TOPPRAOptions(
                self._traj_dt,
                mode=SplineFittingMode.Adaptive,
                max_adaptive_iterations=5,
            ),
        )
        return (
            np.asarray(trajectory.times, dtype=float),
            np.asarray(trajectory.positions, dtype=float),
            np.asarray(trajectory.velocities, dtype=float),
        )

    def _make_trajectory_msg(self, times, positions, velocities):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(self._joint_names)
        for t, q, v in zip(times, positions, velocities):
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in q]
            point.velocities = [float(x) for x in v]
            nanoseconds = int(round(float(t) * 1e9))
            point.time_from_start = Duration(sec=nanoseconds // 1000000000, nanosec=nanoseconds % 1000000000)
            msg.points.append(point)
        return msg

    def _publish_plan(self, times, positions, velocities, plan_started, **extra):
        msg = self._make_trajectory_msg(times, positions, velocities)
        self._planned_msg = msg
        self._planned_kind = extra.get("kind", "")
        self._path_pub.publish(msg)
        duration = float(times[-1]) if len(times) else 0.0
        self.get_logger().info(f"Plan ready: {len(times)} points, {duration:.2f} s.")
        self._publish_status(
            "planned",
            "plan ready; press Execute in plan mode",
            points=len(times),
            trajectory_duration_s=duration,
            planning_time_s=time.monotonic() - plan_started,
            **extra,
        )

    def _current_group_positions(self):
        self._latest_joint_positions = self._get_hardware_positions()
        self._scene.setJointPositions(self._latest_joint_positions)
        return np.asarray(self._latest_joint_positions[self._q_indices], dtype=float)

    # =================================================================
    # Recorded trajectories and drawn paths
    # =================================================================

    def _on_path_goal(self, msg):
        started = time.monotonic()
        kind = "path"
        self._planned_msg = None
        try:
            request = json.loads(msg.data)
            kind = str(request.get("type", "?"))
            name = str(request.get("name", ""))
            if kind == "clear":
                empty = JointTrajectory()
                empty.header.stamp = self.get_clock().now().to_msg()
                empty.joint_names = list(self._joint_names)
                self._path_pub.publish(empty)
                self._publish_status("ready", "plan cleared", kind="clear")
                return
            scale = float(np.clip(float(request.get("speed_scale", 1.0)), 0.05, 1.0))
            self._publish_status("planning", f"{kind} {name}: sampling, IK and timing", kind=kind)
            q_now = self._current_group_positions()

            if kind == "joint_path":
                path_q, timed = self._joint_path(request, scale)
            elif kind == "cartesian_path":
                path_q, timed = self._cartesian_path(request, q_now, scale)
            else:
                raise ValueError(f"unknown path type {kind!r}")
            if kind == "joint_path":
                self._check_path(path_q)

            info = {k: round(float(v), 4) for k, v in timed.info.items()}
            if timed.segment_fastest_s:
                info["segment_fastest_s"] = [round(v, 3) for v in timed.segment_fastest_s]
                info["segment_planned_s"] = [round(v, 3) for v in timed.segment_planned_s]

            times, positions, velocities = timed.times, timed.positions, timed.velocities
            approach_s = 0.0
            if np.max(np.abs(path_q[0] - q_now)) > 2e-3:
                t_a, p_a, v_a = self._rrt_arrays(q_now, timed.positions[0])
                approach_s = float(t_a[-1])
                times = np.concatenate([t_a, timed.times[1:] + t_a[-1]])
                positions = np.vstack([p_a, timed.positions[1:]])
                velocities = np.vstack([v_a, timed.velocities[1:]])
            self._publish_plan(
                times,
                positions,
                velocities,
                started,
                kind=kind,
                name=name,
                approach_duration_s=approach_s,
                path_duration_s=float(timed.times[-1]),
                speed_scale=scale,
                **info,
            )
        except PathError as e:
            self.get_logger().error(f"{kind} failed: {e}")
            self._publish_status(
                "failed", f"{kind}: {e}", kind=kind, failed_segments=list(e.segments), fraction=e.fraction
            )
        except Exception as e:
            self.get_logger().error(f"{kind} failed: {e}")
            self._publish_status("failed", f"{kind} failed: {e}", kind=kind)

    def _joint_path(self, request, scale):
        names = list(request.get("joint_names", self._joint_names))
        order = [names.index(n) for n in self._joint_names]
        positions = np.asarray(request["positions"], dtype=float)[:, order]
        times = np.asarray(request.get("times", []), dtype=float)
        if len(positions) < 2:
            raise PathError("the recording has fewer than two samples")
        if request.get("timing", "original") == "original":
            # Recorded timing, played back at speed_scale; slowed down
            # automatically if it would exceed the joint limits.
            times, positions = smooth_recording(times, positions)
            timed = fit_recorded_timing(times / scale, positions, self._qd_max, self._qdd_max, self._path_dt)
            timed.info["timing_original"] = 1.0
        else:
            # Same geometry, fastest timing within the (scaled) joint limits.
            _, positions = smooth_recording(times, positions)
            keep = np.concatenate([[True], np.max(np.abs(np.diff(positions, axis=0)), axis=1) > 1e-5])
            positions = positions[keep]
            if len(positions) < 2:
                raise PathError("the recording does not move")
            s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))])
            stops = np.zeros(len(s), dtype=bool)
            stops[0] = stops[-1] = True
            timed = parameterize_path(
                positions, s, np.full(len(s), np.inf), stops, self._qd_max * scale, self._qdd_max * scale, self._path_dt
            )
            timed.info["timing_original"] = 0.0
        return positions, timed

    def _solve_cartesian(self, request, q_now):
        """Samples the path, follows it with IK from the current configuration
        and checks joint limits and self-collision (errors name the segment)."""
        waypoints, segments = path_from_payload(request)
        sampled = sample_path(waypoints, self._path_step, look_at=look_at_from_payload(request))
        start_pose = pin.SE3(quat_wxyz_to_matrix(sampled.wxyz[0]), sampled.positions[0])
        # Nearest solution by damped least squares from the current
        # configuration; RoboPlan's IK only if that does not converge.
        q_start, error = self._path_ik.solve(start_pose, q_now)
        if error > 1e-4:
            goal = CartesianConfiguration()
            goal.base_frame = self._base_link
            goal.tip_frame = self._tip_link
            goal.tform = start_pose.homogeneous
            seed = JointConfiguration()
            seed.positions = q_now
            solution = JointConfiguration()
            if not self._ik.solveIk(goal, seed, solution):
                raise PathError("the first waypoint is not reachable", segments=[0])
            q_start = np.asarray(solution.positions, dtype=float)
        path_q = self._path_ik.follow(sampled, q_start)
        self._check_path(path_q, sampled.segment)
        return sampled, path_q, segments

    def _cartesian_path(self, request, q_now, scale):
        sampled, path_q, segments = self._solve_cartesian(request, q_now)
        timed = parameterize_path(
            path_q,
            sampled.length,
            sampled.speed * scale,
            sampled.stop,
            self._qd_max,
            self._qdd_max,
            self._path_dt,
            sampled.segment,
            [s.duration for s in segments],
        )
        timed.info["samples"] = float(len(sampled.positions))
        return path_q, timed

    def _on_path_check(self, msg):
        result = {}
        try:
            request = json.loads(msg.data)
            result["request_id"] = request.get("request_id")
            q_now = self._current_group_positions()
            sampled, path_q, _ = self._solve_cartesian(request, q_now)
            timed = parameterize_path(
                path_q,
                sampled.length,
                sampled.speed,
                sampled.stop,
                self._qd_max,
                self._qdd_max,
                self._path_dt,
                sampled.segment,
            )
            result.update(ok=True, message="reachable", segment_fastest_s=[round(v, 3) for v in timed.segment_fastest_s])
        except PathError as e:
            result.update(ok=False, message=str(e), failed_segments=list(e.segments))
        except Exception as e:
            result.update(ok=False, message=f"check failed: {e}", failed_segments=[])
        self._path_check_pub.publish(String(data=json.dumps(result)))

    def _check_path(self, path_q, owner=None):
        """Joint limits and self-collision; owner = segment index per sample."""

        def where(k):
            return [int(owner[k])] if owner is not None else []

        margin = 1e-4
        bad = np.where(np.any((path_q < self._q_lower - margin) | (path_q > self._q_upper + margin), axis=1))[0]
        if len(bad):
            k = int(bad[0])
            joint = int(np.argmax((path_q[k] < self._q_lower - margin) | (path_q[k] > self._q_upper + margin)))
            raise PathError(
                f"{self._joint_names[joint]} outside its limits at {100.0 * k / (len(path_q) - 1):.0f} % of the path",
                segments=where(k),
                fraction=k / (len(path_q) - 1),
            )
        for k in range(0, len(path_q), 5):
            full = self._scene.toFullJointPositions(self._joint_group, path_q[k])
            if self._scene.hasCollisions(full):
                raise PathError(
                    f"self-collision at {100.0 * k / (len(path_q) - 1):.0f} % of the path",
                    segments=where(k),
                    fraction=k / (len(path_q) - 1),
                )

    # =================================================================
    # Execution
    # =================================================================

    def _on_execute_service(
        self,
        request,
        response,
    ):

        try:
            self._execute_latest()

            response.success = True

            response.message = (
                "Trajectory sent to FAIRINO controller."
            )

        except Exception as e:

            response.success = False
            response.message = str(e)

        return response

    def _execute_latest(self):

        if self._planned_msg is None:
            raise RuntimeError(
                "No planned trajectory."
            )

        # A plan starts where the robot was when it was planned; if the robot
        # moved since (servoing, another program), executing it would make
        # the trajectory controller jump to the first point.
        current = self._current_group_positions()
        first = self._planned_msg.points[0].positions if self._planned_msg.points else None
        if first is not None and np.max(np.abs(current - np.asarray(first))) > 0.003:
            raise RuntimeError("the robot moved since this plan was made; plan again")

        if not self._execute_client.wait_for_server(
            timeout_sec=2.0
        ):
            raise RuntimeError(
                "FollowJointTrajectory server "
                "is not available."
            )

        goal = FollowJointTrajectory.Goal()

        # The published plan carries the planning time (the UI uses it to
        # ignore old latched plans); the controller must start the trajectory
        # on receipt, so the goal gets a zero stamp. A non-zero stamp in the
        # past made the controller skip the beginning or reject the goal.
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(self._planned_msg.joint_names)
        goal.trajectory.points = list(self._planned_msg.points)

        self.get_logger().info(
            "Sending trajectory to "
            "fairino20_controller..."
        )
        self._publish_status("executing", "trajectory sent to fairino20_controller", kind=self._planned_kind)
        self._executing_msg = self._planned_msg
        self._js_subscriber.recording = []
        self._js_subscriber.servoj = []

        future = (
            self._execute_client.send_goal_async(
                goal,
                feedback_callback=(
                    self._execute_feedback
                ),
            )
        )

        future.add_done_callback(
            self._execute_goal_response
        )

    def _execute_goal_response(
        self,
        future,
    ):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                "Trajectory rejected."
            )
            self._publish_status(
                "failed",
                "trajectory rejected (is fairino20_controller active / plan mode?)",
                kind=self._planned_kind,
            )

            return

        self.get_logger().info(
            "Trajectory accepted."
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self._execute_result
        )

    def _execute_feedback(
        self,
        feedback_msg,
    ):
        pass

    def _execute_result(
        self,
        future,
    ):

        result = future.result().result

        if (
            result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        ):

            self.get_logger().info(
                "Trajectory execution complete."
            )
            # The controller is done when its trajectory time is over; the
            # robot arrives a little later (feedback and ServoJ latency).
            # Report success once it is there, with the tracking report.
            kind = self._planned_kind
            threading.Thread(target=self._finish_execution, args=(kind,), daemon=True).start()

        else:

            self.get_logger().error(
                f"Execution failed. "
                f"Error code: {result.error_code}, "
                f"message: {result.error_string}"
            )
            tracking = self._tracking_report()
            self._publish_status(
                "failed",
                f"execution failed: {result.error_code} {result.error_string}",
                kind=self._planned_kind,
                tracking=tracking,
            )

    def _finish_execution(self, kind):
        plan = getattr(self, "_executing_msg", None)
        done = time.monotonic()
        arrival = None
        if plan is not None and plan.points:
            final = np.asarray(plan.points[-1].positions, dtype=float)
            order = [list(plan.joint_names).index(n) for n in self._joint_names]
            while time.monotonic() - done < 2.0:
                # Raw joint states (this thread must not touch the RoboPlan scene).
                msg = self._js_subscriber.last_joint_state
                positions = dict(zip(msg.name, msg.position))
                q = np.array([positions.get(n, np.nan) for n in self._joint_names])
                if np.nanmax(np.abs(q - final[order])) < 0.001:
                    arrival = time.monotonic() - done
                    break
                time.sleep(0.004)
        time.sleep(0.1)
        tracking = self._tracking_report()
        if tracking is not None:
            tracking["arrival_after_controller_s"] = None if arrival is None else round(arrival, 3)
        self._publish_status("succeeded", "trajectory execution complete", kind=kind, tracking=tracking)

    def _tracking_report(self):
        """Deviation of the executed motion from the plan (TCP, spatial) and
        the ServoJ rate the controller kept up with; None without data."""
        recording, self._js_subscriber.recording = self._js_subscriber.recording, None
        servoj, self._js_subscriber.servoj = self._js_subscriber.servoj, None
        plan = getattr(self, "_executing_msg", None)
        if not recording or plan is None or len(recording) < 2 or not plan.points:
            return None
        try:
            names = list(plan.joint_names)
            plan_t = np.array([p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec for p in plan.points])
            plan_q = np.array([p.positions for p in plan.points])
            order = [names.index(n) for n in self._joint_names]
            plan_q = plan_q[:, order]
            act_t = np.array([t for t, _ in recording])
            act_q = np.array([[dict(zip(m.name, m.position))[n] for n in self._joint_names] for _, m in recording])
            report = tracking_report(plan_t, plan_q, act_t, act_q, self._path_ik.fk, servoj or None)
        except Exception as exc:  # noqa: BLE001 - a report must never break execution
            self.get_logger().warning(f"tracking report failed: {exc}")
            return None
        self.get_logger().info(f"Tracking: {report}")
        return report

    # =================================================================
    # Shutdown
    # =================================================================

    def destroy_node(self):

        self._js_subscriber.shutdown()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = FairinoPlanExecute()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
