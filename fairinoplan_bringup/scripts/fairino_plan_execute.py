#!/usr/bin/env python3

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
from std_srvs.srv import Trigger

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

        def joint_state_cb(msg):
            self.last_joint_state = msg

        self._js_node = Node(node_name)
        self._js_sub = self._js_node.create_subscription(
            JointState,
            topic,
            joint_state_cb,
            VOLATILE_QOS,
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
            "wrist3_link",
        )

        self.declare_parameter(
            "auto_execute",
            False,
        )

        self.declare_parameter(
            "include_shortcutting",
            True,
        )

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

        # FAIRINO hardware uses ServoJ cmdT = 0.008 s
        self._traj_dt = 0.008

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

        self._planned_traj = None

        self.get_logger().info(
            "Ready."
        )

        self.get_logger().info(
            "Publish ~/joint_goal or ~/pose_goal to plan."
        )

        self.get_logger().info(
            "Call ~/execute to execute."
        )

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

    # =================================================================
    # Cartesian planning
    # =================================================================

    def _on_pose_goal(self, msg):

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

        self.get_logger().info(
            f"Start: "
            f"{np.round(start.positions, 3).tolist()}"
        )

        self.get_logger().info(
            f"Goal: "
            f"{np.round(goal.positions, 3).tolist()}"
        )

        # -------------------------------------------------------------
        # RRT
        # -------------------------------------------------------------

        path = self._rrt.plan(
            start,
            goal,
        )

        if path is None:
            raise RuntimeError(
                "RRT failed to find a path."
            )

        # -------------------------------------------------------------
        # Shortcut
        # -------------------------------------------------------------

        if self._include_shortcutting:

            self.get_logger().info(
                "Shortcutting..."
            )

            path = self._shortcutter.shortcut(
                path
            )

        # -------------------------------------------------------------
        # TOPP-RA
        # -------------------------------------------------------------

        self.get_logger().info(
            "Generating trajectory..."
        )

        self._planned_traj = (
            self._toppra.generate(
                path,
                TOPPRAOptions(
                    self._traj_dt,
                    mode=SplineFittingMode.Adaptive,
                    max_adaptive_iterations=5,
                ),
            )
        )

        self.get_logger().info(
            f"Plan ready: "
            f"{len(self._planned_traj.positions)} points."
        )

        if self._auto_execute:
            self._execute_latest()

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

        if self._planned_traj is None:
            raise RuntimeError(
                "No planned trajectory."
            )

        if not self._execute_client.wait_for_server(
            timeout_sec=2.0
        ):
            raise RuntimeError(
                "FollowJointTrajectory server "
                "is not available."
            )

        goal = FollowJointTrajectory.Goal()

        goal.trajectory = toJointTrajectory(
            self._planned_traj
        )

        self.get_logger().info(
            "Sending trajectory to "
            "fairino20_controller..."
        )

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

        else:

            self.get_logger().error(
                f"Execution failed. "
                f"Error code: {result.error_code}, "
                f"message: {result.error_string}"
            )

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