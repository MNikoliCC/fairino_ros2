"""FAIRINO FR20 Viser Cartesian servo stack (no RViz).

Starts robot_state_publisher, ros2_control_node (FAIRINO hardware plugin or
mock), joint_state_broadcaster, the trajectory controller (active), the servo
JointGroupPositionController (inactive), the RoboPlan plan/execute node, the
C++ OInK Cartesian servo and the Viser UI.

The FAIRINO status/diagnostics/recovery component is the `fairino_robot` node
created inside the hardware plugin (single owner of the SDK connection):
/fairino_robot/status, /fairino_robot/events, /fairino_robot/recover,
/fairino_robot/stop, /fairino_robot/set_global_speed,
/fairino_robot/set_safety_speed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controller_ip = LaunchConfiguration("controller_ip")
    allow_safety_config_writes = LaunchConfiguration("allow_safety_config_writes")
    auto_recovery = LaunchConfiguration("auto_recovery")
    solver_frequency = LaunchConfiguration("solver_frequency")
    self_collision_pairs = LaunchConfiguration("self_collision_pairs")
    viser_port = LaunchConfiguration("viser_port")
    start_planner = LaunchConfiguration("start_planner")
    controller_manager_prefix = LaunchConfiguration("controller_manager_prefix")

    pkg_share = FindPackageShare("fairinoplan_bringup")
    servo_params = PathJoinSubstitution(
        [FindPackageShare("fairino_cartesian_servo"), "config", "fairino_cartesian_servo.yaml"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution([pkg_share, "config", "fairino20_v6_robot.urdf.xacro"]),
                    " initial_positions_file:=",
                    PathJoinSubstitution([pkg_share, "config", "initial_positions.yaml"]),
                    " use_mock_hardware:=",
                    use_mock_hardware,
                    " controller_ip:=",
                    controller_ip,
                    " allow_safety_config_writes:=",
                    allow_safety_config_writes,
                ]
            ),
            value_type=str,
        )
    }
    # Hardware status is only available from the FAIRINO plugin, not from mock hardware.
    robot_status_required = ParameterValue(
        PythonExpression(["'", use_mock_hardware, "'.lower() not in ('true', '1')"]), value_type=bool
    )

    def spawner(name, *extra):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, *extra, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    joint_state_broadcaster = spawner("joint_state_broadcaster")
    trajectory_controller = spawner("fairino20_controller")
    # Both position controllers claim the same six interfaces: the servo
    # controller is loaded inactive and only activated by a strict switch.
    servo_controller = spawner("fairino_servo_controller", "--inactive")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="true",
                description="true: mock_components/GenericSystem; false: FAIRINO SDK hardware (SimMachine or robot).",
            ),
            DeclareLaunchArgument("controller_ip", default_value="192.168.58.2"),
            DeclareLaunchArgument(
                "allow_safety_config_writes",
                default_value="false",
                description="SAFETY-RELATED: allow SetVelReducePara through /fairino_robot/set_safety_speed.",
            ),
            DeclareLaunchArgument(
                "auto_recovery",
                default_value="true",
                description="Automatically recover whitelisted faults (always ends in HOLD).",
            ),
            DeclareLaunchArgument(
                "solver_frequency",
                default_value="1000.0",
                description="OInK rate [Hz]; joint commands stay at 1 kHz (see docs/servo_benchmark.md).",
            ),
            DeclareLaunchArgument(
                "self_collision_pairs",
                default_value="0",
                description="RoboPlan self-collision barrier pairs; 0 disables it (phase 2).",
            ),
            DeclareLaunchArgument("viser_port", default_value="8000"),
            DeclareLaunchArgument("start_planner", default_value="true"),
            DeclareLaunchArgument(
                "controller_manager_prefix",
                default_value="chrt -f 40",
                description=(
                    "Process prefix for ros2_control_node. SCHED_FIFO 40 keeps the executor and DDS threads "
                    "that deliver /fairino_servo_controller/commands ahead of normal load (the controller "
                    "manager loop itself runs at FIFO 50). Use '' when real-time scheduling is not permitted."
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                prefix=controller_manager_prefix,
                output="screen",
                parameters=[
                    robot_description,
                    PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"]),
                ],
            ),
            joint_state_broadcaster,
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_broadcaster, on_exit=[trajectory_controller])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=trajectory_controller, on_exit=[servo_controller])
            ),
            Node(
                package="fairinoplan_bringup",
                executable="fairino_plan_execute.py",
                name="fairinoplan",
                output="screen",
                condition=IfCondition(start_planner),
                parameters=[
                    {
                        "joint_group": "fairino20_v6_group",
                        "base_link": "base_link",
                        "tip_link": "wrist3_link",
                        "joint_state_topic": "/joint_states",
                        "action_server_name": "/fairino20_controller/follow_joint_trajectory",
                        "auto_execute": False,
                    }
                ],
            ),
            Node(
                package="fairino_cartesian_servo",
                executable="fairino_cartesian_servo",
                output="screen",
                parameters=[
                    servo_params,
                    {
                        "solver_frequency": ParameterValue(solver_frequency, value_type=float),
                        "self_collision_pairs": ParameterValue(self_collision_pairs, value_type=int),
                        "auto_recovery": ParameterValue(auto_recovery, value_type=bool),
                        "robot_status_required": robot_status_required,
                    },
                ],
            ),
            Node(
                package="fairinoplan_bringup",
                executable="fairino_viser_ui.py",
                output="screen",
                parameters=[
                    {
                        "host": "0.0.0.0",
                        "port": ParameterValue(viser_port, value_type=int),
                        "display_frequency": 30.0,
                        "heartbeat_frequency": 30.0,
                        "robot_status_required": robot_status_required,
                    }
                ],
            ),
        ]
    )
