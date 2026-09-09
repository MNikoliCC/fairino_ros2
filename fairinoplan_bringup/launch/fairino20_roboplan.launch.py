from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    auto_execute = LaunchConfiguration("auto_execute")

    pkg_share = FindPackageShare("fairinoplan_bringup")

    xacro_file = PathJoinSubstitution(
        [pkg_share, "config", "fairino20_v6_robot.urdf.xacro"]
    )
    initial_positions_file = PathJoinSubstitution(
        [pkg_share, "config", "initial_positions.yaml"]
    )
    controllers_file = PathJoinSubstitution(
        [pkg_share, "config", "ros2_controllers.yaml"]
    )

    rviz_config = PathJoinSubstitution(
    [
        pkg_share,
        "config",
        "fairino20.rviz",
    ]
)

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_file,
                " initial_positions_file:=",
                initial_positions_file,
                " use_mock_hardware:=",
                use_mock_hardware,
            ]
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_file,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    fairino_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fairino20_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    roboplan_node = Node(
        package="fairinoplan_bringup",
        executable="fairino_plan_execute.py",
        name="fairinoplan",
        output="screen",
        parameters=[
            {
                "joint_group": "fairino20_v6_group",
                "base_link": "base_link",
                "tip_link": "wrist3_link",
                "joint_state_topic": "/joint_states",
                "action_server_name":
                    "/fairino20_controller/follow_joint_trajectory",
                "auto_execute": auto_execute,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Use mock ros2_control hardware instead of FAIRINO hardware.",
            ),
            DeclareLaunchArgument(
                "auto_execute",
                default_value="false",
                description="Automatically execute each successfully planned goal.",
            ),
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            fairino_controller_spawner,
            roboplan_node,
            rviz_node,
        ]
    )
