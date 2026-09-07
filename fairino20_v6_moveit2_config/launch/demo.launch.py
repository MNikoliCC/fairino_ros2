from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    moveit_config = (
        MoveItConfigsBuilder(
            "fairino20_v6_robot",
            package_name="fairino20_v6_moveit2_config",
        )
        .robot_description(
            mappings={"use_mock_hardware": use_mock_hardware},
        )
        .to_moveit_configs()
    )

    demo = generate_demo_launch(moveit_config)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description=(
                    "Use mock_components/GenericSystem when true; use the "
                    "FAIRINO SDK hardware interface when false."
                ),
            ),
            *demo.entities,
        ]
    )
