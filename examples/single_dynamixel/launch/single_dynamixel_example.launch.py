from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare custom arguments
    robot_description_path = PathJoinSubstitution(
        [
            FindPackageShare("layered_hardware_dynamixel"),
            "examples/single_dynamixel/description",
            "robot_description.urdf.xacro",
        ]
    )

    # Include base launch description with custom arguments
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("layered_hardware"),
                    "examples/single_actuator/launch",
                    "single_actuator_example.launch.py"
                ])
            ]),
            launch_arguments={
                "robot_description_path": robot_description_path,
                "gui": "false"
            }.items()
        )
    ])