from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_manipulator_description"))

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch" / "simulation.launch.py"))
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="mobile_manipulator_rviz",
                arguments=["-d", str(share / "rviz" / "mobile_manipulator.rviz")],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
