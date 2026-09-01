from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("mobile_manipulator_navigation"))
    parameters = str(share / "config" / "nav2_global_planning.yaml")
    map_yaml = str(share / "maps" / "construction_site.yaml")

    return LaunchDescription(
        [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[parameters, {"yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[parameters],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_global_planning",
                output="screen",
                parameters=[parameters],
            ),
        ]
    )
