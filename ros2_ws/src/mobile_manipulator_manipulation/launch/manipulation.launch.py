"""B3 arm reconfiguration: move_group, the known planning scene, and ReconfigurePanel.

Run next to arm_control.launch.py (Unity) or mock_stack.launch.py (no Unity, with
use_sim_time:=false). `scenario` adds that scenario's unmapped obstacle boxes to the known
world; `initial_footprint_profile` must be the profile Nav2 was launched with.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    use_sim_time = value('use_sim_time').lower() == 'true'
    moveit = Path(get_package_share_directory('mobile_manipulator_moveit_config'))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit / 'launch/move_group.launch.py')),
            launch_arguments={'use_sim_time': str(use_sim_time).lower(),
                              'scene_source': value('scene_source')}.items()),
        Node(package='mobile_manipulator_manipulation', executable='planning_scene_loader', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'scenario': value('scenario'),
                          'include_scenario_obstacles': value('scene_source') == 'known'}]),
        Node(package='mobile_manipulator_manipulation', executable='reconfigure_panel_server', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'initial_footprint_profile': value('initial_footprint_profile'),
                          'scene_source': value('scene_source'),
                          'footprint_mode': value('footprint_mode')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='', description='scenarios.yaml entry ("" = none)'),
        DeclareLaunchArgument('scene_source', default_value='known', description='known or octomap'),
        DeclareLaunchArgument('initial_footprint_profile', default_value='home'),
        DeclareLaunchArgument('footprint_mode', default_value='profiles',
                              description='profiles (B3: the server switches named profiles) or dynamic '
                                          '(B4: dynamic_footprint_node owns the footprint, the server checks the hull)'),
        OpaqueFunction(function=launch_setup),
    ])
