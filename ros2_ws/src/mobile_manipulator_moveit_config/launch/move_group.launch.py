"""MoveIt move_group for the arm (B3 baseline).

The URDF is the mobile_manipulator_description file (single geometry source); the robot
model is published by robot_state_publisher elsewhere, so only the semantic description
is published here.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

CAPABILITIES = 'move_group/ClearOctomapService'


def moveit_config(scene_source='known'):
    description = Path(get_package_share_directory('mobile_manipulator_description'))
    builder = (MoveItConfigsBuilder('mobile_manipulator', package_name='mobile_manipulator_moveit_config')
               .robot_description(file_path=str(description / 'urdf/mobile_manipulator.urdf'))
               .robot_description_semantic(file_path='config/mobile_manipulator.srdf')
               .robot_description_kinematics(file_path='config/kinematics.yaml')
               .joint_limits(file_path='config/joint_limits.yaml')
               .trajectory_execution(file_path='config/moveit_controllers.yaml')
               .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
               .planning_scene_monitor(publish_robot_description=False,
                                       publish_robot_description_semantic=True))
    # Known geometry has no 3D sensor (there is deliberately no config/sensors_3d.yaml);
    # octomap adds the filtered Livox cloud.
    if scene_source == 'octomap':
        builder = builder.sensors_3d(file_path='config/sensors_3d_octomap.yaml')
    return builder.to_moveit_configs()


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    scene_source = LaunchConfiguration('scene_source').perform(context)
    if scene_source not in ('known', 'octomap'):
        raise RuntimeError(f"scene_source must be known or octomap, not '{scene_source}'")
    config = moveit_config(scene_source)
    return [Node(package='moveit_ros_move_group', executable='move_group', output='screen',
                 parameters=[config.to_dict(),
                             {'use_sim_time': use_sim_time, 'capabilities': CAPABILITIES}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scene_source', default_value='known',
                              description='known (geometry only) or octomap (adds the Livox Octomap)'),
        OpaqueFunction(function=launch_setup),
    ])
