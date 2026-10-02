"""Use alongside the existing description launch; never start another TF authority.

use_mock_hardware:=true replaces the Unity arm with ros2_control's mock GenericSystem on
wall time, for MoveIt tests in the container without Unity (mock_stack.launch.py in
mobile_manipulator_moveit_config supplies TF and the full /joint_states).
"""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def without_sim_time(config_path):
    """Copy of the controller configuration with every use_sim_time set to false."""
    config = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    for node in config.values():
        node.get('ros__parameters', {})['use_sim_time'] = False
    output = tempfile.NamedTemporaryFile('w', suffix='_controllers.yaml', delete=False)
    yaml.safe_dump(config, output)
    output.close()
    return output.name


def launch_setup(context):
    mock = LaunchConfiguration('use_mock_hardware').perform(context).lower() == 'true'
    description = Path(get_package_share_directory('mobile_manipulator_description'))
    control = Path(get_package_share_directory('mobile_manipulator_control'))
    robot = ET.parse(description / 'urdf/mobile_manipulator.urdf').getroot()
    system = ET.SubElement(robot, 'ros2_control', name='UnityArm', type='system')
    hardware = ET.SubElement(system, 'hardware')
    if mock:
        ET.SubElement(hardware, 'plugin').text = 'mock_components/GenericSystem'
    else:
        ET.SubElement(hardware, 'plugin').text = 'mobile_manipulator_control/UnityArmSystem'
        for name, value in {'command_topic': '/arm/command', 'state_topic': '/arm/state', 'state_timeout': '0.5'}.items():
            ET.SubElement(hardware, 'param', name=name).text = value
    for joint in robot.findall('joint'):
        if joint.get('type') != 'revolute':
            continue
        controlled = ET.SubElement(system, 'joint', name=joint.get('name'))
        limits = joint.find('limit')
        for key in ('lower', 'upper', 'velocity'):
            ET.SubElement(controlled, 'param', name=key).text = limits.get(key)
        for interface in ('position', 'velocity'):
            command = ET.SubElement(controlled, 'command_interface', name=interface)
            low, high = ((limits.get('lower'), limits.get('upper')) if interface == 'position'
                         else (str(-float(limits.get('velocity'))), limits.get('velocity')))
            ET.SubElement(command, 'param', name='min').text = low
            ET.SubElement(command, 'param', name='max').text = high
            ET.SubElement(controlled, 'state_interface', name=interface)
    controllers = str(control / 'config/controllers.yaml')
    nodes = [
        Node(package='mobile_manipulator_control', executable='control_description',
             parameters=[{'robot_description': ET.tostring(robot, encoding='unicode')}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             remappings=[('robot_description', '/arm/robot_description')],
             parameters=[without_sim_time(controllers) if mock else controllers,
                         {'use_sim_time': not mock}]),
        Node(package='controller_manager', executable='spawner',
             arguments=['arm_joint_state_broadcaster', 'arm_controller', '--controller-manager-timeout', '30'],
             output='screen'),
    ]
    if not mock:
        # Re-activates the arm after a stale-feedback pause; a new epoch still needs a restart.
        nodes.append(Node(package='mobile_manipulator_control', executable='arm_recovery_supervisor',
                          output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='Mock GenericSystem on wall time instead of the Unity arm'),
        OpaqueFunction(function=launch_setup),
    ])
