"""Use alongside the existing description launch; never start another TF authority."""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description = Path(get_package_share_directory('mobile_manipulator_description'))
    control = Path(get_package_share_directory('mobile_manipulator_control'))
    robot = ET.parse(description / 'urdf/mobile_manipulator.urdf').getroot()
    system = ET.SubElement(robot, 'ros2_control', name='UnityArm', type='system')
    hardware = ET.SubElement(system, 'hardware')
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
    return LaunchDescription([
        Node(package='mobile_manipulator_control', executable='control_description.py',
             parameters=[{'robot_description': ET.tostring(robot, encoding='unicode')}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             remappings=[('robot_description', '/arm/robot_description')],
             parameters=[str(control / 'config/controllers.yaml'),
                         {'use_sim_time': True}]),
        Node(package='controller_manager', executable='spawner',
             arguments=['arm_joint_state_broadcaster', 'arm_controller', '--controller-manager-timeout', '30'],
             output='screen'),
    ])
