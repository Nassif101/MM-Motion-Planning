#!/usr/bin/env python3
"""Bounded commissioning goals through the standard FollowJointTrajectory action."""
import argparse
import json
import time
import math
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist

NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint', choices=NAMES)
    parser.add_argument('--delta', type=float, default=0.05)
    parser.add_argument('--positions', type=float, nargs=6)
    parser.add_argument('--duration', type=float, default=4.0)
    parser.add_argument('--cancel-after', type=float)
    parser.add_argument('--hold-seconds', type=float, default=0.0)
    parser.add_argument('--disturbance', choices=['none','compact','extended','gate'], default='none',
                        help='Bounded simulation-only base sequence after reaching the arm goal; requires a cleared test area')
    parser.add_argument('--output', type=Path, default=Path('/tmp/arm-action-result.json'))
    args = parser.parse_args()
    if not 1 <= args.duration <= 30 or abs(args.delta) > 1 or not 0 <= args.hold_seconds <= 60:
        parser.error('Use duration 1..30 seconds and delta <= 1 rad')
    rclpy.init()
    node = Node('arm_commissioning', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    actual = {}
    def feedback(msg):
        actual.update(zip(msg.name, msg.position))
        actual['_received'] = time.monotonic()
    subscription = node.create_subscription(JointState, '/arm/state', feedback, 1)
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    deadline = time.monotonic() + 10
    while not all(n in actual for n in NAMES) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not all(n in actual for n in NAMES) or time.monotonic()-actual['_received'] > 0.5:
        raise RuntimeError('No fresh physical arm state')
    if not client.wait_for_server(timeout_sec=5):
        raise RuntimeError('Arm action server unavailable')
    start = [actual[n] for n in NAMES]
    target = args.positions or [q + (args.delta if args.joint in (None,n) else 0) for q,n in zip(start,NAMES)]
    model = ET.parse(Path(get_package_share_directory('mobile_manipulator_description')) / 'urdf/mobile_manipulator.urdf').getroot()
    # Conservative reference-payload operating envelope; JTC itself is not a trajectory retimer.
    acceleration = [2.0,1.5,2.0,3.0,3.0,4.0]
    for i,name in enumerate(NAMES):
        limit = model.find("joint[@name='"+name+"']/limit")
        delta = abs(target[i]-start[i])
        segment = args.duration-0.1
        if (not math.isfinite(target[i]) or not float(limit.get('lower')) <= target[i] <= float(limit.get('upper'))
                or 1.5*delta/segment > 0.5*float(limit.get('velocity'))
                or 6*delta/segment**2 > 0.5*acceleration[i]):
            raise ValueError('Unsafe cubic segment for '+name+'; increase duration or reduce displacement')
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = NAMES
    # Explicit zero endpoint velocities yield synchronized cubic interpolation in JTC.
    for duration, positions in ((0.1,start),(args.duration,target)):
        point = JointTrajectoryPoint(positions=positions, velocities=[0.0]*6)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = round((duration-int(duration))*1e9)
        goal.trajectory.points.append(point)
    future=client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node,future,timeout_sec=5)
    handle=future.result()
    if handle is None or not handle.accepted:
        raise RuntimeError('Goal rejected or response timed out')
    result=handle.get_result_async()
    began=time.monotonic()
    cancelled=False
    while not result.done() and time.monotonic()-began < 120:
        rclpy.spin_once(node,timeout_sec=0.05)
        if args.cancel_after and not cancelled and time.monotonic()-began >= args.cancel_after:
            cancel=handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node,cancel,timeout_sec=3)
            cancelled=True
    if not result.done():
        cancel=handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node,cancel,timeout_sec=3)
        raise RuntimeError('Experiment timed out; cancellation requested')
    response=result.result()
    hold_start=node.get_clock().now().nanoseconds*1e-9
    hold_max=[0.0]*6
    # (duration in simulation seconds, forward m/s, yaw rad/s). Always finish stopped.
    schedules={
        'none': [],
        'compact': [(3,.3,0),(1,0,0),(3,-.3,0),(1,0,0),(2,0,.4),(1,0,0),
                    (2,0,-.4),(1,0,0),(2,.15,.2),(1,0,0),(2,.15,-.2),(1,0,0),(2,-.3,0),(2,0,0)],
        'extended': [(2,.15,0),(1,0,0),(2,-.15,0),(1,0,0),(2,0,.2),(1,0,0),(2,0,-.2),(3,0,0)],
        'gate': [(18,.2,0),(3,0,0)]}
    schedule=schedules[args.disturbance]
    args.hold_seconds=max(args.hold_seconds,sum(s[0] for s in schedule))
    base_publisher=node.create_publisher(Twist,'/cmd_vel',10) if schedule else None
    hold_deadline=time.monotonic()+max(10,args.hold_seconds*5)
    last_publish=-1.0
    try:
        while response.result.error_code==0 and node.get_clock().now().nanoseconds*1e-9-hold_start < args.hold_seconds:
            rclpy.spin_once(node,timeout_sec=0.01)
            if time.monotonic()>hold_deadline or time.monotonic()-actual['_received']>0.5:
                raise RuntimeError('Hold observation lost feedback or simulation stopped')
            elapsed=node.get_clock().now().nanoseconds*1e-9-hold_start
            if base_publisher and elapsed-last_publish >= .05:
                command=Twist(); boundary=0.0
                for seconds,v,w in schedule:
                    boundary+=seconds
                    if elapsed<boundary:
                        command.linear.x=float(v); command.angular.z=float(w); break
                base_publisher.publish(command); last_publish=elapsed
            for i,n in enumerate(NAMES): hold_max[i]=max(hold_max[i],abs(actual[n]-target[i]))
    finally:
        if base_publisher:
            for _ in range(3):
                base_publisher.publish(Twist())
                rclpy.spin_once(node,timeout_sec=.05)
    report=dict(status=response.status,error_code=response.result.error_code,
                error_string=response.result.error_string,start=start,target=target,
                final=[actual[n] for n in NAMES],wall_seconds=time.monotonic()-began,
                hold_seconds=args.hold_seconds,hold_max_error=hold_max,disturbance=args.disturbance)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
    node.destroy_node()
    rclpy.shutdown()
    if response.result.error_code != 0 and not cancelled:
        raise SystemExit(1)

if __name__ == '__main__':
    main()
