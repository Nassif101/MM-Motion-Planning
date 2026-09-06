#!/usr/bin/env python3
"""Bounded arm-specific transport benchmark; simulation stamps are not one-way network latency."""
import argparse
import json
import statistics
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=8)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error('Benchmark duration must be 1..60 seconds')
    rclpy.init()
    node=Node('arm_transport_benchmark')
    samples={topic:[] for topic in ['/arm/command','/arm/state','/clock']}
    subscriptions=[]
    latest_clock=[None]
    for topic in samples:
        def receive(msg,t=topic):
            stamp=msg.clock if t=='/clock' else msg.header.stamp
            seconds=stamp.sec+stamp.nanosec*1e-9
            if t=='/clock': latest_clock[0]=seconds
            samples[t].append((time.monotonic(),seconds,latest_clock[0]))
        # The observer must retain bursts rather than manufacture a lower measured rate.
        subscriptions.append(node.create_subscription(Clock if topic=='/clock' else JointState,topic,receive,1000))
    stop=time.monotonic()+args.seconds
    while time.monotonic()<stop:
        rclpy.spin_once(node,timeout_sec=.1)
    result={}
    for topic,points in samples.items():
        intervals=[b[0]-a[0] for a,b in zip(points,points[1:])]
        if not intervals:
            raise RuntimeError('No transport samples: '+topic)
        stamp_intervals=[b[1]-a[1] for a,b in zip(points,points[1:])]
        lag=[clock-stamp for _,stamp,clock in points if clock is not None]
        result[topic]={'count':len(points),'wall_hz':(len(points)-1)/(points[-1][0]-points[0][0]),
                       'simulation_hz':(len(points)-1)/(points[-1][1]-points[0][1]),
                       'median_wall_interval':statistics.median(intervals),'max_wall_interval':max(intervals),
                       'median_stamp_interval':statistics.median(stamp_intervals),
                       'max_stamp_interval':max(stamp_intervals),
                       'nonincreasing_stamps':sum(dt<=0 for dt in stamp_intervals),
                       'max_observed_clock_minus_stamp':max(lag,default=0),
                       'median_observed_clock_minus_stamp':statistics.median(lag) if lag else None}
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
