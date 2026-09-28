#!/usr/bin/env python3
"""Aggregate navigation state into one small message for the Unity telemetry window.

Publishes /mm/telemetry (std_msgs/String, JSON) at 5 Hz and /mm/telemetry/path
(nav_msgs/Path, transient local) downsampled to 0.1 m only when the global plan changes.
Everything Unity already knows (base motion, arm state, contacts, clock) stays in Unity;
this node only forwards ROS-side navigation state, keeping the Unity link to about
2-3 KB/s. Set the `scenario` parameter to label runs.
"""
import json
import math

import rclpy
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from mobile_manipulator_navigation.telemetry import (
    GOAL_STATUS, MONITOR_ACTION, EventLog, GapCounter, cross_track, downsample, path_length)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class NavTelemetry(Node):
    def __init__(self):
        super().__init__("nav_telemetry", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.scenario = self.declare_parameter("scenario", "").value
        self.labels = {name: self.declare_parameter(name, default).value for name, default in (
            ("footprint_profile", ""), ("behavior_tree", ""),
            ("planner", "Lattice"), ("controller", "RPP"))}
        self.add_on_set_parameters_callback(self.on_parameters)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.events = EventLog()
        self.lidar = GapCounter(0.5)
        self.path, self.path_stamp = [], None
        self.cross_track_max = 0.0
        self.goal = self.empty_goal("idle")
        self.goal_id, self.goal_started, self.goal_seq = None, None, 0
        self.monitor = ("none", "")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.telemetry_pub = self.create_publisher(String, "/mm/telemetry", 1)
        self.path_pub = self.create_publisher(Path, "/mm/telemetry/path", latched)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self.on_status, 10)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage,
                                 "/navigate_to_pose/_action/feedback", self.on_feedback, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self.on_monitor, 10)
        self.create_subscription(PointCloud2, "/livox/points_filtered", self.on_lidar,
                                 qos_profile_sensor_data)
        self.create_timer(0.2, self.publish)
        # The ROS-TCP endpoint subscribes with volatile QoS, so a latched path is not
        # replayed to a reconnecting Unity; resend the (small) current path every 5 s.
        self.last_path = None
        self.create_timer(5.0, self.republish_path)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def empty_goal(status):
        return {"status": status, "recoveries": 0, "distance_remaining_m": -1,
                "eta_s": -1, "elapsed_s": 0, "pose": [0.0, 0.0]}

    def on_parameters(self, parameters):
        from rcl_interfaces.msg import SetParametersResult
        for parameter in parameters:
            if parameter.name == "scenario":
                self.scenario = parameter.value
                self.events.add(self.now(), f"scenario {parameter.value}")
            elif parameter.name in self.labels:
                self.labels[parameter.name] = parameter.value
        return SetParametersResult(successful=True)

    def on_plan(self, message):
        points = [(p.pose.position.x, p.pose.position.y,
                   2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w)) for p in message.poses]
        if points == self.path:
            return
        self.path, self.path_stamp = points, self.now()
        self.events.add(self.now(), f"new plan {path_length(points):.1f} m")
        out = Path()
        out.header = message.header
        kept = downsample([(x, y, yaw, index) for index, (x, y, yaw) in enumerate(points)], 0.1)
        out.poses = [message.poses[point[3]] for point in kept]
        self.last_path = out
        self.path_pub.publish(out)

    def republish_path(self):
        if self.last_path is not None:
            self.path_pub.publish(self.last_path)

    def on_status(self, message):
        if not message.status_list:
            return
        latest = message.status_list[-1]
        goal_id = bytes(latest.goal_info.goal_id.uuid)
        status = GOAL_STATUS.get(latest.status, str(latest.status))
        if goal_id != self.goal_id:
            self.goal_id, self.goal_started = goal_id, self.now()
            self.goal_seq += 1
            self.cross_track_max = 0.0
            self.goal = self.empty_goal(status)
            self.events.add(self.now(), "goal received")
        if status != self.goal.get("status"):
            self.events.add(self.now(), f"goal {status}")
        self.goal["status"] = status

    def on_feedback(self, message):
        feedback = message.feedback
        recoveries = feedback.number_of_recoveries
        if recoveries > self.goal.get("recoveries", 0):
            self.events.add(self.now(), f"recovery {recoveries}")
        pose = feedback.current_pose.pose
        self.goal.update({
            "recoveries": recoveries,
            "distance_remaining_m": round(feedback.distance_remaining, 2),
            "eta_s": round(stamp_seconds(feedback.estimated_time_remaining), 1),
            "elapsed_s": round(stamp_seconds(feedback.navigation_time), 1),
            "pose": [round(pose.position.x, 2), round(pose.position.y, 2)],
        })

    def on_monitor(self, message):
        state = (MONITOR_ACTION.get(message.action_type, str(message.action_type)), message.polygon_name)
        if state != self.monitor and state[0] != "none":
            self.events.add(self.now(), f"monitor {state[0]} ({state[1]})")
        self.monitor = state

    def on_lidar(self, message):
        gap = self.lidar.add(stamp_seconds(message.header.stamp))
        if gap is not None:
            self.events.add(self.now(), f"lidar gap {gap:.2f} s")

    def publish(self):
        position, deviation = None, None
        try:
            t = self.buffer.lookup_transform("map", "base_footprint", Time()).transform
            position = (t.translation.x, t.translation.y)
        except TransformException:
            pass
        if position is not None and self.path and self.goal.get("status") == "executing":
            deviation = cross_track(position, self.path)
            self.cross_track_max = max(self.cross_track_max, deviation)
        # Unity's JsonUtility has no null: unavailable numbers are sent as -1.
        message = {
            "t": round(self.now(), 2),
            "scenario": self.scenario,
            **self.labels,
            "goal_seq": self.goal_seq,
            "goal": self.goal,
            "path": {"length_m": round(path_length(self.path), 2), "points": len(self.path),
                     "age_s": round(self.now() - self.path_stamp, 1) if self.path_stamp else -1,
                     "cross_track_m": round(deviation, 3) if deviation is not None else -1,
                     "cross_track_max_m": round(self.cross_track_max, 3)},
            "monitor": {"action": self.monitor[0], "polygon": self.monitor[1]},
            "lidar": {"rate_hz": round(self.lidar.rate(), 1), "gaps_over_0p5s": self.lidar.long_gaps,
                      "max_gap_s": round(self.lidar.max_gap, 2)},
            "events": self.events.as_list(),
        }
        self.telemetry_pub.publish(String(data=json.dumps(message, separators=(",", ":"))))


def main():
    rclpy.init()
    node = NavTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
