#!/usr/bin/env python3
"""Bounded /cmd_vel step, brake, watchdog, and breakaway tests for the Unity skid-steer base.

Simulation only. Requires Unity in Play with the base stopped in a surveyed open
area (for example after `unity command arm_test_place_open`), no other /cmd_vel
publisher, and fresh ground-truth TF. Every case drives out and back so the robot
ends near its start. Ground truth comes from Unity's odom -> base_footprint TF and
the wheel joint velocities on /joint_states; all timing uses simulation time.
With --breakaway-speeds/--breakaway-yaw-rates it runs only small steps from rest, to
find the smallest command that starts the base moving.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

WHEEL_RADIUS = 0.14
WHEELS = ('front_left_wheel_joint', 'rear_left_wheel_joint',
          'front_right_wheel_joint', 'rear_right_wheel_joint')
COMMAND_PERIOD = 0.05


def yaw_roll_pitch(q):
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return yaw, roll, pitch


def cases(speed, yaw_rate):
    """(name, segments); segment = (v, w, seconds, publish?)."""
    drive = max(4.0, 1.5 / speed)
    return [
        (f'forward-brake-{speed:.2f}', [(speed, 0, drive, True), (0, 0, 3, True),
                                        (-speed, 0, drive, True), (0, 0, 3, True)]),
        (f'reverse-brake-{speed:.2f}', [(-speed, 0, drive, True), (0, 0, 3, True),
                                        (speed, 0, drive, True), (0, 0, 3, True)]),
        (f'watchdog-stop-{speed:.2f}', [(speed, 0, drive, True), (0, 0, 3, False),
                                        (-speed, 0, drive, True), (0, 0, 3, False)]),
        (f'yaw-brake-{yaw_rate:.2f}', [(0, yaw_rate, 4, True), (0, 0, 3, True),
                                       (0, -yaw_rate, 4, True), (0, 0, 3, True)]),
    ]


def breakaway_cases(speeds, yaw_rates, seconds):
    """Small steps from rest, out and back: does the base start moving, and how fast?"""
    return ([(f'breakaway-v-{v:.4f}', [(v, 0, seconds, True), (0, 0, 2, True),
                                       (-v, 0, seconds, True), (0, 0, 2, True)]) for v in speeds]
            + [(f'breakaway-w-{w:.4f}', [(0, w, seconds, True), (0, 0, 2, True),
                                         (0, -w, seconds, True), (0, 0, 2, True)]) for w in yaw_rates])


class Recorder(Node):
    def __init__(self):
        super().__init__('base_step_test',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 50)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.samples = []
        self.wheels = None
        self.command = (0.0, 0.0, False)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        if all(name in index for name in WHEELS):
            self.wheels = [msg.velocity[index[name]] for name in WHEELS]

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id != 'base_footprint' or self.wheels is None:
                continue
            yaw, roll, pitch = yaw_roll_pitch(t.transform.rotation)
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            self.samples.append([stamp, t.transform.translation.x, t.transform.translation.y,
                                 yaw, roll, pitch, *self.wheels, *self.command])

    def spin_for(self, seconds, v, w, publish):
        self.command = (v, w, publish)
        end = self.now() + seconds
        next_publish = self.now()
        while self.now() < end:
            if publish and self.now() >= next_publish:
                message = Twist()
                message.linear.x = float(v)
                message.angular.z = float(w)
                self.publisher.publish(message)
                next_publish += COMMAND_PERIOD
            rclpy.spin_once(self, timeout_sec=0.01)


def derive(samples):
    """Body-frame velocities and accelerations from 50 Hz ground truth (centred differences)."""
    rows = []
    for i in range(2, len(samples) - 2):
        t0, x0, y0, yaw0 = samples[i - 2][:4]
        t1, x1, y1, yaw1 = samples[i + 2][:4]
        dt = t1 - t0
        if dt <= 0:
            continue
        heading = samples[i][3]
        vx_world, vy_world = (x1 - x0) / dt, (y1 - y0) / dt
        v = math.cos(heading) * vx_world + math.sin(heading) * vy_world
        lateral = -math.sin(heading) * vx_world + math.cos(heading) * vy_world
        w = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0)) / dt
        wheels = samples[i][6:10]
        rim_left = WHEEL_RADIUS * (wheels[0] + wheels[1]) / 2
        rim_right = WHEEL_RADIUS * (wheels[2] + wheels[3]) / 2
        rows.append({'t': samples[i][0], 'x': samples[i][1], 'y': samples[i][2],
                     'yaw': heading, 'roll': samples[i][4], 'pitch': samples[i][5],
                     'v': v, 'lateral': lateral, 'w': w,
                     'rim_left': rim_left, 'rim_right': rim_right,
                     'cmd_v': samples[i][10], 'cmd_w': samples[i][11],
                     'publishing': samples[i][12]})
    for i in range(2, len(rows) - 2):
        dt = rows[i + 2]['t'] - rows[i - 2]['t']
        rows[i]['a'] = (rows[i + 2]['v'] - rows[i - 2]['v']) / dt if dt > 0 else 0.0
        rows[i]['alpha'] = (rows[i + 2]['w'] - rows[i - 2]['w']) / dt if dt > 0 else 0.0
    return rows[2:-2]


def summarize(rows, segments):
    """Per-segment steady state, acceleration, stopping distance and slip."""
    start = rows[0]['t']
    boundaries, elapsed = [], 0.0
    for v, w, seconds, publish in segments:
        boundaries.append((start + elapsed, start + elapsed + seconds, v, w, publish))
        elapsed += seconds
    result = []
    for begin, end, v, w, publish in boundaries:
        part = [r for r in rows if begin <= r['t'] < end]
        if len(part) < 10:
            continue
        linear = abs(v) > 0 or (v == 0 and w == 0 and abs(part[0]['v']) > 0.02)
        key, accel = ('v', 'a') if linear else ('w', 'alpha')
        entry = {'command': [v, w], 'published': publish, 'duration_s': end - begin,
                 'peak_abs_accel': max(abs(r[accel]) for r in part),
                 'max_abs_lateral_mps': max(abs(r['lateral']) for r in part),
                 'max_tilt_deg': math.degrees(max(max(abs(r['roll']), abs(r['pitch'])) for r in part))}
        if v != 0 or w != 0:
            settled = [r[key] for r in part if r['t'] > end - 1.5]
            entry['steady_state'] = sum(settled) / len(settled)
            target = entry['steady_state']
            lo = next((r['t'] for r in part if abs(r[key]) >= 0.1 * abs(target)), None)
            hi = next((r['t'] for r in part if abs(r[key]) >= 0.9 * abs(target)), None)
            if lo is not None and hi is not None and hi > lo:
                entry['mean_accel_10_90'] = 0.8 * abs(target) / (hi - lo)
        else:
            initial = part[0][key]
            entry['initial'] = initial
            stop = next((r for r in part if abs(r[key]) < 0.01), None)
            if stop is not None:
                entry['time_to_stop_s'] = stop['t'] - begin
                if linear:
                    entry['stopping_distance_m'] = math.hypot(stop['x'] - part[0]['x'],
                                                              stop['y'] - part[0]['y'])
                if entry['time_to_stop_s'] > 0:
                    entry['mean_decel'] = abs(initial) / entry['time_to_stop_s']
            if linear:
                # Positive slip: wheel rims turn faster (same sign) than the chassis moves.
                slip = [((r['rim_left'] + r['rim_right']) / 2 - r['v']) for r in part
                        if abs(r['v']) > 0.02]
                entry['max_abs_rim_minus_body_mps'] = max((abs(s) for s in slip), default=0.0)
        result.append(entry)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speeds', type=float, nargs='+', default=[0.3])
    parser.add_argument('--yaw-rate', type=float, default=0.4)
    parser.add_argument('--breakaway-speeds', type=float, nargs='*', default=[],
                        help='m/s; run breakaway steps instead of the step/brake cases')
    parser.add_argument('--breakaway-yaw-rates', type=float, nargs='*', default=[],
                        help='rad/s; in-place breakaway steps')
    parser.add_argument('--breakaway-seconds', type=float, default=4.0)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--prefix', required=True)
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Recorder()
    while node.now() == 0.0:
        rclpy.spin_once(node, timeout_sec=0.1)  # wait for the first /clock
    node.spin_for(2.0, 0, 0, False)
    others = node.count_publishers('/cmd_vel') - 1
    if others != 0:
        raise SystemExit(f'Refusing to command: {others} other /cmd_vel publisher(s)')
    if len(node.samples) < 20:
        raise SystemExit('Refusing to command: no fresh odom -> base_footprint TF')
    recent = derive(node.samples[-15:]) if len(node.samples) >= 15 else []
    if any(abs(r['v']) > 0.02 or abs(r['w']) > 0.02 for r in recent):
        raise SystemExit('Refusing to command: base is not stationary')

    report = {'wheel_radius_m': WHEEL_RADIUS, 'command_rate_hz': 1 / COMMAND_PERIOD, 'cases': {}}
    if args.breakaway_speeds or args.breakaway_yaw_rates:
        selected = breakaway_cases(args.breakaway_speeds, args.breakaway_yaw_rates,
                                   args.breakaway_seconds)
    else:
        selected = [c for s in args.speeds for c in cases(s, args.yaw_rate)
                    if not c[0].startswith('yaw')] + [cases(args.speeds[0], args.yaw_rate)[-1]]
    for name, segments in selected:
        stem = f'{args.prefix}-{name}'
        if (output / f'{stem}.csv').exists():
            raise SystemExit(f'Refusing to overwrite evidence: {stem}.csv')
        node.samples = []
        for v, w, seconds, publish in segments:
            node.spin_for(seconds, v, w, publish)
        node.spin_for(1.0, 0, 0, True)
        rows = derive(node.samples)
        with (output / f'{stem}.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()), lineterminator='\n')
            writer.writeheader()
            writer.writerows(rows)
        report['cases'][name] = summarize(rows, segments)
        print(name, json.dumps(report['cases'][name], indent=1), flush=True)

    (output / f'{args.prefix}-summary.json').write_text(json.dumps(report, indent=2))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
