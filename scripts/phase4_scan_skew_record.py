#!/usr/bin/env python3
"""Record raw lidar scans and ground truth while the robot stands still and then moves.

    python3 scripts/phase4_scan_skew_record.py <output directory>

Phase 4 open question 3 (docs/phase-4-mapping/implementation.md section 5): is one LDS-01
sweep stamped, and simulated, as a single instant, or as a rotation during which the robot
moves? This node only records; scripts/phase4_scan_skew_analysis.py decides, from the data.

It needs a running world with ground truth
(`ros2 launch sim/webots/launch/localization_world.launch.py gui:=false ground_truth:=true`)
and drives the robot itself, through the same TwistStamped /cmd_vel the Phase 3 route driver
uses, at the speeds that driver uses (0.18 m/s straight, 0.8 rad/s in place), plus one
deliberately faster rotation so that an effect too small to see at driving speed is not
mistaken for no effect. It reads /ground_truth/pose, which is allowed here for the same
reason it is allowed for an evaluator: this is a measurement harness, not an estimator.

Written next to each other, in the output directory:

  scans.csv    one row per /scan message: header stamp, frame, the LaserScan timing fields
               exactly as published (time_increment, scan_time), angle limits, and the ranges
  gt.csv       one row per /ground_truth/pose sample: stamp, x, y, qz, qw
  phases.csv   when each commanded phase started, in simulation time
"""
import csv
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

# (name, linear m/s, angular rad/s, duration s). The first phase is the noise floor: with
# the robot standing still any difference between "instant" and "swept" must be zero.
PHASES = [
    ("still", 0.0, 0.0, 8.0),
    ("straight_0.18", 0.18, 0.0, 8.0),
    ("stop_a", 0.0, 0.0, 3.0),
    ("turn_0.8", 0.0, 0.8, 8.0),
    ("stop_b", 0.0, 0.0, 3.0),
    ("turn_2.0_stress", 0.0, 2.0, 6.0),
    ("stop_c", 0.0, 0.0, 3.0),
]


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, out):
        super().__init__("phase4_scan_skew_record", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        os.makedirs(out, exist_ok=True)
        self.scan_f = open(os.path.join(out, "scans.csv"), "w", newline="")
        self.gt_f = open(os.path.join(out, "gt.csv"), "w", newline="")
        self.ph_f = open(os.path.join(out, "phases.csv"), "w", newline="")
        self.scan_w = csv.writer(self.scan_f)
        self.gt_w = csv.writer(self.gt_f)
        self.ph_w = csv.writer(self.ph_f)
        self.gt_w.writerow(["stamp_s", "x", "y", "qz", "qw"])
        self.ph_w.writerow(["phase", "linear", "angular", "start_sim_s", "duration_s"])
        self.scan_header_written = False
        self.n_scans = 0
        self.n_gt = 0
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/ground_truth/pose", self.on_gt, 10)
        self.phase = -1
        self.phase_end = None
        self.timer = self.create_timer(0.05, self.tick)
        self.done = False

    def on_scan(self, m):
        if not self.scan_header_written:
            self.scan_w.writerow(
                ["stamp_s", "frame_id", "angle_min", "angle_max", "angle_increment",
                 "time_increment", "scan_time", "range_min", "range_max", "n"]
                + [f"r{i}" for i in range(len(m.ranges))])
            self.scan_header_written = True
        self.scan_w.writerow(
            [f"{stamp_s(m.header.stamp):.6f}", m.header.frame_id, m.angle_min, m.angle_max,
             m.angle_increment, m.time_increment, m.scan_time, m.range_min, m.range_max,
             len(m.ranges)] + [repr(float(r)) for r in m.ranges])
        self.n_scans += 1

    def on_gt(self, m):
        q = m.pose.orientation
        self.gt_w.writerow([f"{stamp_s(m.header.stamp):.6f}", repr(m.pose.position.x),
                            repr(m.pose.position.y), repr(q.z), repr(q.w)])
        self.n_gt += 1

    def send(self, lin, ang):
        msg = TwistStamped()  # zero stamp: see route_driver.rs send() for why
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        self.pub.publish(msg)

    def tick(self):
        now = self.get_clock().now()
        if self.n_scans < 3 or self.n_gt < 10:
            return  # wait for both streams before the first phase starts
        if self.phase_end is None or now >= self.phase_end:
            self.phase += 1
            if self.phase >= len(PHASES):
                self.send(0.0, 0.0)
                self.done = True
                return
            name, lin, ang, dur = PHASES[self.phase]
            self.ph_w.writerow([name, lin, ang, f"{now.nanoseconds * 1e-9:.6f}", dur])
            self.ph_f.flush()
            self.phase_end = Time(nanoseconds=now.nanoseconds + int(dur * 1e9),
                                  clock_type=now.clock_type)
            self.get_logger().info(f"phase {name}")
        _, lin, ang, _ = PHASES[self.phase]
        self.send(lin, ang)


def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    rclpy.init()
    node = Recorder(sys.argv[1])
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.send(0.0, 0.0)
        for f in (node.scan_f, node.gt_f, node.ph_f):
            f.close()
        node.get_logger().info(f"scans {node.n_scans} gt {node.n_gt}")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
