#!/usr/bin/env python3
"""Record every /scan of a live run in the schema of integrated-scans.csv, with ground-truth pose columns.

    python3 scripts/phase5_record_scans.py OUT.csv [--offset -0.06] [--mount-x -0.03]

Why not the Phase 4 mapper again: the mapper looks the sensor pose up in TF in a `map` frame, and in a
live Phase 5 run `map -> odom` is owned by pose_graph_node (the SLAM estimate), so a mapper-recorded pose would
be the SLAM's own and not the ground truth. This recorder reads /scan and /ground_truth/pose directly:

    pose columns = ground truth of base_link at (stamp + offset) composed with the lidar mount

with `offset` -0.06 s (the Phase 4 convention, D15) and the mount (-0.03, 0, 0) read off the first row of
the Stage A recordings (x -0.029999999152978334 at yaw 1.7e-8). Ground truth between its 50 Hz samples is
interpolated linearly in position and along the shortest arc in heading. Rows are written when the ground
truth has caught up with the instant they need; the ones it never reaches are not written and are counted.

It is the same file a mapper-recorded run produces, from the same messages, so every Phase 5 tool reads it.
Harness only: nothing in the SLAM nodes reads ground truth or this file.
"""
import argparse
import bisect
import math
import signal
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Recorder(Node):
    def __init__(self, path, offset, mount_x):
        super().__init__('phase5_scan_recorder')
        self.f = open(path, 'w', buffering=1 << 16)
        self.f.write('# stamp_s,sensor_x,sensor_y,sensor_theta,angle_min,angle_increment,range_min,range_max,n,ranges...\n')
        self.offset, self.mount_x = offset, mount_x
        self.t, self.x, self.y, self.yaw = [], [], [], []
        self.pending = []
        self.written = self.skipped = self.received = 0
        self.create_subscription(PoseStamped, '/ground_truth/pose', self.on_gt, 200)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_gt(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.yaw:
            yaw = self.yaw[-1] + wrap(yaw - self.yaw[-1])
        self.t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.x.append(m.pose.position.x)
        self.y.append(m.pose.position.y)
        self.yaw.append(yaw)
        self.flush(final=False)

    def on_scan(self, m):
        self.received += 1
        self.pending.append(m)
        self.flush(final=False)

    def pose_at(self, when):
        i = bisect.bisect_right(self.t, when)
        if i <= 0 or i >= len(self.t):
            return None
        a = (when - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        return (self.x[i - 1] + a * (self.x[i] - self.x[i - 1]), self.y[i - 1] + a * (self.y[i] - self.y[i - 1]),
                self.yaw[i - 1] + a * (self.yaw[i] - self.yaw[i - 1]))

    def flush(self, final):
        while self.pending:
            m = self.pending[0]
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            need = stamp + self.offset
            if not final and (not self.t or self.t[-1] < need):
                return
            p = self.pose_at(need)
            self.pending.pop(0)
            if p is None:
                self.skipped += 1
                continue
            c, s = math.cos(p[2]), math.sin(p[2])
            sx, sy, sth = p[0] + c * self.mount_x, p[1] + s * self.mount_x, wrap(p[2])
            r = ','.join(str(np.float32(v)) for v in m.ranges)
            self.f.write(f'{stamp:.9f},{sx!r},{sy!r},{sth!r},{m.angle_min!r},{m.angle_increment!r},'
                         f'{m.range_min!r},{m.range_max!r},{len(m.ranges)},{r}\n')
            self.written += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--offset', type=float, default=-0.06)
    ap.add_argument('--mount-x', type=float, default=-0.03)
    a = ap.parse_args()
    rclpy.init()
    node = Recorder(a.out, a.offset, a.mount_x)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    while rclpy.ok() and not stop:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.flush(final=True)
    node.f.close()
    print(f'phase5_record_scans: {node.received} scans received, {node.written} written, {node.skipped} without ground truth',
          flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
