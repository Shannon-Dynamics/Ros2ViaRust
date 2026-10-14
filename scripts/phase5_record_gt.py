#!/usr/bin/env python3
"""Write every /ground_truth/pose message to a CSV: stamp_s, x, y, yaw.

    python3 scripts/phase5_record_gt.py OUT.csv

Why: the pose columns of a recorded scan (integrated-scans.csv) are the ground truth at the scan's
stamp less a constant offset, and the Webots lidar's stamp does not follow its measurement instant
(docs/phase-4-mapping/implementation.md 5.2 item 2). While the robot drives straight that is a
few centimetres; while it turns at 0.8 rad/s it is up to a tenth of a radian. The dense track
(50 Hz, the supervisor's rate) lets an analysis ask what the ground truth was at any instant
instead of trusting the one the mapper picked. Harness only: nothing in the SLAM nodes reads it.
Stops on SIGINT or SIGTERM and flushes.
"""
import math
import signal
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GtRecorder(Node):
    def __init__(self, path):
        super().__init__('phase5_gt_recorder')
        self.f = open(path, 'w', buffering=1)
        self.f.write('# stamp_s,x,y,yaw\n')
        self.n = 0
        self.create_subscription(PoseStamped, '/ground_truth/pose', self.on_pose, 100)

    def on_pose(self, m):
        q = m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.f.write(f'{t:.9f},{m.pose.position.x!r},{m.pose.position.y!r},{yaw!r}\n')
        self.n += 1


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    rclpy.init()
    node = GtRecorder(sys.argv[1])
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    while rclpy.ok() and not stop:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.f.close()
    print(f'phase5_record_gt: {node.n} poses', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
