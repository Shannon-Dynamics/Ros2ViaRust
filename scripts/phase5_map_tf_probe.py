#!/usr/bin/env python3
"""Record what pose_graph_node puts on /map, /tf and /slam/path during a run.

    python3 scripts/phase5_map_tf_probe.py OUT.json

Used by scripts/verify_phase5_slam_nodes.sh (section G) and scripts/run_phase5_live.sh (Stage D): it
subscribes before the run starts and writes OUT.json when it gets SIGINT or SIGTERM:

  maps       one entry per /map message: header stamp, frame, size, resolution, origin, counts of free /
             occupied / unknown cells, a sha1 of the data, and the wall time of arrival (s since the probe
             started)
  tf         every map -> odom transform seen: header stamp, x, y, yaw, wall time; and every other
             (parent, child) edge seen on /tf with the number of messages, so that a second owner of an
             edge shows; static_edges: the edges of /tf_static
  paths      one entry per /slam/path message: frame, number of poses, wall time; and the poses of the last
  diagnostics_last   the key/value pairs of the newest pose_graph_node status (loop_closure, counts)

It decides nothing: scripts/phase5_stage_b_check.py reads the file. Harness only; not part of any launch.
"""
import hashlib
import json
import math
import signal
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Probe(Node):
    def __init__(self):
        super().__init__('phase5_map_tf_probe')
        self.t0 = time.monotonic()
        self.maps, self.map_odom, self.edges, self.paths = [], [], {}, []
        self.static_edges = {}
        self.last_path = []
        self.diag = {}
        latched = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        tf_qos = QoSProfile(depth=2000, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(Path, '/slam/path', self.on_path, latched)
        self.create_subscription(TFMessage, '/tf', self.on_tf, tf_qos)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf_static,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 50)

    def wall(self):
        return time.monotonic() - self.t0

    def on_map(self, m):
        data = bytes((v & 0xFF) for v in m.data)
        free = sum(1 for v in m.data if v == 0)
        occ = sum(1 for v in m.data if v == 100)
        self.maps.append({
            'stamp': stamp(m.header), 'frame': m.header.frame_id, 'width': m.info.width,
            'height': m.info.height, 'resolution': m.info.resolution,
            'origin': [m.info.origin.position.x, m.info.origin.position.y],
            'free': free, 'occupied': occ, 'unknown': len(m.data) - free - occ,
            'sha1': hashlib.sha1(data).hexdigest(), 'wall_s': self.wall()})

    def on_path(self, m):
        self.paths.append({'frame': m.header.frame_id, 'poses': len(m.poses), 'wall_s': self.wall()})
        self.last_path = [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)] for p in m.poses]

    def on_tf(self, msg):
        for t in msg.transforms:
            key = f'{t.header.frame_id}->{t.child_frame_id}'
            self.edges[key] = self.edges.get(key, 0) + 1
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                self.map_odom.append([stamp(t.header), t.transform.translation.x, t.transform.translation.y,
                                      yaw(t.transform.rotation), self.wall()])

    def on_tf_static(self, msg):
        for t in msg.transforms:
            key = f'{t.header.frame_id}->{t.child_frame_id}'
            self.static_edges[key] = [t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)]

    def on_diag(self, msg):
        for s in msg.status:
            if s.name.endswith('pose_graph_node'):
                self.diag = {kv.key: kv.value for kv in s.values}
                self.diag['_level'] = int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level)
                self.diag['_message'] = s.message


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    rclpy.init()
    node = Probe()
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    while rclpy.ok() and not stop:
        rclpy.spin_once(node, timeout_sec=0.1)
    with open(sys.argv[1], 'w') as f:
        json.dump({'maps': node.maps, 'map_odom': node.map_odom, 'edges': node.edges,
                   'static_edges': node.static_edges,
                   'paths': node.paths, 'last_path': node.last_path,
                   'diagnostics_last': node.diag}, f)
    print(f'phase5_map_tf_probe: {len(node.maps)} map(s), {len(node.map_odom)} map->odom, '
          f'{len(node.paths)} path message(s), edges {sorted(node.edges)}', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
