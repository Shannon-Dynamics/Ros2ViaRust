#!/usr/bin/env python3
"""Ground truth and kidnapping, as an extern Webots Supervisor controller.

Two things that only the simulator can provide, and that no estimator may ever read:

* **Ground truth.** The true pose of the robot, read out of the Webots scene tree and
  published on ``/ground_truth/pose`` (``geometry_msgs/PoseStamped``, frame ``map``).
  It exists so that an estimate can be graded from the outside. ``localization_node``
  does not subscribe to it, which is what makes the comparison meaningful.
* **Kidnapping.** A teleport: on a message to ``/kidnap/teleport``
  (``geometry_msgs/PoseStamped``) the robot is moved instantly to that pose through the
  Supervisor API, with no force, no wheel motion and no announcement. Nothing tells the
  localization node that it happened, which is exactly the kidnapped robot problem of
  Thrun and Fox: the estimator has to notice, from its sensors alone, that its belief has
  become impossible.

  Every teleport is echoed on ``/kidnap/events`` (``geometry_msgs/PoseStamped``, stamped
  with the simulation time it happened at and carrying the target pose), which the
  evaluator uses to split a run into before and after. That topic is evidence, not a
  signal to the estimator.

This attaches to the ``evaluation_supervisor`` Robot node of the world, which has no
devices of its own. The official ``webots_ros2_turtlebot`` driver and the official
``Ros2Supervisor`` are untouched by it.

Environment:

    GROUND_TRUTH_RATE_HZ   publish rate of the ground truth (default 50)
    ROBOT_DEF              DEF name of the robot node in the world (default TURTLEBOT3)
    GROUND_TRUTH_FRAME     frame id of the published pose (default map)
"""

import math
import os

import rclpy
from controller import Supervisor
from geometry_msgs.msg import PoseStamped


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class EvaluationSupervisor:
    def __init__(self):
        self.sup = Supervisor()
        self.dt = int(self.sup.getBasicTimeStep())
        def_name = os.environ.get('ROBOT_DEF', 'TURTLEBOT3')
        self.robot = self.sup.getFromDef(def_name)
        if self.robot is None:
            raise RuntimeError(f'no node with DEF {def_name} in this world')
        self.translation = self.robot.getField('translation')
        self.rotation = self.robot.getField('rotation')
        self.frame = os.environ.get('GROUND_TRUTH_FRAME', 'map')

        rclpy.init(args=None)
        self.node = rclpy.create_node('evaluation_supervisor')
        self.pose_pub = self.node.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        self.event_pub = self.node.create_publisher(PoseStamped, '/kidnap/events', 10)
        self.node.create_subscription(PoseStamped, '/kidnap/teleport', self.on_teleport, 10)

        rate = float(os.environ.get('GROUND_TRUTH_RATE_HZ', '50'))
        self.every = max(1, int(round(1000.0 / rate / self.dt)))
        self.pending = []
        self.node.get_logger().info(
            f'evaluation_supervisor: {def_name}, ground truth every {self.every} step(s) '
            f'of {self.dt} ms, frame {self.frame}'
        )

    def stamp(self):
        t = self.sup.getTime()
        msg = PoseStamped()
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
        msg.header.frame_id = self.frame
        return msg

    def true_pose(self):
        p = self.robot.getPosition()
        r = self.robot.getOrientation()
        # Row major 3x3; the planar heading is the direction of the body x axis.
        return p[0], p[1], math.atan2(r[3], r[0])

    def publish_ground_truth(self):
        x, y, yaw = self.true_pose()
        msg = self.stamp()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        q = quaternion_from_yaw(yaw)
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        self.pose_pub.publish(msg)

    def on_teleport(self, msg):
        # Applied in the control step, not in the callback, so that the pose is set
        # between two simulation steps.
        self.pending.append(msg)

    def apply_teleport(self, msg):
        yaw = yaw_from_quaternion(msg.pose.orientation)
        x, y = msg.pose.position.x, msg.pose.position.y
        z = self.translation.getSFVec3f()[2]
        before = self.true_pose()
        self.translation.setSFVec3f([x, y, z])
        self.rotation.setSFRotation([0.0, 0.0, 1.0, yaw])
        self.robot.resetPhysics()
        event = self.stamp()
        event.pose = msg.pose
        self.event_pub.publish(event)
        self.node.get_logger().warn(
            f'KIDNAP at sim t={self.sup.getTime():.3f}: '
            f'({before[0]:.3f}, {before[1]:.3f}, {before[2]:.3f}) -> ({x:.3f}, {y:.3f}, {yaw:.3f}). '
            f'Nothing was told to the estimator.'
        )

    def run(self):
        step = 0
        while self.sup.step(self.dt) != -1:
            rclpy.spin_once(self.node, timeout_sec=0)
            while self.pending:
                self.apply_teleport(self.pending.pop(0))
            if step % self.every == 0:
                self.publish_ground_truth()
            step += 1


if __name__ == '__main__':
    EvaluationSupervisor().run()
