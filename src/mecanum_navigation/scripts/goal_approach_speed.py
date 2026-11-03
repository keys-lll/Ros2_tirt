#!/usr/bin/env python3
"""Gradually lower the controller's speed limit near the current plan endpoint."""
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Path
from nav2_msgs.msg import SpeedLimit
from action_msgs.msg import GoalStatusArray, GoalStatus
from tf2_ros import Buffer, TransformListener, TransformException


def approach_limit(distance, start=0.2, end=0.1, cruise=0.25, minimum=0.07):
    ratio = max(0.0, min(1.0, (distance - end) / (start - end)))
    return minimum + ratio * (cruise - minimum)


class ApproachSpeed(Node):
    def __init__(self):
        super().__init__('goal_approach_speed')
        self.goal = None
        self.active = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.create_subscription(Path, '/plan', self.plan, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status, 10)
        self.create_timer(0.1, self.update)

    def plan(self, msg):
        if msg.poses:
            self.goal = (msg.header.frame_id, msg.poses[-1].pose.position)

    def status(self, msg):
        self.active = any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
                          for s in msg.status_list)
        if not self.active:
            self.goal = None

    def update(self):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        # Zero means reset to configured maximum, not stop.
        msg.speed_limit = 0.0
        if self.active and self.goal:
            frame, goal = self.goal
            try:
                transform = self.buffer.lookup_transform(frame, 'base_link', Time())
                stamp = Time.from_msg(transform.header.stamp)
                age = (self.get_clock().now() - stamp).nanoseconds / 1e9
                if not -0.5 <= age <= 1.0:
                    raise ValueError('stale robot pose')
                pos = transform.transform.translation
                distance = math.hypot(goal.x - pos.x, goal.y - pos.y)
                # Inside the position tolerance, DWB brakes translation and aligns yaw.
                # Reset the external speed cap so final rotation can use its own limits.
                msg.speed_limit = 0.0 if distance <= 0.1 else approach_limit(distance)
            except (TransformException, ValueError):
                msg.speed_limit = 0.07
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ApproachSpeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
