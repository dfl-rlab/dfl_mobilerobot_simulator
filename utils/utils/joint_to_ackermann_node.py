#!/usr/bin/env python3
"""
joint_to_ackermann_node
=======================
Subscribe to /joint_states and /odom, convert to AckermannDriveStamped feedback.

Published topic:
    /ackermann_feedback  (ackermann_msgs/AckermannDriveStamped)

Subscribed topics:
    /joint_states        (sensor_msgs/JointState)       — steering angles
    /odom                (nav_msgs/Odometry)             — vehicle speed

Steering angle computation:
    Uses the Ackermann → bicycle model harmonic-mean formula to convert
    left/right front steering angles into a single virtual centre angle:

        tan(centre) = 2 · tan(left) · tan(right) / (tan(left) + tan(right))

    This is exact for the Ackermann geometry and does NOT require wheel_base
    or track_width.

Vehicle parameters (from saye model.sdf):
    wheel_base      = 0.2255 m
    wheel_separation = 0.2 m
    wheel_radius     = 0.0365 m
"""

import math

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class JointToAckermannNode(Node):
    """Convert Gazebo joint states + odometry into AckermannDriveStamped."""

    # Joint names from model.sdf
    LEFT_STEER_JOINT = 'front_left_wheel_steering_joint'
    RIGHT_STEER_JOINT = 'front_right_wheel_steering_joint'

    def __init__(self):
        super().__init__('joint_to_ackermann_node')

        # --- Parameters ---
        self.declare_parameter('wheel_base', 0.2255)
        self.declare_parameter('pub_rate', 50.0)

        self.wheel_base = self.get_parameter('wheel_base').value
        pub_rate = self.get_parameter('pub_rate').value

        # --- State ---
        self.latest_steer_left = 0.0
        self.latest_steer_right = 0.0
        self.latest_steer_vel_left = 0.0
        self.latest_steer_vel_right = 0.0
        self.latest_v = 0.0
        self.prev_v = 0.0
        self.prev_v_time = None

        # --- Subscribers ---
        self.create_subscription(
            JointState, '/joint_states', self._cb_joint_states, 10)
        self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)

        # --- Publisher ---
        self.ack_pub = self.create_publisher(
            AckermannDriveStamped, '/ackermann_feedback', 10)

        # --- Timer ---
        self.create_timer(1.0 / pub_rate, self._publish)

        self.get_logger().info(
            f'joint_to_ackermann_node started  '
            f'(wheel_base={self.wheel_base}, rate={pub_rate} Hz)')

    # ─────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────
    def _cb_joint_states(self, msg: JointState):
        """Extract front steering joint positions and velocities."""
        for i, name in enumerate(msg.name):
            if name == self.LEFT_STEER_JOINT:
                if i < len(msg.position):
                    self.latest_steer_left = msg.position[i]
                if i < len(msg.velocity):
                    self.latest_steer_vel_left = msg.velocity[i]
            elif name == self.RIGHT_STEER_JOINT:
                if i < len(msg.position):
                    self.latest_steer_right = msg.position[i]
                if i < len(msg.velocity):
                    self.latest_steer_vel_right = msg.velocity[i]

    def _cb_odom(self, msg: Odometry):
        """Extract signed speed from odometry twist.

        Uses vx sign to distinguish forward (+) from reverse (-).
        """
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        magnitude = math.sqrt(vx ** 2 + vy ** 2)
        self.latest_v = math.copysign(magnitude, vx)

    # ─────────────────────────────────────────────────────────────────
    # Publish
    # ─────────────────────────────────────────────────────────────────
    def _publish(self):
        now = self.get_clock().now()

        # --- Steering angle (bicycle model via harmonic mean) ---
        tl = math.tan(self.latest_steer_left)
        tr = math.tan(self.latest_steer_right)
        denom = tl + tr
        if abs(denom) > 1e-9:
            centre_tan = (2.0 * tl * tr) / denom
            steering_angle = math.atan(centre_tan)
        else:
            steering_angle = 0.0

        # --- Steering angle velocity (average of left/right) ---
        steering_angle_velocity = (
            self.latest_steer_vel_left + self.latest_steer_vel_right) / 2.0

        # --- Acceleration (finite difference on speed) ---
        acceleration = 0.0
        if self.prev_v_time is not None:
            dt = (now - self.prev_v_time).nanoseconds * 1e-9
            if dt > 1e-6:
                acceleration = (self.latest_v - self.prev_v) / dt
        self.prev_v = self.latest_v
        self.prev_v_time = now

        # --- Build message ---
        msg = AckermannDriveStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = steering_angle_velocity
        msg.drive.speed = self.latest_v
        msg.drive.acceleration = acceleration
        msg.drive.jerk = 0.0  # not computed

        self.ack_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointToAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
