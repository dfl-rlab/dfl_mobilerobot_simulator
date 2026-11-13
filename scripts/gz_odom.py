#!/usr/bin/env python3
"""Publish filtered odometry and TF from the Harmonic odometry bridge."""
import argparse
import math
from collections import deque
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time as TimeMsg

def quat_normalize(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x/n, y/n, z/n, w/n


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def rot_matrix_from_quat(x, y, z, w):
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return [
        [1.0 - 2.0*(yy + zz),  2.0*(xy - wz),       2.0*(xz + wy)],
        [    2.0*(xy + wz),  1.0 - 2.0*(xx + zz),   2.0*(yz - wx)],
        [    2.0*(xz - wy),      2.0*(yz + wx),  1.0 - 2.0*(xx + yy)]
    ]


def mat_vec(R, v):
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]


def median_of(deq):
    if not deq:
        return 0.0
    arr = sorted(deq)
    n = len(arr)
    m = n // 2
    return float(arr[m]) if n % 2 == 1 else float(0.5 * (arr[m-1] + arr[m]))


class OnePoleEMA:
    def __init__(self, alpha: float):
        self.alpha = float(alpha)
        self.y = None

    def reset(self):
        self.y = None

    def filt(self, x: float) -> float:
        if self.y is None:
            self.y = float(x)
        else:
            self.y = self.alpha * float(x) + (1.0 - self.alpha) * self.y
        return self.y


class GzToOdomTF(Node):
    def __init__(self, args):
        super().__init__('gz_to_odom_tf_filtered')
        self.entity_name = self.pose_name = 'robot_model'
        self.tf_parent, self.tf_child = args.tf_parent, args.tf_child
        self.max_dt = args.max_dt
        self.deadband_lin, self.deadband_ang = args.deadband_lin, args.deadband_ang
        self.prev_t = self.prev_pos = self.prev_yaw = None
        self.vx_med, self.vy_med, self.wz_med = (deque(maxlen=max(1, args.median_window)) for _ in range(3))
        self.vx_ema, self.vy_ema, self.wz_ema = (OnePoleEMA(args.alpha) for _ in range(3))
        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, '/simulation/odom', self.on_odom, 10)

    def on_odom(self, msg):
        self.current_sec = msg.header.stamp.sec
        self.current_nsec = msg.header.stamp.nanosec
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pos = dict(x=p.x, y=p.y, z=p.z)
        self.ori = dict(x=q.x, y=q.y, z=q.z, w=q.w)
        self._maybe_publish()

    def _apply_filters(self, vx_b_raw, vy_b_raw, wz_raw):
        self.vx_med.append(float(vx_b_raw))
        self.vy_med.append(float(vy_b_raw))
        self.wz_med.append(float(wz_raw))

        vx_sm = self.vx_ema.filt(median_of(self.vx_med))
        vy_sm = self.vy_ema.filt(median_of(self.vy_med))
        wz_sm = self.wz_ema.filt(median_of(self.wz_med))

        if abs(vx_sm) < self.deadband_lin: vx_sm = 0.0
        if abs(vy_sm) < self.deadband_lin: vy_sm = 0.0
        if abs(wz_sm) < self.deadband_ang: wz_sm = 0.0

        return vx_sm, vy_sm, wz_sm

    def _maybe_publish(self):
        # Process only the target entity
        if self.pose_name != self.entity_name:
            return
        if None in self.pos.values() or None in self.ori.values():
            return

        if self.current_sec is not None and self.current_nsec is not None:
            stamp   = TimeMsg(sec=int(self.current_sec), nanosec=int(self.current_nsec))
            t_float = float(self.current_sec) + float(self.current_nsec) * 1e-9
        else:
            now_ros = self.get_clock().now()
            stamp   = now_ros.to_msg()
            t_float = now_ros.nanoseconds * 1e-9

        px = float(self.pos['x'])
        py = float(self.pos['y'])
        pz = float(self.pos['z'])
        ox, oy, oz, ow = quat_normalize(
            float(self.ori['x']), float(self.ori['y']),
            float(self.ori['z']), float(self.ori['w'])
        )

        vx_b = vy_b = vz_b = wz = 0.0
        if self.prev_t is not None:
            dt = t_float - self.prev_t
            if 1e-6 < dt <= self.max_dt:
                # World-frame linear velocity
                vx_w = (px - self.prev_pos[0]) / dt
                vy_w = (py - self.prev_pos[1]) / dt
                vz_w = (pz - self.prev_pos[2]) / dt

                # Rotate to the body frame (R^T * v_world)
                R  = rot_matrix_from_quat(ox, oy, oz, ow)
                RT = [[R[j][i] for j in range(3)] for i in range(3)]
                vx_b, vy_b, vz_b = mat_vec(RT, [vx_w, vy_w, vz_w])

                # Yaw rate about the Z axis
                yaw  = quat_to_yaw(ox, oy, oz, ow)
                dyaw = yaw - self.prev_yaw
                if   dyaw >  math.pi: dyaw -= 2.0 * math.pi
                elif dyaw < -math.pi: dyaw += 2.0 * math.pi
                wz = dyaw / dt

                # Filter velocities
                vx_b, vy_b, wz = self._apply_filters(vx_b, vy_b, wz)

        if self.prev_t is not None and not (1e-6 < t_float - self.prev_t <= self.max_dt):
            for history in (self.vx_med, self.vy_med, self.wz_med):
                history.clear()
            for filt in (self.vx_ema, self.vy_ema, self.wz_ema):
                filt.reset()

        # Update previous state
        self.prev_t   = t_float
        self.prev_pos = [px, py, pz]
        self.prev_yaw = quat_to_yaw(ox, oy, oz, ow)

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.tf_parent   # 'odom'
        odom.child_frame_id  = self.tf_child    # Lite3 root link

        odom.pose.pose.position.x    = px
        odom.pose.pose.position.y    = py
        odom.pose.pose.position.z    = pz
        odom.pose.pose.orientation.x = ox
        odom.pose.pose.orientation.y = oy
        odom.pose.pose.orientation.z = oz
        odom.pose.pose.orientation.w = ow

        pose_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            pose_cov[i] = 1e-3
        odom.pose.covariance = pose_cov

        odom.twist.twist.linear.x  = float(vx_b)
        odom.twist.twist.linear.y  = float(vy_b)
        odom.twist.twist.linear.z  = float(vz_b)   # Include vertical velocity
        odom.twist.twist.angular.z = float(wz)

        twist_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            twist_cov[i] = 1e-3
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = self.tf_parent
        t.child_frame_id  = self.tf_child
        t.transform.translation.x = px
        t.transform.translation.y = py
        t.transform.translation.z = pz
        t.transform.rotation.x = ox
        t.transform.rotation.y = oy
        t.transform.rotation.z = oz
        t.transform.rotation.w = ow
        self.tf_broadcaster.sendTransform(t)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tf-child', default='TORSO')
    parser.add_argument('--tf-parent', default='odom')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--median-window', type=int, default=3)
    parser.add_argument('--alpha', type=float, default=0.3)
    parser.add_argument('--deadband-lin', type=float, default=0.01)
    parser.add_argument('--deadband-ang', type=float, default=0.01)
    parser.add_argument('--max-dt', type=float, default=0.5)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = GzToOdomTF(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
