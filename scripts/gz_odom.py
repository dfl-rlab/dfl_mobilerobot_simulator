#!/usr/bin/env python3

import argparse
import math
import re
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time as TimeMsg


# ─── Math helpers ──────────────────────────────────────────────────

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


# ─── ROS node ─────────────────────────────────────────────────────────

class GzToOdomTF(Node):
    def __init__(self, entity_name, gz_topic, odom_topic, tf_parent, tf_child,
                 idle_timeout, max_backoff, median_window, alpha,
                 deadband_lin, deadband_ang, max_dt):
        super().__init__('gz_to_odom_tf_filtered')

        self.entity_name  = entity_name
        self.gz_topic     = gz_topic

        self.odom_pub      = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_parent  = tf_parent   # 'odom'
        self.tf_child   = tf_child    # Lite3 root link

        self.idle_timeout  = float(idle_timeout)
        self.max_backoff   = float(max_backoff)
        self.max_dt        = float(max_dt)
        self.median_window = max(1, int(median_window))
        self.alpha         = float(alpha)
        self.deadband_lin  = float(deadband_lin)
        self.deadband_ang  = float(deadband_ang)

        self.proc         = None
        self.running      = True
        self.last_line_ts = 0.0
        self.backoff      = 1.0

        self.float_re = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')
        self.name_re  = re.compile(r'name:\s*"([^"]+)"')

        # ── parser state ──
        self.current_sec  = None
        self.current_nsec = None
        self.in_time      = False

        self.in_pose        = False
        self.in_position    = False
        self.in_orientation = False
        self.pose_name      = None
        self.pos  = {'x': None, 'y': None, 'z': None}
        self.ori  = {'x': None, 'y': None, 'z': None, 'w': None}

        # ── velocity state ──
        self.prev_t   = None
        self.prev_pos = None
        self.prev_yaw = None

        self.vx_med = deque(maxlen=self.median_window)
        self.vy_med = deque(maxlen=self.median_window)
        self.wz_med = deque(maxlen=self.median_window)

        self.vx_ema = OnePoleEMA(self.alpha)
        self.vy_ema = OnePoleEMA(self.alpha)
        self.wz_ema = OnePoleEMA(self.alpha)

        self._spawn_proc()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.stderr_thread = threading.Thread(target=self._stderr_loop, daemon=True)
        self.stderr_thread.start()
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f"[gz_odom] entity='{self.entity_name}'  gz_topic='{self.gz_topic}'\n"
            f"          odom → '{odom_topic}'   TF: {tf_parent} → {tf_child}\n"
            f"          filters: median={self.median_window}  alpha={self.alpha}  "
            f"deadband_lin={self.deadband_lin}  deadband_ang={self.deadband_ang}"
        )

    # ── process management ──────────────────────────────────────────

    def _spawn_proc(self):
        self._kill_proc()
        try:
            self.proc = subprocess.Popen(
                ['gz', 'topic', '-e', self.gz_topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=1,
                text=True,
            )
            self.last_line_ts = time.time()
            self.get_logger().info(f"[gz_odom] Started: gz topic -e {self.gz_topic}")
        except Exception as e:
            self.get_logger().error(f"[gz_odom] Failed to start gz topic: {e}")
            self.proc = None

    def _kill_proc(self):
        if self.proc is not None and self.proc.poll() is None:
            try:
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
            except Exception:
                pass
        self.proc = None

    def _watchdog(self):
        now = time.time()
        if self.proc is None or self.proc.poll() is not None:
            self.get_logger().warn("[gz_odom] gz topic process not running — restarting...")
            time.sleep(self.backoff)
            self._spawn_proc()
            self.backoff = min(self.backoff * 2.0, self.max_backoff)
            return
        if now - self.last_line_ts > self.idle_timeout:
            self.get_logger().warn(
                f"[gz_odom] No data for {now - self.last_line_ts:.1f}s — restarting gz..."
            )
            self._spawn_proc()
            self.backoff = min(self.backoff * 2.0, self.max_backoff)
        else:
            self.backoff = 1.0

    # ── pose block helpers ──────────────────────────────────────────

    def _reset_pose_block(self):
        self.in_pose = self.in_position = self.in_orientation = False
        self.pose_name = None
        self.pos = {'x': None, 'y': None, 'z': None}
        self.ori = {'x': None, 'y': None, 'z': None, 'w': None}

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

        # ── timestamp ──
        if self.current_sec is not None and self.current_nsec is not None:
            stamp   = TimeMsg(sec=int(self.current_sec), nanosec=int(self.current_nsec))
            t_float = float(self.current_sec) + float(self.current_nsec) * 1e-9
        else:
            now_ros = self.get_clock().now()
            stamp   = now_ros.to_msg()
            t_float = now_ros.nanoseconds * 1e-9

        # ── pose ──
        px = float(self.pos['x'])
        py = float(self.pos['y'])
        pz = float(self.pos['z'])
        ox, oy, oz, ow = quat_normalize(
            float(self.ori['x']), float(self.ori['y']),
            float(self.ori['z']), float(self.ori['w'])
        )

        # ── velocity (finite differences in the body frame) ──
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

        # ── Publish odometry ──
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

        # ── TF broadcast: odom → robot root ──
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

    # ── reader thread ───────────────────────────────────────────────

    def _reader_loop(self):
        while self.running:
            if self.proc is None:
                time.sleep(0.1);  continue
            try:
                line = self.proc.stdout.readline()
                if line == '':
                    time.sleep(0.05);  continue
                self.last_line_ts = time.time()
                s = line.strip()
                if not s:
                    continue

                # ── time block ──
                if s.startswith('time {'):
                    self.in_time = True
                    self.current_sec = self.current_nsec = None
                    continue
                if self.in_time:
                    if s.startswith('sec:'):
                        m = self.float_re.search(s)
                        if m: self.current_sec = int(float(m.group(0)))
                    elif s.startswith('nsec:'):
                        m = self.float_re.search(s)
                        if m: self.current_nsec = int(float(m.group(0)))
                    elif s == '}':
                        self.in_time = False
                    continue

                # ── pose block ──
                if s.startswith('pose {'):
                    self._reset_pose_block()
                    self.in_pose = True;  continue

                if self.in_pose:
                    if s.startswith('name:'):
                        nm = self.name_re.search(s)
                        if nm: self.pose_name = nm.group(1)
                        continue

                    if s.startswith('position {'):
                        self.in_position = True;  continue
                    if s.startswith('orientation {'):
                        self.in_orientation = True;  continue

                    if self.in_position:
                        for k in ('x', 'y', 'z'):
                            if s.startswith(f'{k}:'):
                                m = self.float_re.search(s)
                                self.pos[k] = m.group(0) if m else None
                                break
                        if s == '}':
                            self.in_position = False
                        continue

                    if self.in_orientation:
                        for k in ('x', 'y', 'z', 'w'):
                            if s.startswith(f'{k}:'):
                                m = self.float_re.search(s)
                                self.ori[k] = m.group(0) if m else None
                                break
                        if s == '}':
                            self.in_orientation = False
                        continue

                    if s == '}':
                        self._maybe_publish()
                        self._reset_pose_block()

            except Exception as e:
                self.get_logger().error(f"reader error: {e}")
                time.sleep(0.2)

    def _stderr_loop(self):
        while self.running:
            if self.proc is None:
                time.sleep(0.1);  continue
            try:
                err = self.proc.stderr.readline()
                if err == '':
                    time.sleep(0.05);  continue
                self.get_logger().warn(f"[gz stderr] {err.rstrip()}")
            except Exception:
                time.sleep(0.2)

    def shutdown(self):
        self.running = False
        self._kill_proc()
        self.get_logger().info("[gz_odom] Node shut down.")


# ─── main ────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Gazebo pose topic → ROS2 Odometry + TF (standalone)'
    )
    # ── Defaults for the bundled terrain ──
    parser.add_argument('--entity',        default='robot_model',
                        help='Gazebo entity/model name (spawn_entity -entity name)')
    parser.add_argument('--gz-topic',      default='/gazebo/complex_terrain_v6/pose/info',
                        help='gz topic to echo')
    parser.add_argument('--odom-topic',    default='/odom')
    parser.add_argument('--tf-parent',     default='odom')
    parser.add_argument('--tf-child',      default='TORSO',   # Lite3 root link
                        help='Robot root link in URDF')
    parser.add_argument('--idle-timeout',  type=float, default=5.0)
    parser.add_argument('--max-backoff',   type=float, default=10.0)
    parser.add_argument('--median-window', type=int,   default=3)
    parser.add_argument('--alpha',         type=float, default=0.3,
                        help='EMA smoothing 0..1 (higher values respond faster with less smoothing)')
    parser.add_argument('--deadband-lin',  type=float, default=0.01,
                        help='Linear speeds below this value are set to zero (m/s)')
    parser.add_argument('--deadband-ang',  type=float, default=0.01,
                        help='Angular speeds below this value are set to zero (rad/s)')
    parser.add_argument('--max-dt',        type=float, default=0.5,
                        help='Skip velocity estimation when dt exceeds this value (seconds)')
    args = parser.parse_args()

    rclpy.init()
    node = GzToOdomTF(
        entity_name    = args.entity,
        gz_topic       = args.gz_topic,
        odom_topic     = args.odom_topic,
        tf_parent      = args.tf_parent,
        tf_child       = args.tf_child,
        idle_timeout   = args.idle_timeout,
        max_backoff    = args.max_backoff,
        median_window  = args.median_window,
        alpha          = args.alpha,
        deadband_lin   = args.deadband_lin,
        deadband_ang   = args.deadband_ang,
        max_dt         = args.max_dt,
    )

    def _sig_handler(signum, frame):
        node.get_logger().info("Caught signal, shutting down...")
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT,  _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _sig_handler(None, None)


if __name__ == '__main__':
    main()
