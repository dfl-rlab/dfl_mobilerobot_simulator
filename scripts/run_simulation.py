#!/usr/bin/env python3
"""Supervise Gazebo, odometry and the foreground rl_sar keyboard controller."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import termios
import time


def check_graphics(backend):
    result = subprocess.run(['glxinfo', '-B'], capture_output=True, text=True, timeout=20)
    details = result.stdout + result.stderr
    if result.returncode:
        raise RuntimeError('OpenGL/display check failed. Check DISPLAY and Xauthority.\n' + details)
    software = any(name in details.lower() for name in ('llvmpipe', 'softpipe', 'swrast'))
    if backend == 'gpu' and software:
        raise RuntimeError('OpenGL uses software rendering. Configure hardware GPU access to run this simulator.')
    renderer = next((line.strip() for line in details.splitlines() if 'renderer string' in line), details)
    print(f'{renderer}\nDome LiDAR backend: {backend}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['gpu', 'cpu'], default='gpu')
    parser.add_argument('--startup-timeout', type=float, default=120.0)
    parser.add_argument('launch_args', nargs='*', help='Additional name:=value launch arguments')
    args = parser.parse_args()
    mappings = dict(item.split(':=', 1) for item in args.launch_args if ':=' in item)
    if len(mappings) != len(args.launch_args):
        parser.error('Launch arguments must be unique name:=value pairs.')
    if any(name in mappings for name in ('gui', 'dome_lidar_backend')):
        parser.error('Use --backend to select GPU or CPU LiDAR; the GUI is always enabled.')
    if mappings.get('rname', 'lite3') != 'lite3':
        parser.error('This integrated launcher currently supports Lite3; use upstream launches for other robots.')
    if not sys.stdin.isatty():
        parser.error('An interactive terminal is required for r / 0 / 1 / n. Use docker run -it.')
    if not os.environ.get('DISPLAY'):
        parser.error('Run from a graphical desktop with DISPLAY set.')
    return run(args, mappings)


def run(args, mappings):
    check_graphics(args.backend)
    if args.backend == 'cpu':
        mappings.setdefault('dome_lidar_horizontal_samples', '360')
        mappings.setdefault('dome_lidar_vertical_samples', '64')
    launch_args = [f'{name}:={value}' for name, value in mappings.items()]

    import rclpy
    from ament_index_python.packages import get_package_prefix
    from nav_msgs.msg import Odometry
    from controller_manager_msgs.srv import ListControllers
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    log_dir = Path(os.environ.get('SIM_LOG_DIR', '/tmp/rl_lite3'))
    log_dir.mkdir(parents=True, exist_ok=True)
    children = []
    logs = []
    terminal_state = termios.tcgetattr(sys.stdin)
    stopped = False

    def stop(_signum=None, _frame=None):
        nonlocal stopped
        stopped = True

    def start(name, command, interactive=False):
        stream = None
        if not interactive:
            stream = (log_dir / f'{name}.log').open('w')
            logs.append(stream)
        process = subprocess.Popen(command, stdin=None if interactive else subprocess.DEVNULL,
                                   stdout=stream, stderr=None if interactive else subprocess.STDOUT,
                                   start_new_session=True)
        children.append((name, process))
        return process

    def check_children():
        for name, process in children:
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited ({process.returncode}). See {log_dir}/{name}.log')

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = None
    try:
        print(f'Logs: {log_dir}\nWaiting for Gazebo, odometry and LiDAR...', flush=True)
        start('gazebo', ['ros2', 'launch', 'rl_sar', 'gazebo.launch.py', 'rname:=lite3',
                         f'dome_lidar_backend:={args.backend}', *launch_args])
        start('odom', [sys.executable, str(Path(__file__).with_name('gz_odom.py')),
                       '--tf-child', 'TORSO'])
        # Do not start the controller until the model and feedback have appeared.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('dfl_simulation_startup')
        seen = set()
        subscriptions = [node.create_subscription(Odometry, '/odom', lambda _: seen.add('odom'), 10)]
        lidar_enabled = mappings.get('dome_lidar_enabled', 'true').lower() in ('true', '1')
        if lidar_enabled:
            subscriptions.append(node.create_subscription(
                PointCloud2, mappings.get('dome_lidar_topic', '/dome_lidar/points'),
                lambda _: seen.add('lidar'), qos_profile_sensor_data))
        expected = {'odom', 'lidar'} if lidar_enabled else {'odom'}
        controller_client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        controller_status = None
        deadline = time.monotonic() + args.startup_timeout
        while not stopped:
            check_children()
            rclpy.spin_once(node, timeout_sec=0.1)
            services = {name for name, _ in node.get_service_names_and_types()}
            if expected <= seen and '/param_node/get_parameters' in services and controller_client.service_is_ready():
                if controller_status is None:
                    controller_status = controller_client.call_async(ListControllers.Request())
                elif controller_status.done():
                    response = controller_status.result()
                    if response and any(c.name == 'joint_state_broadcaster' and c.state == 'active'
                                        for c in response.controller):
                        break
                    controller_status = None
            if time.monotonic() > deadline:
                raise RuntimeError(f'Startup timed out; received {sorted(seen)}. Check {log_dir}.')
        node.destroy_node()
        node = None
        rclpy.shutdown()
        if stopped:
            return 130
        print('\nReady. Focus this terminal: r reset | 0 stand (wait) | 1 policy | n navigation ON\n'
              'Navigation sends /cmd_vel. Ctrl-C stops all simulation processes.\n', flush=True)
        controller = Path(get_package_prefix('rl_sar')) / 'lib/rl_sar/rl_sim'
        start('controller', [str(controller)], interactive=True)
        while not stopped:
            check_children()
            time.sleep(0.2)
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for _, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 10
        for _, process in reversed(children):
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()
        termios.tcsetattr(sys.stdin, termios.TCSANOW, terminal_state)
        for stream in logs:
            stream.close()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, subprocess.SubprocessError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        sys.exit(1)
