# rl_lite3

A ROS 2 Humble / Gazebo Classic simulator with a **Lite3 RL locomotion policy**,
**dome LiDAR**, odometry and a multi-level terrain. Built on
[rl_sar](https://github.com/fan-ziqi/rl_sar) by **Ziqi Fan (fan-ziqi)** and its
contributors, using the pretrained Lite3 HIMLoco policy.

## Build the image

On a Linux x86-64 host with Docker:

```bash
cd ~
git clone --branch rl_lite3 --recurse-submodules \
  https://github.com/dfl-rlab/dfl_mobilerobot_simulator.git
cd dfl_mobilerobot_simulator/docker
docker build -t dddmr_simulation:rl_lite3 -f Dockerfile ..
```

The Docker image includes the complete simulation workspace.

## Start Gazebo

**Prefer GPU mode when a GPU is available; CPU ray casting runs much slower.**

Run from the same `docker/` directory on a graphical desktop:

```bash
# Recommended when a GPU is available (NVIDIA requires NVIDIA Container Toolkit).
./rl_lite3_gpu.bash

# CPU ray casting, without GPU access.
./rl_lite3_cpu.bash
```

Both scripts use the same image and start Lite3, Gazebo, the policy controller,
dome LiDAR and odometry. They select GPU rendering or CPU ray casting for the
LiDAR. GPU mode uses a 900 x 128 scan; CPU mode uses a reduced 360 x 64 scan
but still runs much slower. Use GPU mode whenever possible.

Both launchers always use `ROS_DOMAIN_ID=22`, regardless of the host terminal
environment. The image also defaults to domain `22`.

After the robot drops onto the ground, keep this terminal focused: press `r`, wait 1 second, press `0`, wait 1 second, press `1`, wait 1 second, then press `n`.

| Key | Action |
| --- | --- |
| `r` | Reset the world pose |
| `0` | Stand up; wait until complete |
| `1` | Enter policy locomotion |
| `n` | Toggle `/cmd_vel` input |

Press Ctrl-C to stop the simulation.

## Acknowledgements

This project builds on the excellent work of **Ziqi Fan (fan-ziqi)** and the
[rl_sar](https://github.com/fan-ziqi/rl_sar) contributors, together with the robot
models in [rl_sar_zoo](https://github.com/fan-ziqi/rl_sar_zoo).
The original authors deserve full credit for the core controller, robot models
and pretrained policies; our contribution adds the dome LiDAR, terrain,
odometry and Docker integration.

Thanks also to **Yuxuan Ma** for the upstream Lite3 integration, **Deep Robotics**
for Lite3 and its SDK, and the **HIMLoco** authors. Original copyright notices
and licenses are retained in the upstream submodules.

Please use the citation requested by rl_sar when using this code or parts of it:

```bibtex
@software{fan-ziqi2024rl_sar,
  author = {fan-ziqi},
  title = {rl_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.},
  url = {https://github.com/fan-ziqi/rl_sar},
  year = {2024}
}
```
