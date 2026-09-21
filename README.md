# Go2 Gazebo Simulation

Go2 simulation for DDDMR navigation, based on [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) (`humble` branch, ROS 2 Humble and Gazebo Classic).

The Docker setup includes [a setup script](scripts/apply_go2_changes.bash) and supporting files to adapt the simulation for DDDMR navigation demos.

## Build the image

Run on the host:

```bash
cd ~
git clone --branch go2 https://github.com/dfl-rlab/dfl_mobilerobot_simulator.git
cd dfl_mobilerobot_simulator/docker
docker build -t dddmr_gz:humble -f Dockerfile_x64_gazebo .
```

## Start Gazebo

From this repository's `docker` directory:

```bash
./run_x64_gazebo.bash
```

This creates `dddmr_humble_gazebo` and automatically starts `go2_config/gz_lidar_odom.launch.py`. Keep the terminal open. Ensure the simulation and navigation containers use the same `ROS_DOMAIN_ID`; the script uses the host's value, or `0` when unset.
