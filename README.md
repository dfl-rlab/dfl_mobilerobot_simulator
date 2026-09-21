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

## Acknowledgements

This project builds on the excellent work from [Unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2).
We are grateful to the original author for integrating the Unitree Go2 robot with ROS 2 and Gazebo.
The original authors deserve full credit for the core simulation — our contribution adds support for DDDMR navigation demos.

As acknowledged in that project, credits also go to the following upstream works:

- [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/go2_description) — For the Go2 robot description (URDF model).
- [CHAMP](https://github.com/chvmp/champ) — For the quadruped controller framework.
- [CHAMP Robots](https://github.com/chvmp/robots) — For robot configurations and setup examples.
