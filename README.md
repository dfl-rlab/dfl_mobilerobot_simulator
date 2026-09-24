# Ackermann Gazebo Simulation

Explore DDDMR navigation with the [Saye Ackermann](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2) (Ali, alitekes1) simulation, using ROS 2 Jazzy and Gazebo Harmonic. This repository provides the Docker setup and supporting integration to get the simulation ready for navigation demos.

## Build the image

With Docker installed, run these commands on the host:

```bash
cd ~
git clone --branch ackermann https://github.com/dfl-rlab/dfl_mobilerobot_simulator.git
cd dfl_mobilerobot_simulator
git submodule update --init --recursive
cd docker
docker build -t dddmr_simulation:ackermann -f Dockerfile_po_builtin .
```

The image includes the simulation and a compiled ROS workspace, ready to launch.

## Start Gazebo

Run on the host:

```bash
cd ~/dfl_mobilerobot_simulator/docker
./ackermann_bring_up.bash
```

The script creates the `dddmr_ackermann` container and starts the Saye robot in Gazebo, together with the feedback needed for DDDMR navigation. Keep this terminal open while navigating.

> [!IMPORTANT]
> The simulation uses `ROS_DOMAIN_ID=14`. Make sure the navigation container uses the same domain ID before launching navigation.

## Run navigation

Open another terminal and follow the [DDDMR Ackermann navigation tutorial](https://github.com/dfl-rlab/dddmr_navigation/blob/main/src/dddmr_p2p_move_base/kinematics_md/ACKERMANN.md) to prepare the map, start the navigation stack, and run the P2P test.

## Acknowledgements

This project builds on the excellent work from [ackermann-vehicle-gzsim-ros2](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2) (Ali, alitekes1).
We are grateful to the original author for integrating the Saye Ackermann vehicle with ROS 2 and Gazebo.
The original authors deserve full credit for the core simulation — our contribution adds support for DDDMR navigation demos.
