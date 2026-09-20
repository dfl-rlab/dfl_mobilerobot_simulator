# Ackermann Simulation Plugin

Additional integration for DDDMR navigation with the [Saye Gazebo simulation](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2). The upstream repository is cloned unchanged during the Docker image build. This repository contains the additional feedback node, joint-state bridge, launch integration, and Docker setup.

## Build the image

Run on the host:

```bash
cd ~
git clone --branch ackermann https://github.com/dfl-rlab/dfl_mobilerobot_simulator.git
cd dfl_mobilerobot_simulator/docker
docker build -t dddmr_simulation:ackermann -f Dockerfile_po_builtin .
```

The image uses ROS 2 Jazzy and Gazebo Harmonic. Its Dockerfile clones both repositories into `/ws_ackermann/src` and builds them together:
有修改原碼」
```text
/ws_ackermann/src/
├── ackermann-vehicle-gzsim-ros2/  # Upstream source
└── dfl_mobilerobot_simulator/
    ├── utils/
    └── ackermann_plugin_bringup/
```

## Start Gazebo

From this repository's `docker` directory:

```bash
./ackermann_bring_up.bash
```

This creates the `dddmr_ackermann` container and starts Gazebo through `ackermann_plugin_bringup/ackermann_sim.launch.py`. The launch file includes the original `saye_bringup/saye_spawn.launch.py` with `rviz:=false`, adds a separate Gazebo-to-ROS `/joint_states` bridge, and starts `utils/joint_to_ackermann` to publish `/ackermann_feedback` from `/joint_states` and `/odom`.

Keep the terminal open while navigating. The startup script sets `ROS_DOMAIN_ID=14`; ensure the navigation container uses the same domain ID.
