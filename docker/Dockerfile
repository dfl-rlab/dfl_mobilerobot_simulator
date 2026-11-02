# Build from docker/ with the repository root as context, after initializing recursive submodules.
FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl wget unzip ca-certificates \
    python3-colcon-common-extensions python3-numpy python3-dev \
    libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev \
    libtbb-dev liblcm-dev libgl1-mesa-dri libgl1-mesa-glx libegl1 mesa-utils \
    xauth \
    ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-control-toolbox \
    ros-humble-gazebo-ros2-control ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher ros-humble-xacro ros-humble-joy \
    ros-humble-demo-nodes-cpp ros-humble-tf2-ros ros-humble-nav-msgs \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Keep the policy under the same path used when compiling POLICY_DIR.
WORKDIR /ws_rl
COPY upstream/rl_sar/ /ws_rl/src/rl_sar/
COPY upstream/rl_sar_zoo/ /ws_rl/src/rl_sar_zoo/
RUN bash src/rl_sar/scripts/download_inference_runtime.sh
COPY overlays/ /opt/dfl/overlays/
COPY scripts/apply_overlays.bash /opt/dfl/scripts/apply_overlays.bash
RUN bash /opt/dfl/scripts/apply_overlays.bash /ws_rl
ARG BUILD_JOBS=2
RUN source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} MAKEFLAGS=-j${BUILD_JOBS} \
       colcon build --symlink-install --executor sequential \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
# The bundled world is self-contained; no model downloads are needed at runtime.
ENV GAZEBO_MODEL_DATABASE_URI="" ROS_DOMAIN_ID=22
COPY scripts/ /opt/dfl/scripts/
COPY LICENSE /opt/dfl/LICENSE
RUN printf '\nsource /ws_rl/install/setup.bash\n' >> /root/.bashrc
ENTRYPOINT ["bash", "-c", "source /ws_rl/install/setup.bash && exec \"$@\"", "--"]
CMD ["python3", "/opt/dfl/scripts/run_simulation.py"]
