# Build from docker/ with the repository root as context, after initializing recursive submodules.
FROM ros:jazzy-ros-base-noble AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl wget unzip ca-certificates \
    python3-colcon-common-extensions python3-numpy python3-dev \
    libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev \
    libtbb-dev liblcm-dev libgl1-mesa-dri libgl1 libglx-mesa0 libegl1 mesa-utils \
    xauth \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-control-toolbox \
    ros-jazzy-gz-ros2-control ros-jazzy-ros-gz \
    ros-jazzy-gz-rendering-vendor ros-jazzy-gz-sim-vendor ros-jazzy-gz-sensors-vendor \
    ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-joy \
    ros-jazzy-demo-nodes-cpp ros-jazzy-tf2-ros ros-jazzy-nav-msgs \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

FROM dependencies AS renderer
ARG BUILD_JOBS=2
# Harmonic normally caps native GPU LiDAR cube faces at 1024 pixels. This
# optional override preserves the Dome LiDAR's configured 2048-pixel faces.
COPY docker/patches/gz-rendering8-cubemap.patch /tmp/cubemap.patch
RUN curl -fsSL https://github.com/gazebosim/gz-rendering/archive/refs/tags/gz-rendering8_8.2.3.tar.gz -o /tmp/rendering.tar.gz \
    && echo "5bfe18d164414e098b2ba6ef81bf9391f7ff8f2c39b8a4fa9f8543f778e921c8  /tmp/rendering.tar.gz" | sha256sum -c - \
    && tar -xzf /tmp/rendering.tar.gz -C /tmp \
    && cd /tmp/gz-rendering-gz-rendering8_8.2.3 && patch -p1 < /tmp/cubemap.patch \
    && source /opt/ros/jazzy/setup.bash \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
       -DSKIP_ogre=ON -DCMAKE_PREFIX_PATH="$(printf '%s;' /opt/ros/jazzy/opt/*_vendor)" \
       -DCMAKE_INSTALL_PREFIX=/opt/ros/jazzy/opt/gz_rendering_vendor \
    && cmake --build build --target gz-rendering8-ogre2 -j${BUILD_JOBS} \
    && mkdir /renderer-output \
    && cp build/lib/libgz-rendering8-ogre2.so.8.2.3 /renderer-output/ \
    && cp LICENSE /renderer-output/LICENSE.gz-rendering

FROM dependencies AS simulator
COPY --from=renderer /renderer-output/libgz-rendering8-ogre2.so.8.2.3 /opt/ros/jazzy/opt/gz_rendering_vendor/lib/
COPY --from=renderer /renderer-output/libgz-rendering8-ogre2.so.8.2.3 /opt/ros/jazzy/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins/
COPY --from=renderer /renderer-output/LICENSE.gz-rendering /opt/dfl/LICENSE.gz-rendering

# Keep the policy under the same path used when compiling POLICY_DIR.
WORKDIR /ws_rl
COPY upstream/rl_sar/ /ws_rl/src/rl_sar/
COPY upstream/rl_sar_zoo/ /ws_rl/src/rl_sar_zoo/
RUN bash src/rl_sar/scripts/download_inference_runtime.sh
COPY overlays/ /opt/dfl/overlays/
COPY scripts/apply_overlays.bash /opt/dfl/scripts/apply_overlays.bash
RUN bash /opt/dfl/scripts/apply_overlays.bash /ws_rl
ARG BUILD_JOBS=2
RUN source /opt/ros/jazzy/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} MAKEFLAGS=-j${BUILD_JOBS} \
       colcon build --symlink-install --executor sequential \
       --packages-up-to rl_sar lite3_description robot_sensors_description \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
# The bundled world is self-contained; no model downloads are needed at runtime.
ENV ROS_DOMAIN_ID=23 GZ_PARTITION=rl_lite3_jazzy
COPY scripts/ /opt/dfl/scripts/
COPY LICENSE /opt/dfl/LICENSE
RUN printf '\nsource /ws_rl/install/setup.bash\n' >> /root/.bashrc
ENTRYPOINT ["bash", "-c", "source /ws_rl/install/setup.bash && exec \"$@\"", "--"]
CMD ["python3", "/opt/dfl/scripts/run_simulation.py"]
