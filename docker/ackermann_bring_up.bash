#!/bin/bash
set -euo pipefail

# Load the repository profile by default; SIM_RTPS_PROFILE can override it.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
rtps_profile="${SIM_RTPS_PROFILE:-$script_dir/../rtps_lidar_shm_profile.xml}"
[[ -f "$rtps_profile" && -r "$rtps_profile" ]] || {
    echo "RTPS profile is not a readable file: $rtps_profile" >&2
    exit 1
}
rtps_profile="$(realpath -- "$rtps_profile")"
rtps_args=(
    --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    --env="FASTRTPS_DEFAULT_PROFILES_FILE=/opt/dfl/rtps_profile.xml"
    --env="RMW_FASTRTPS_USE_QOS_FROM_XML=1"
    --volume="$rtps_profile:/opt/dfl/rtps_profile.xml:ro"
)

xhost +local:docker

docker run -it \
    --privileged \
    --network=host \
    --ipc=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=14" \
    --volume="/tmp:/tmp" \
    --volume="/dev:/dev" \
    --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
    --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
    "${rtps_args[@]}" \
    --name="dddmr_ackermann" \
    dddmr_simulation:ackermann \
    bash -c "cd /ws_ackermann && source install/setup.bash && ros2 launch ackermann_plugin_bringup ackermann_sim.launch.py"
