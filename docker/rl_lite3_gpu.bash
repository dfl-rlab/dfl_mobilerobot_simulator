#!/usr/bin/env bash
# Recommended when a GPU is available.
set -euo pipefail
: "${DISPLAY:?Run this script from a graphical desktop.}"
xauth_file="${XAUTHORITY:-$HOME/.Xauthority}"
[[ -r "$xauth_file" ]] || { echo "Xauthority file not found: $xauth_file" >&2; exit 1; }
if command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
    graphics=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,display
        -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia)
elif [[ -d /dev/dri ]]; then
    graphics=(--device /dev/dri)
else
    echo 'No GPU detected. Hardware OpenGL support is required.' >&2
    exit 1
fi

# Load the repository profile by default; SIM_RTPS_PROFILE can override it.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
rtps_profile="${SIM_RTPS_PROFILE:-$script_dir/../rtps_lidar_shm_profile.xml}"
[[ -f "$rtps_profile" && -r "$rtps_profile" ]] || {
    echo "RTPS profile is not a readable file: $rtps_profile" >&2
    exit 1
}
rtps_profile="$(realpath -- "$rtps_profile")"
rtps_args=(-e RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/opt/dfl/rtps_profile.xml
    -e RMW_FASTRTPS_USE_QOS_FROM_XML=1
    -v "$rtps_profile:/opt/dfl/rtps_profile.xml:ro")

exec docker run --rm -it --init --network host --ipc host \
    --name "${SIM_CONTAINER:-rl_lite3}" \
    -e DISPLAY -e QT_X11_NO_MITSHM=1 -e ROS_DOMAIN_ID=22 \
    -e XAUTHORITY=/tmp/dfl.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$xauth_file:/tmp/dfl.xauth:ro" \
    "${graphics[@]}" \
    "${rtps_args[@]}" \
    "${SIM_IMAGE:-dddmr_simulation:rl_lite3}" \
    python3 /opt/dfl/scripts/run_simulation.py --backend gpu "$@"
