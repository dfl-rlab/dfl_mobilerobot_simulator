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

exec docker run --rm -it --init --network host --ipc host \
    --name "${SIM_CONTAINER:-rl_lite3}" \
    -e DISPLAY -e QT_X11_NO_MITSHM=1 -e ROS_DOMAIN_ID=22 \
    -e XAUTHORITY=/tmp/dfl.xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$xauth_file:/tmp/dfl.xauth:ro" \
    "${graphics[@]}" \
    "${SIM_IMAGE:-dddmr_simulation:rl_lite3}" \
    python3 /opt/dfl/scripts/run_simulation.py --backend gpu "$@"
