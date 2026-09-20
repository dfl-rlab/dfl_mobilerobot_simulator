#!/bin/bash

xhost +local:docker

docker run -it \
    --privileged \
    --network=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=14" \
    --volume="/tmp:/tmp" \
    --volume="/dev:/dev" \
    --volume="${HOME}/dddmr_navigation:/root/dddmr_navigation" \
    --volume="${HOME}/dddmr_bags:/root/dddmr_bags" \
    --name="dddmr_ackermann" \
    dddmr_simulation:ackermann \
    bash -c "cd /ws_ackermann && source install/setup.bash && ros2 launch ackermann_plugin_bringup ackermann_sim.launch.py"
