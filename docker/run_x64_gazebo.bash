#!/bin/bash

xhost +local:docker

docker run -it \
    --privileged \
    --network=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
    --volume="/tmp:/tmp" \
    --volume="/dev:/dev" \
    --name="dddmr_humble_gazebo" \
    dddmr_gz:humble \
    bash -c "source /opt/ros/humble/setup.bash && cd /ws_gz && source install/setup.bash && ros2 launch go2_config gz_lidar_odom.launch.py"
