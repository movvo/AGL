#!/bin/bash

docker run -it --rm \
    --privileged \
    --volume=/dev/input:/dev/input \
    --volume=$(dirname $(dirname $(realpath $0)))/config:/ros2_ws/src/ast_joystick/config:rw \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --env=RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
    --net=host \
    --name ast_joystick \
    ast_joystick:latest \
    ros2 launch ast_joystick launch.py