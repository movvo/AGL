#!/bin/bash 

docker run -it --rm  \
    --volume=$(dirname $(dirname $(realpath $0))):/ros2_ws/src/ast_joystick:rw \
    --privileged \
    --volume=/dev/input:/dev/input \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --env=RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
    --net=host \
    ast_joystick