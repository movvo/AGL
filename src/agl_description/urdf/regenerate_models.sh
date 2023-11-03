#!/bin/bash
ros2 run xacro xacro andresito.xacro > andresito.urdf
gz sdf -p andresito.urdf > ../model/andresito/model.sdf