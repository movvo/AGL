
std_msgs/Header header #Standard ROS2 header

# Possible states of the node
uint16 UNCONFIGURED=0
uint16 STANDBY=1
uint16 RUN=2
uint16 SHUTDOWN=3
uint16 FAULT=4

uint16 state # State of the Node

# This field is only used in the Diagnostics pkg
uint16[] nodes_states

Error error # Formado por tres int32 
