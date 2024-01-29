# This message publishes the state of the Control module

# Standard ROS2 header
std_msgs/Header header 

# States of navigation
uint8 MANUAL     = 0
uint8 AUTONOMOUS = 1
uint8 PAUSE      = 2
uint8 UNKNOWN    = 3

uint8 mode

string message