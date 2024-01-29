# Standard ROS2 header
std_msgs/Header header 

# Types of navigation modes
uint8 IDLE=0
uint8 NAVIGATION=1
uint8 MAPPING=2
uint8 SLAM=3

uint8 mode
string map_name
string description
