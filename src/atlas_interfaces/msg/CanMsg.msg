# This message re-publish the CAN msgs received  

# Standard ROS2 header
std_msgs/Header header 

uint32 id
uint8 size
builtin_interfaces/Time can_stamp
uint8[] data