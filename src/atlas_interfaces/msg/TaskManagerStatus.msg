# Standard ROS2 header
std_msgs/Header header 

# Types of TM modes
uint8 IDLE=0
uint8 MAPPING=1
uint8 EXECUTE_MISSION=2
uint8 CREATE_MISSION=3

uint8 state
int8 mission_id