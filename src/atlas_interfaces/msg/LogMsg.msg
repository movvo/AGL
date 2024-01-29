# Possible states of the node
uint8 UNCONFIGURED=0
uint8 STANDBY=1
uint8 RUN=2
uint8 SHUTDOWN=3
uint8 FAULT=4

string name
uint8 state
Error error
builtin_interfaces/Time timestamp