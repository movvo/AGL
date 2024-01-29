cmake_minimum_required(VERSION 3.5)
project(atlas_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#SET(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

#SET(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH} /opt/ros/humble")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


set(msg_files
  "msg/LaserDriverHeader.msg"
  "msg/ChannelValues.msg"
  "msg/Diagnostics.msg"
  "msg/Error.msg"
  "msg/Control.msg"
  "msg/DigitalInputs.msg"
  "msg/DigitalOutputs.msg"
  "msg/ErrorsLog.msg"
  "msg/LogMsg.msg"
  "msg/NavigationStatus.msg"
  "msg/CanMsg.msg"
  "msg/CanMsgs.msg"
  "msg/RobotInfo.msg"
  "msg/Heartbeat.msg"
  "msg/TaskManagerStatus.msg"
  "msg/Point.msg"
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  "msg/Action.msg"
  "msg/Map.msg"
  "msg/Mission.msg"
  "msg/MissionId.msg"
  "msg/Waypoint.msg"
  "msg/WaypointActions.msg"
)
set(srv_files
  "srv/Heartbeat.srv"
  "srv/LoadMap.srv"
  "srv/Maintenance.srv"
  "srv/YamlTransfer.srv"
  "srv/ToggleState.srv"
  "srv/StartMapping.srv"
  "srv/SwitchNavigationMode.srv"
  "srv/RunMission.srv"
  "srv/StringBool.srv"
  "srv/FollowPath.srv"
  "srv/SaveWaypoint.srv"
  "srv/AssignAction.srv"
  "srv/CreateMission.srv"
  "srv/GetDiagnostics.srv"
  "srv/GetMissions.srv"
  "srv/GetActions.srv"
  "srv/StatusEmergency.srv"
  "srv/GetMaps.srv"
  "srv/DeleteMap.srv"
  "srv/LinkAction.srv"
  "srv/StatusMission.srv"
  "srv/GetWaypoints.srv"
  "srv/DeleteWaypoint.srv"
  "srv/GetMapImage.srv"
  "srv/Rosbag.srv"
  "srv/GetNavigationMode.srv"
  "srv/GetPose.srv"
  "srv/Navigate.srv"
  "srv/StartProcess.srv"
  "srv/ProcessCompleted.srv"
)

set(action_files
  "action/Backward.action"
  "action/Forward.action"
  "action/Sound.action"
  "action/Docking.action"
  "action/ActionManager.action"
  "action/Undocking.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
 )


if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
