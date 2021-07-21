cmake_minimum_required(VERSION 3.5)
project(me00_joystick)

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

#=====================================
#             Find dependencies
#=====================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ageve_interfaces REQUIRED)
find_package(ageve_utils REQUIRED)

#=====================================
#          Build and Add executables
#=====================================
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/joystick.cpp" 
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib 
                          rclcpp 
                          geometry_msgs
                          sensor_msgs
                          ageve_interfaces
                          ageve_utils
                        )

add_executable(${PROJECT_NAME} src/joystick.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}
                          rclcpp 
                          geometry_msgs
                          sensor_msgs
                          ageve_interfaces
                          ageve_utils
                        )

#=====================================
#             Install
#=====================================
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})   

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)                   

ament_package()