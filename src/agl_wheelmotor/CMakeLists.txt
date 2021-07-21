cmake_minimum_required(VERSION 3.5)
project(me00_wheelmotor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ageve_interfaces REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)


# Include files directory
include_directories(
  include
)

# ODVA EtherNet/IP
add_library(odva_eip_lib STATIC
            src/odva_ethernetip/connection.cpp
            src/odva_ethernetip/copy_serializable.cpp
            src/odva_ethernetip/cpf_item.cpp
            src/odva_ethernetip/cpf_packet.cpp
            src/odva_ethernetip/encap_header.cpp
            src/odva_ethernetip/encap_packet.cpp
            src/odva_ethernetip/identity_item_data.cpp
            src/odva_ethernetip/io_scanner.cpp
            src/odva_ethernetip/message_router_response.cpp
            src/odva_ethernetip/path.cpp
            src/odva_ethernetip/rr_data.cpp
            src/odva_ethernetip/session.cpp
          )
target_link_libraries(odva_eip_lib)
ament_target_dependencies(odva_eip_lib
                          rclcpp 
                          std_msgs
                          ageve_interfaces
                        )

# Main executable of project
add_executable(${PROJECT_NAME} 
              src/main.cpp
              src/roboteq_motor_controller_driver_node.cpp
              src/diff_odom.cpp
              src/controller.cpp
              src/controller_eip.cpp
            )
target_link_libraries(${PROJECT_NAME} odva_eip_lib)
ament_target_dependencies(${PROJECT_NAME}
                           rclcpp
                           std_msgs
                           sensor_msgs
                           geometry_msgs
                           nav_msgs
                           rclpy 
                           tf2_ros 
                           ageve_interfaces
                          )

# Install
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
