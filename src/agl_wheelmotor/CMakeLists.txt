cmake_minimum_required(VERSION 3.5)
project(agl_wheelmotor)

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
find_package(tf2_ros REQUIRED)
find_package(ageve_interfaces REQUIRED)
find_package(ageve_utils REQUIRED)
#find_package(libserial REQUIRED)



# Include files directory
include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/diff_odom.cpp"
  "src/wheelmotor.cpp"
)

# libserial
pkg_check_modules(SERIAL libserial)
target_include_directories(${PROJECT_NAME}_lib PRIVATE ${SERIAL_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}_lib 
                           rclcpp
                           std_msgs
                           sensor_msgs
                           geometry_msgs
                           nav_msgs
                           ageve_utils 
                           tf2_ros 
                           ageve_interfaces
                          )

# Main executable of project
add_executable(${PROJECT_NAME} 
              src/diff_odom.cpp
              src/wheelmotor.cpp
            )

target_link_libraries(${PROJECT_NAME} 
                      ${PROJECT_NAME}_lib
                      ${SERIAL_LIBRARIES} 
                      ${CMAKE_THREAD_LIBS_INIT}
                     )
ament_target_dependencies(${PROJECT_NAME}
                            rclcpp
                            std_msgs
                            sensor_msgs
                            geometry_msgs
                            nav_msgs
                            ageve_utils 
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
