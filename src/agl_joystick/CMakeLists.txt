cmake_minimum_required(VERSION 3.5)
project(agl_joystick)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lgtest -lgmock -g)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  test/include
)

# Add cpp libraries
add_library(${PROJECT_NAME}_lib
  "src/joystick.cpp"
)

target_link_libraries(${PROJECT_NAME}_lib
                        stdc++fs                        
                        )
ament_target_dependencies(${PROJECT_NAME}_lib 
                          rclcpp 
                          sensor_msgs
                          geometry_msgs
                        )

# Add executable
add_executable(${PROJECT_NAME}
                src/main.cpp
              )
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}
                          rclcpp
                          sensor_msgs
                          geometry_msgs
                        )

# Installs
install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
      )

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
      )

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
      )

# TESTING
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_joystick test/src/test_joystick.cpp)
  ament_target_dependencies(test_joystick
                            rclcpp 
                            sensor_msgs
                            geometry_msgs
                          )
  target_link_libraries(test_joystick ${PROJECT_NAME}_lib)
endif()

ament_package()
