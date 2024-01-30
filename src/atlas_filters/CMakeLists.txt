cmake_minimum_required(VERSION 3.5)
project(atlas_filters)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

#=====================================
#          Find dependencies
#=====================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(atlas_interfaces REQUIRED)
find_package(atlas_utils REQUIRED)
find_package(message_filters REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

#=====================================
#          Include directories
#=====================================
include_directories(
  include
  test/include
)

#=====================================
#          Libraries and Executables
#=====================================
set(dependencies
  rclcpp 
  sensor_msgs
  std_msgs
  atlas_interfaces
  atlas_utils
  message_filters
  laser_geometry
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
  tf2
  tf2_eigen
  tf2_sensor_msgs
  PCL
)

add_library(${PROJECT_NAME}_lib
  "src/filter_node.cpp"
)
target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

#=====================================
#          Install
#=====================================
install(TARGETS atlas_filters
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

#=====================================
#          Test
#=====================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_${PROJECT_NAME} test/src/test_filters.cpp)
  ament_target_dependencies(test_${PROJECT_NAME} ${dependencies})
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_lib)
endif()

ament_package()