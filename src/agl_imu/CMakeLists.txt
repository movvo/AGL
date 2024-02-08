cmake_minimum_required(VERSION 3.5)
project(agl_imu)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

#=====================================
#             Find dependencies
#=====================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(atlas_interfaces REQUIRED)
find_package(PkgConfig)
find_package(atlas_utils REQUIRED)

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
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  atlas_interfaces
  atlas_utils
)

add_library(${PROJECT_NAME}_lib
  src/imu.cpp
  src/imu_drivers/wit_imu.cpp
  src/imu_drivers/aceinna_imu.cpp
  src/can_utils/CANdb.cpp
)

# libserial
pkg_check_modules(SERIAL libserial)
target_include_directories(${PROJECT_NAME}_lib PRIVATE ${SERIAL_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

target_link_libraries(${PROJECT_NAME}_lib
  ${SERIAL_LIBRARIES} 
  ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

#=====================================
#          Install
#=====================================
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

#=====================================
#          Test
#=====================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_${PROJECT_NAME} test/src/test_imu.cpp)
  ament_target_dependencies(test_${PROJECT_NAME} ${dependencies})
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_lib)
endif()

ament_package()
