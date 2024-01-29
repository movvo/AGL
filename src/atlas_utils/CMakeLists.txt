cmake_minimum_required(VERSION 3.5)
project(atlas_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra )
endif()

#=====================================
#          Find dependencies
#=====================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ageve_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

#=====================================
#          Include directories
#=====================================
include_directories(
  include
  test/include
)

#=====================================
#          Libraries and Executable
#=====================================
set(dependencies
  rclcpp
  ageve_interfaces
  OpenCV
  cv_bridge
  image_transport
  sensor_msgs
)

add_library(${PROJECT_NAME}
  "src/general/general_funcs.cpp"
  "src/general/system_funcs.cpp"
  "src/image/image_transport.cpp"
  "src/sm/StateMachine.cpp"
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#=====================================
#          Installs
#=====================================
install(DIRECTORY include/
  DESTINATION include/
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#=====================================
#          Test
#=====================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  set(TESTS "")
  list(APPEND TESTS test_${PROJECT_NAME}_general)
  list(APPEND TESTS test_${PROJECT_NAME}_image)
  list(APPEND TESTS test_${PROJECT_NAME}_sm)

  ament_add_gmock(test_${PROJECT_NAME}_general test/src/test_general.cpp)
  ament_add_gmock(test_${PROJECT_NAME}_image test/src/test_image.cpp)
  ament_add_gmock(test_${PROJECT_NAME}_sm test/src/test_sm.cpp)

  foreach(TEST IN LISTS TESTS)
    ament_target_dependencies(${TEST} ${dependencies})
    target_link_libraries(${TEST} ${PROJECT_NAME})
  endforeach()
endif()


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()