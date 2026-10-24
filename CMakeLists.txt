cmake_minimum_required(VERSION 3.8)
project(rabbits_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rabbits_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rabcl REQUIRED)
find_package(rabcl_ros2 REQUIRED)

include_directories(include)

# Executables
add_executable(stm32_bridge_node
  src/stm32_bridge_node.cpp
)
ament_target_dependencies(stm32_bridge_node
  rclcpp
  rabbits_msgs
  sensor_msgs
  rabcl
  rabcl_ros2
)

add_executable(transceiver_module_bridge_node
  src/transceiver_module_bridge_node.cpp
)
ament_target_dependencies(transceiver_module_bridge_node
  rclcpp
  rabbits_msgs
)

add_executable(robot_bridge_node
  src/robot_bridge_node.cpp
)
ament_target_dependencies(robot_bridge_node
  rclcpp
  rabbits_msgs
  sensor_msgs
  std_msgs
  rabcl
  rabcl_ros2
)

install(TARGETS
  stm32_bridge_node
  transceiver_module_bridge_node
  robot_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Libraries for tests (main() excluded via preprocessor)
  add_library(stm32_bridge_lib STATIC src/stm32_bridge_node.cpp)
  ament_target_dependencies(stm32_bridge_lib rclcpp rabbits_msgs sensor_msgs rabcl rabcl_ros2)
  target_compile_definitions(stm32_bridge_lib PRIVATE RABBITS_BRIDGE_EXCLUDE_MAIN)

  add_library(transceiver_module_bridge_lib STATIC src/transceiver_module_bridge_node.cpp)
  ament_target_dependencies(transceiver_module_bridge_lib rclcpp rabbits_msgs)
  target_compile_definitions(transceiver_module_bridge_lib PRIVATE RABBITS_BRIDGE_EXCLUDE_MAIN)

  ament_add_gtest(test_crc test/test_crc.cpp)
  target_link_libraries(test_crc transceiver_module_bridge_lib)

  ament_add_gtest(test_process_frame test/test_process_frame.cpp)
  target_link_libraries(test_process_frame transceiver_module_bridge_lib)

  ament_add_gtest(test_stm32_bridge test/test_stm32_bridge.cpp)
  target_link_libraries(test_stm32_bridge stm32_bridge_lib)
  target_include_directories(stm32_bridge_lib PUBLIC include)

  add_library(robot_bridge_lib STATIC src/robot_bridge_node.cpp)
  ament_target_dependencies(robot_bridge_lib rclcpp rabbits_msgs sensor_msgs std_msgs rabcl rabcl_ros2)
  target_compile_definitions(robot_bridge_lib PRIVATE RABBITS_BRIDGE_EXCLUDE_MAIN)
  target_include_directories(robot_bridge_lib PUBLIC include)

  ament_add_gtest(test_robot_bridge test/test_robot_bridge.cpp)
  target_link_libraries(test_robot_bridge robot_bridge_lib)
endif()

ament_package()
