cmake_minimum_required(VERSION 3.8)
project(agt_g70_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(agt_capture_msgs REQUIRED)

add_library(agt_g70_ubx SHARED src/ubx.cpp)
target_compile_features(agt_g70_ubx PUBLIC cxx_std_17)
target_include_directories(agt_g70_ubx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(agt_g70_serial SHARED src/serial_port.cpp)
target_compile_features(agt_g70_serial PUBLIC cxx_std_17)
target_include_directories(agt_g70_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(agt_g70_ros_conversion SHARED src/ros_conversion.cpp)
target_compile_features(agt_g70_ros_conversion PUBLIC cxx_std_17)
target_include_directories(agt_g70_ros_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(agt_g70_ros_conversion agt_g70_ubx)
ament_target_dependencies(agt_g70_ros_conversion
  rclcpp sensor_msgs geometry_msgs agt_capture_msgs)

add_executable(g70_node src/g70_node.cpp)
target_compile_features(g70_node PUBLIC cxx_std_17)
target_link_libraries(g70_node agt_g70_ubx agt_g70_serial agt_g70_ros_conversion Threads::Threads)
ament_target_dependencies(g70_node
  rclcpp sensor_msgs geometry_msgs diagnostic_msgs agt_capture_msgs)

install(TARGETS agt_g70_ubx agt_g70_serial agt_g70_ros_conversion g70_node
  EXPORT export_agt_g70_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ubx test/test_ubx.cpp)
  target_link_libraries(test_ubx agt_g70_ubx)
  ament_add_gtest(test_serial test/test_serial.cpp)
  target_link_libraries(test_serial agt_g70_serial util)
endif()

ament_export_targets(export_agt_g70_driver HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs diagnostic_msgs agt_capture_msgs)
ament_package()
