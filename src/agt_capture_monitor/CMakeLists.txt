cmake_minimum_required(VERSION 3.8)
project(agt_capture_monitor)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(agt_capture_msgs REQUIRED)

add_library(agt_stream_tracker SHARED src/stream_tracker.cpp)
target_compile_features(agt_stream_tracker PUBLIC cxx_std_17)
target_include_directories(agt_stream_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(monitor_node src/monitor_node.cpp)
target_compile_features(monitor_node PUBLIC cxx_std_17)
target_link_libraries(monitor_node agt_stream_tracker)
ament_target_dependencies(monitor_node
  rclcpp sensor_msgs diagnostic_msgs livox_ros_driver2 agt_capture_msgs)

install(TARGETS agt_stream_tracker monitor_node
  EXPORT export_agt_capture_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stream_tracker test/test_stream_tracker.cpp)
  target_link_libraries(test_stream_tracker agt_stream_tracker)
endif()

ament_export_targets(export_agt_capture_monitor HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
