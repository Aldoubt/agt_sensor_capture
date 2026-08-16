cmake_minimum_required(VERSION 3.8)
project(agt_hik_camera_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(AGT_BUILD_HIK_CAMERA_DRIVER "Build the MVS-backed Hikrobot camera executable" ON)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(agt_timebase REQUIRED)

add_library(agt_camera_timestamp_policy SHARED src/timestamp_policy.cpp)
target_compile_features(agt_camera_timestamp_policy PUBLIC cxx_std_17)
target_include_directories(agt_camera_timestamp_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(agt_camera_timestamp_policy agt_timebase)

if(AGT_BUILD_HIK_CAMERA_DRIVER)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(camera_info_manager REQUIRED)
  find_package(agt_capture_msgs REQUIRED)

  find_path(MVS_INCLUDE_DIR MvCameraControl.h HINTS /opt/MVS/include /usr/local/include)
  find_library(MVS_LIBRARY MvCameraControl HINTS /opt/MVS/lib/64 /opt/MVS/lib/aarch64 /usr/local/lib)
  if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
    message(FATAL_ERROR "Hikrobot MVS SDK not found. Install MVS or build with -DAGT_BUILD_HIK_CAMERA_DRIVER=OFF for pure unit tests.")
  endif()

  add_executable(camera_node src/camera_node.cpp)
  target_compile_features(camera_node PUBLIC cxx_std_17)
  target_include_directories(camera_node PRIVATE ${MVS_INCLUDE_DIR})
  target_link_libraries(camera_node agt_camera_timestamp_policy ${MVS_LIBRARY} Threads::Threads)
  ament_target_dependencies(camera_node rclcpp sensor_msgs camera_info_manager agt_capture_msgs agt_timebase)
  install(TARGETS camera_node RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS agt_camera_timestamp_policy
  EXPORT export_agt_camera_timestamp_policy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_timestamp_policy test/test_timestamp_policy.cpp)
  target_link_libraries(test_timestamp_policy agt_camera_timestamp_policy)
endif()

ament_export_targets(export_agt_camera_timestamp_policy HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(agt_timebase)
ament_package()
