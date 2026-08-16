cmake_minimum_required(VERSION 3.8)
project(agt_g70_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(agt_g70_ubx SHARED src/ubx.cpp)
target_compile_features(agt_g70_ubx PUBLIC cxx_std_17)
target_include_directories(agt_g70_ubx
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS agt_g70_ubx
  EXPORT export_agt_g70_ubx
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ubx test/test_ubx.cpp)
  target_link_libraries(test_ubx agt_g70_ubx)
endif()

ament_export_targets(export_agt_g70_ubx HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
