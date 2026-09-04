cmake_minimum_required(VERSION 3.8)
project(ifssim_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fs_msgs REQUIRED)

# Bridge library
add_library(ifssim_ros_wrapper STATIC
  src/ifssim_ros_wrapper.cpp
  src/tcp_client.cpp
  src/udp_receiver.cpp
)
target_include_directories(ifssim_ros_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ifssim_ros_wrapper
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros fs_msgs
)

# Link Winsock2 on Windows
if(WIN32)
  target_link_libraries(ifssim_ros_wrapper ws2_32)
endif()

# Main bridge node
add_executable(ifssim_bridge src/ifssim_bridge_node.cpp)
target_link_libraries(ifssim_bridge ifssim_ros_wrapper)
ament_target_dependencies(ifssim_bridge rclcpp)

# Install
install(TARGETS ifssim_bridge ifssim_ros_wrapper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Unit tests — header-only LWS sensor model (no ROS plumbing required).
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lws_steering_sensor
    test/test_lws_steering_sensor.cpp
  )
  target_include_directories(test_lws_steering_sensor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
