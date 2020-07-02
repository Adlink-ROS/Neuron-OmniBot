cmake_minimum_required(VERSION 3.5)

project(omni_base_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(base_controller REQUIRED)

include_directories(
  include
  ${BASE_CONTROLLER_INCLUDE_DIRS}
  )

set(omnibot_node_name "omnibot_node")
set(node_plugins "")

add_library(${omnibot_node_name} SHARED
  src/omnibot_node.cpp
  )

target_link_libraries(${omnibot_node_name}
  ${BASE_CONTROLLER_LIBRARIES}
  )

ament_target_dependencies(${omnibot_node_name}
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  )

rclcpp_components_register_nodes(${omnibot_node_name}
	"omnibot_base::OmniBotNode")
set(node_plugins
  "${node_plugins}omnibot_base::OmniBotNode;$<TARGET_FILE:omnibot_base>\n")

install(TARGETS
  ${omnibot_node_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
