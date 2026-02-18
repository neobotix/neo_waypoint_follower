cmake_minimum_required(VERSION 3.5)

project(neo_waypoint_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LooperMetrics.msg"
  "msg/Waypoints.msg"
  "srv/VaultList.srv"
  "srv/VaultSaveCurrent.srv"
  "srv/VaultLoadToLooper.srv"
  "srv/VaultPreview.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Ensure generated headers are available to targets
ament_export_dependencies(rosidl_default_runtime)

# Save Waypoints Server Executable
add_executable(save_waypoints_server src/save_waypoints_server.cpp)
ament_target_dependencies(save_waypoints_server
  rclcpp visualization_msgs std_srvs)
target_link_libraries(save_waypoints_server yaml-cpp)

install(TARGETS save_waypoints_server
  DESTINATION lib/${PROJECT_NAME})

# Waypoint Looper Executable
add_executable(waypoint_looper src/waypoint_looper.cpp)
ament_target_dependencies(waypoint_looper
  rclcpp rclcpp_action nav2_msgs geometry_msgs std_srvs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(waypoint_looper ${cpp_typesupport_target})

target_link_libraries(waypoint_looper yaml-cpp)

install(TARGETS waypoint_looper
  DESTINATION lib/${PROJECT_NAME})

# Vault Manager Executable
add_executable(vault_manager src/vault_manager.cpp)
ament_target_dependencies(vault_manager
  rclcpp rclcpp_action rclcpp_components std_srvs geometry_msgs visualization_msgs rcl_interfaces yaml-cpp)
target_link_libraries(vault_manager ${cpp_typesupport_target})
target_link_libraries(vault_manager yaml-cpp)

install(TARGETS vault_manager
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and configs
install(DIRECTORY launch config msg srv
  DESTINATION share/${PROJECT_NAME})

ament_package()
