cmake_minimum_required(VERSION 3.5)
project(nav2_send_goal)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# include 
include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(navigation2 REQUIRED)

add_executable(rosbot_auto_localizator 
  src/amcl_auto_localization_node.cpp 
  src/amcl_auto_localization.cpp
)
add_executable(nav2_send_goal
  src/nav2_send_goal.cpp
)
add_executable(nav2_get_position
  src/nav2_get_position.cpp
)
add_executable(nav2_get_positionAMCL
  src/nav2_get_positionAMCL.cpp
)
add_executable(nav2_send_waypoint
  src/nav2_send_waypoint.cpp
)
add_executable(nav2_send_initial_pose
  src/nav2_send_initial_pose.cpp
)
ament_target_dependencies(nav2_send_goal
  rclcpp
  rclcpp_action
  nav2_msgs
)
ament_target_dependencies(nav2_get_position
  rclcpp
  rclcpp_action
  nav2_msgs
)

ament_target_dependencies(nav2_get_positionAMCL
  rclcpp
  rclcpp_action
  nav2_msgs
)
ament_target_dependencies(nav2_send_waypoint
  rclcpp
  rclcpp_action
  nav2_msgs
)
ament_target_dependencies(nav2_send_initial_pose
  rclcpp
  geometry_msgs
)

target_link_libraries(rosbot_auto_localizator 
  ${Boost_LIBRARIES}
  )
ament_target_dependencies(
  rosbot_auto_localizator 
  rclcpp navigation2 
  std_srvs 
  lifecycle_msgs
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  nav2_send_goal
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  nav2_get_position
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  nav2_get_positionAMCL
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  nav2_send_waypoint
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  rosbot_auto_localizator
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  nav2_send_initial_pose
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
