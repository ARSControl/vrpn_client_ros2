cmake_minimum_required(VERSION 3.5)
project(vrpn_client_ros2)

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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(VRPN REQUIRED)

include_directories(
  include
  SYSTEM ${VRPN_INCLUDE_DIR}
)

add_library(vrpn_client_ros2
  src/vrpn_client_ros2.cpp
)
ament_target_dependencies(vrpn_client_ros2
  rclcpp
  geometry_msgs
  tf2_ros
)
target_link_libraries(vrpn_client_ros2
  ${VRPN_LIBRARIES}
)
#ament_export_targets(vrpn_client_ros2 HAS_LIBRARY_TARGET)

#add_executable(vrpn_tracker_node
#  src/vrpn_tracker_node.cpp)
#ament_target_dependencies(vrpn_tracker_node vrpn_client_ros2)

add_executable(vrpn_client_node
  src/vrpn_client_node.cpp)
target_link_libraries(vrpn_client_node vrpn_client_ros2)
#ament_target_dependencies(vrpn_client_node vrpn_client_ros2)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS vrpn_client_ros2
  DESTINATION lib/${PROJECT_NAME})
  install(TARGETS vrpn_client_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
