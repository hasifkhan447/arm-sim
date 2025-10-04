cmake_minimum_required(VERSION 3.8)
project(gantry2_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(moveit_ros_move_group REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(linkattacher_msgs REQUIRED)

add_executable(gantry2_commander src/gantry2_commander.cpp)
target_include_directories(gantry2_commander PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${linkattacher_msgs_INCLUDE_DIRS} )
target_compile_features(gantry2_commander PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  gantry2_commander
  rclcpp
  rclcpp_action          # ADD THIS
  moveit_ros_planning_interface
  control_msgs           # ADD THIS
  trajectory_msgs        # ADD THIS
)
target_link_libraries(gantry2_commander
  # ... existing link libraries ...
  ${linkattacher_msgs_LIBRARIES}     # <--- Must be here!
)
install(TARGETS gantry2_commander
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
