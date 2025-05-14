cmake_minimum_required(VERSION 3.8)
project(six_dof_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)  # and others you use
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(gazebo REQUIRED)


add_executable(trajectory src/trajectory.cpp)

ament_target_dependencies(trajectory
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_visual_tools 
)



install(TARGETS trajectory
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY urdf meshes launch rviz config worlds scripts
  DESTINATION share/${PROJECT_NAME}
)
# # Install Python scripts
# install(PROGRAMS
#   scripts/torque_plotter.py
#   DESTINATION lib/${PROJECT_NAME}
# )



link_directories(${GAZEBO_LIBRARY_DIRS})

# include_directories(
#   include
#   ${rclcpp_INCLUDE_DIRS}
#   ${geometry_msgs_INCLUDE_DIRS}
#   ${GAZEBO_INCLUDE_DIRS}
# )
#
# # Define the executable
# add_executable(torque_publisher src/torque_publisher.cpp)
#
# # Link libraries for ROS 2 and Gazebo
# target_link_libraries(torque_publisher
#   ${rclcpp_LIBRARIES}
#   ${geometry_msgs_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
# )
#
# # Install the executable
# install(TARGETS torque_publisher
#   DESTINATION lib/${PROJECT_NAME}
# )


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
