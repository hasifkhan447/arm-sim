# Steps to run

1. You should mark run.sh and rebuild.sh as executable
2. Run ./rebuild.sh
3. Run ./run.sh




# Commands 

## On building the system
Make sure you are in the workspace directory. This is where the src, build, install, and log files are stored. Run the command: 
colcon build --symlink-install.

symlink-install allows you to avoid having to rebuild everything for python packages (unless you've added new files). For cpp packages you will need to rebuild.

Create a new package with the following command: 
ros2 pkg create  --build-type ament_cmake  --dependencies <dependencies> rclcpp  --node-name <name> <name>

After this is it imperative that you run 
source install/setup.sh

in order to populate your path properly. Many errors can be avoided by re-sourcing.

## On running the system 
Launch the gantry with gazebo:
ros2 launch gantry2 gazebo.py

Launch the gantry with rviz only: (useful for debugging purposes)
ros2 launch gantry2 rviz.py


For six_dof_arm (about to be deleted), it is 
ros2 launch six_dof_arm launch_moveit.py


# What do I need to do?
TODO: Based on ghufran's EE adjust the gripping and evaluate it
TODO: Set up cameras and then do tracking 
TODO: Do grasping of the cardboard box
