import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_my_box_description = get_package_share_directory('my_box_description')

    # Define the path to your URDF file
    urdf_file = os.path.join(pkg_my_box_description, 'urdf', 'box.urdf')

    # Set a launch argument for the use of simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Launch the Gazebo server and client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'empty.world'}.items(),
    )

    # Spawn the URDF model in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_box'],
        output='screen',
    )

    # Publish the URDF to the topic '/robot_description'
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file, 'r').read()}],
        arguments=[urdf_file],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
