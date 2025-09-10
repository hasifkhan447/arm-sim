import os 
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

from launch_ros.actions import Node 
import xacro 

def generate_launch_description():
    package_path = os.path.join(
        get_package_share_directory('gantry2')
    )

    xacro_path = os.path.join(package_path,
                              'urdf',
                              'gantry.xacro')

    rviz_config_path = os.path.join(package_path,
                                    'rviz',
                                    'view_only.rviz')

    xacro_doc = xacro.parse(open(xacro_path))
    xacro.process_doc(xacro_doc)

    params = {'robot_description' : xacro_doc.toxml()}

    node_joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    node_joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz_node_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time' : True},
        ]
    )



    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_joint_state_publisher,
        rviz_node_launch,
    ])



