import os 
from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'
        ])
    )


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'prismatic_chain_controller'],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'gantry'
        ]
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
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller]
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller]
            )
        ),

        gazebo,
        node_robot_state_publisher,
        spawn_entity



    ])



