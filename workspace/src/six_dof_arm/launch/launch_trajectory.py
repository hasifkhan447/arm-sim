import os
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import TimerAction

import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'
        ])
    )

    package_path = os.path.join(
        get_package_share_directory('six_dof_arm')
    )

    xacro_file = os.path.join(package_path,
                              'urdf',
                              'kr120r2500pro.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    moveit_pkg_path = get_package_share_directory('six_dof_arm_moveit_config')
    srdf_path = os.path.join(moveit_pkg_path, 'config',
                             'kuka_kr120r2500pro.srdf')
    with open(srdf_path, 'r') as srdf_file:
        robot_srdf = srdf_file.read()

    params = {
        'robot_description': doc.toxml(),
        'robot_description_semantic': robot_srdf,
        'use_sim_time': True
    }

    moveit_config = (
        MoveItConfigsBuilder("six_dof_arm")
        .robot_description(
            file_path="config/kuka_kr120r2500pro.urdf.xacro",
            # mappings={
            #     "ros2_control_hardware_type": "mock_components"
            # },
        )
        .robot_description_semantic(
            file_path="config/kuka_kr120r2500pro.srdf"
        )

        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}  # <-- ADD THIS LINE
        ],
    )

    rviz_base = os.path.join(package_path, "rviz")
    rviz_config_path = os.path.join(rviz_base, "six_dof_arm.rviz")
    print(rviz_config_path)

    rviz_node_launch = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': True},
            moveit_config.joint_limits,
            moveit_config.robot_description_kinematics
        ]
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
        output='screen',
        # Set environment variable for sim time
        additional_env={'ROS_USE_SIM_TIME': 'true'}
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'arm_group_controller'],
        output='screen',
        # Set environment variable for sim time
        additional_env={'ROS_USE_SIM_TIME': 'true'}
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'six_dof_arm'
        ],

        parameters=[
            {'use_sim_time': True}
        ]
    )

    run_planning = Node(
        package="six_dof_arm",
        executable="trajectory",
        output="screen",
    )

    delayed_run_planning = TimerAction(
        period=5.0,
        actions=[run_planning]
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
        spawn_entity,
        run_move_group_node,
        # rviz_node_launch,

        delayed_run_planning
    ])
