import os 
from ament_index_python.packages import get_package_share_directory

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node 
import xacro 
from moveit_configs_utils import MoveItConfigsBuilder

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

    world_path = os.path.join(
        get_package_share_directory('gantry2'),
        'worlds',
        'test.sdf'   # or .world
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world_path
        }.items()
    )


    moveit_config = (
        MoveItConfigsBuilder("gantry2")
        .robot_description(
            file_path="config/rectangular_gantry.urdf.xacro",
            # mappings={
            #     "ros2_control_hardware_type": "mock_components"
            # },
        )
        .robot_description_semantic(
            file_path="config/rectangular_gantry.srdf"
        )

        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
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


    load_ee_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'ee_controller'],
        output='screen'
    )



    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'gantry'
        ],
    )

    rviz_node_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_path],
        parameters=[
            moveit_config.robot_description_kinematics,
            {'use_sim_time' : True},
        ]
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

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[load_ee_controller]
            )
        ),



        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        run_move_group_node,
        rviz_node_launch
    ])



