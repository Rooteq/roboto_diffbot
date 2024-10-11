import os
import xacro

#TODO: fix use sim time as a settable parameter

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command



def generate_launch_description():

    # Check if we're told to use sim time
    sim_mode = LaunchConfiguration('sim_mode', default=False)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('roboto_diffbot'))

    xacro_file = os.path.join(pkg_path,'description','robot','robot.urdf.xacro')
    robot_desc = Command(['xacro ', xacro_file, ' sim_mode:=', sim_mode])
    # doc = xacro.process_file(xacro_file, mappings={'use_sim' : use_sim_time})
    # robot_desc = doc.toprettyxml(indent='  ')
    
    robot_state_publisher_params = {'robot_description': robot_desc, 'use_sim_time' : use_sim_time}  


    controls = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('roboto_diffbot'),'launch','controls_launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('roboto_diffbot'), 'launch', 'sllidar_c1_launch.py'
        )])
    )

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time' : use_sim_time}, 
                    robot_state_publisher_params]
        # launch_arguments=[{'user_ros2_control' : 'true'}.items()]
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory('roboto_diffbot'), 'config', 'controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description' : robot_description},
                    controller_params_file]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[diff_drive_spawner]
            )
    )

    delayed_broad_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[joint_broad_spawner]
            )
    )

    delayed_lidar_spawner = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=joint_broad_spawner,
                on_start=[lidar]
            )
    )

    delayed_controller_manager= TimerAction(period=3.0, actions=[controller_manager])


    localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('roboto_diffbot'), 'launch','localization_launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('roboto_diffbot'), 'launch','navigation_launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 'map_subscribe_transient_local' : 'true'}.items()
    )

    delayed_amcl = TimerAction(period=5.0, actions=[localization])
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('roboto_diffbot'), 'launch', 'online_async_launch.py')]),
        launch_arguments={
            'use_sim_time' : use_sim_time,
            'declare_slam_params_file_cmd' : os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml'),
            
        }.items()
    )
    delayed_slam = TimerAction(period=8.0, actions=[slam])

    delayed_nav = TimerAction(period=8.0, actions=[navigation])
    # Launch!
    return LaunchDescription([
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gz_spawn_entity,
        #         on_exit=[load_joint_state_broadcaster],
        #     )
        # ),

        # gazebo_resource_path,
        # arguments,
        # gazebo,dfs

        node_robot_state_publisher,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_broad_spawner,
        controls,
        delayed_lidar_spawner
        # delayed_slam
        # delayed_amcl,
        # delayed_nav
    ])