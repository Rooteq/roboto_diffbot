import os
import xacro



from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('test'))

    xacro_file = os.path.join(pkg_path,'description','robot','robot.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    
    params = {'robot_description': robot_desc}  

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time' : True}, 
                    params]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/rooteq/ros2_ws/src/test/sim/world:/home/rooteq/ros2_ws/src/test/description/robot'
        # value=[
        #     os.path.join(pkg_path, 'description', 'sim', 'world')
        #     ]
        ) # FIX FIX FIX FIX FIX
    
    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='world',
                          description='Gz sim World'),
           ]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -r']
                    )
                ]
             )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.07',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'roboto',
                   '-allow_renaming', 'false'],
    )


    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_path, 'config', 'bridge_params.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    rviz_config_file = os.path.join(pkg_path, 'config', 'view_robot.rviz')

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_diff_drive_controller],
            )
        ),
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        rviz
    ])