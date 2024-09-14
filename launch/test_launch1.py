import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import xacro


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_project_bringup = get_package_share_directory('roboto_diffbot')


    # Process the URDF file
    # pkg_path = os.path.join(get_package_share_directory('roboto_diffbot'))
    # xacro_file = os.path.join(pkg_path,'description','roboto.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)
    
    sdf_file  =  os.path.join(pkg_project_bringup, 'description', 'worlds', 'roboto', 'roboto.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read() 
    
    # Create a robot_state_publisher node
    # params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
    # params = {'robot_description': robot_desc, 'use_sim_time': True}
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_bringup,
            'description',
            'worlds',
            'diff_drive.sdf'
        ])}.items(),
    )


#   gz_sim = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
#         launch_arguments={'gz_args': PathJoinSubstitution([
#             pkg_project_bringup,
#             'description',
#             'worlds',
#             'diff_drive.sdf'
#         ])}.items(),
#     )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'bridge_params.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[sdf_file],
        output=['screen']
    )
    # Launch!
    return LaunchDescription([
        gz_sim,
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='false',
        #     description='Use sim time if true'),
        bridge,
        node_robot_state_publisher,
        # joint_state_publisher_gui
    ])