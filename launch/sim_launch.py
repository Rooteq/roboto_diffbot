import os
import xacro

# TODO: fix use sim time as a settable parameter

from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory("roboto_diffbot"))
    apriltag_ros_dir = get_package_share_directory('apriltag_ros')

    apriltags_config_file = os.path.join(apriltag_ros_dir, 'cfg', 'tags_36h11.yaml')

    # Check if we're told to use sim time
    sim_mode = LaunchConfiguration("sim_mode", default=True)
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    xacro_file = os.path.join(pkg_path, "description", "robot", "robot.urdf.xacro")
    robot_desc = Command(["xacro ", xacro_file, " sim_mode:=", sim_mode])
    robot_state_publisher_params = {
        "robot_description": robot_desc,
        "use_sim_time": use_sim_time,
    }

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_state_publisher_params],
    )

    controls = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("roboto_diffbot"),
                    "launch",
                    "controls_launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("roboto_diffbot"),
                    "launch",
                    "localization_launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("roboto_diffbot"),
                    "launch",
                    "navigation_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_subscribe_transient_local": "true",
        }.items(),
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_path, "sim", "world")
        + ":"
        + os.path.join(pkg_path, "description", "robot"),
    )

    arguments = LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value="world", description="Gz sim World"
            ),
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[("gz_args", [LaunchConfiguration("world"), ".sdf", " -r"])],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",robot_desc,
            "-x","0.5",
            "-y","0.5",
            "-z","0.07",
            "-R","0.0",
            "-P","0.0",
            "-Y","0.0",
            "-name","roboto",
            "-allow_renaming","false",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_path, "config", "bridge_params.yaml"),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )


    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    dock_detection = Node(
        package="roboto_diffbot",
        executable="detected_dock_pose_publisher",
        output="screen",
        parameters=[{
            'use_sim_time': True
        }],
        emulate_tty=True
    )

    apriltags = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[{'params_file': "/home/rooteq/ros2_ws/install/apriltag_ros/share/apriltag_ros/cfg/tags_36h11.yaml"}]
    )

    rviz_config_file = os.path.join(pkg_path, "config", "nav_view.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_broad",
        ],
        output="screen",
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_cont",
        ],
        output="screen",
    )

    gui_integration_node = Node(
        package="roboto_diffbot",
        executable="gui_integration_node",
        output="screen",
        emulate_tty=True,
    )

    delayed_gui_integration = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_robot_state_publisher,
            on_start=[gui_integration_node],
        )
    )

    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',  # Change this
        name='rectify_node',
        remappings=[
            ('image', 'camera/image_raw'),
            ('image_rect', 'camera/image_rect'),
        ],
        parameters=[{
            'queue_size': 5,
            'interpolation': 1,
            'use_sim_time': use_sim_time,
            'image_transport': 'raw'
        }],
        # Rest of parameters remain the same
    )

    return LaunchDescription(
        [
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
            rviz,
            controls,
            localization,
            navigation,
            delayed_gui_integration,

            # image_bridge,
            # dock_detection,
            # rectify_node
            # apriltags
        ]
    )
