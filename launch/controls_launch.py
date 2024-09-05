from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

import os
from ament_index_python.packages import get_package_share_directory

use_sim_time = LaunchConfiguration('use_sim_time')

def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('RobotoDiffSim'), 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            remappings=[('/cmd_vel_in','/diff_drive_controller/cmd_vel_unstamped'),
                        ('/cmd_vel_out','/diff_drive_controller/cmd_vel')]
        )
    
    twist_mux_params = os.path.join(get_package_share_directory('RobotoDiffSim'), 'config', 'twist_mux.yaml') 
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')]
    )
    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_stamper,
        twist_mux
    ])