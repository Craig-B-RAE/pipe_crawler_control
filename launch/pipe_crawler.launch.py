from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboclaw_driver2',
            executable='roboclaw_node',
            name='roboclaw',
            parameters=[
                {'dev_names': '/dev/ttyACM0'},
                {'baud_rate': 115200},
                {'address': 128},
                {'loop_hz': 10},
                {'deadman_secs': 3},
                {'speed_cmd_topic': 'speed_command'},
                {'stats_topic': 'stats'},
                {'test_mode': False}
            ]
        ),
        Node(
            package='pipe_crawler_control',
            executable='cpu_temp_publisher',
            name='cpu_temp_publisher'
        ),
        Node(
            package='pipe_crawler_control',
            executable='simple_controller',
            name='simple_controller'
        ),
        Node(
            package='pipe_crawler_control',
            executable='update_manager',
            name='update_manager'
        ),
       # Node(
       #    package='pipe_crawler_control',
       #    executable='network_info_publisher',
       #    name='network_info_publisher'
       # ),
    ])
