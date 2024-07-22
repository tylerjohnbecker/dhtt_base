from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dhtt',
            executable='start_server',
        ),
        Node(
            package='dhtt_plot',
            executable='build_nutree',
        ),
    ])