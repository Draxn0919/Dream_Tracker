
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='proyecto_final',
            executable='img_publi',
            name='img_publi',
            output='screen',
        ),
        Node(
            package='proyecto_final',
            executable='img_emo',
            name='img_emo',
            output='screen',
        ),
        Node(
            package='proyecto_final',
            executable='img_microdreams',
            name='img_microdreams',
            output='screen',
        ),
        Node(
            package='proyecto_final',
            executable='img_tracking',
            name='img_tracking',
            output='screen',
        ),
        Node(
            package='proyecto_final',
            executable='img_displa',
            name='img_displa',
            output='screen',
        ),
    ])