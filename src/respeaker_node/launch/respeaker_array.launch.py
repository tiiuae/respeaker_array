from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='respeaker_node',
            namespace='mic_array',
            executable='mic_array',
            name='mic'
        ),
        Node(
            package='respeaker_node',
            namespace='audio_storage',
            executable='storage',
            name='audio_storage'
        )
    ])