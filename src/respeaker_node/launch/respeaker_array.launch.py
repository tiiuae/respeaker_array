from launch import LaunchDescription
from launch_ros.actions import Node
import getpass

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='respeaker_node',
            namespace='{:s}'.format(getpass.getuser()),
            executable='mic_array',
            name='mic_array'
        ),
        Node(
            package='respeaker_node',
            namespace='{:s}'.format(getpass.getuser()),
            executable='storage',
            name='audio_storage'
        )
    ])