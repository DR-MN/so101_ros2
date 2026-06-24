from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='so101_ros2',
            executable='so101_inference_node_with_conversion',
            name='so101_inference_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'robot_name': 'so101_follower'},
                {'port': '/dev/ttyACM0'},
                {'recalibrate': False},
                {'publish_rate': 30.0},
            ]
        )
    ])