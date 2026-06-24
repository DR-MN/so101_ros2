"""
so101_moveit_bridge_launch.py

Launches the SO101 ↔ MoveIt2 bridge.

Usage:
  ros2 launch so101_ros2 so101_moveit_bridge_launch.py
  ros2 launch so101_ros2 so101_moveit_bridge_launch.py port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port',         default_value='/dev/ttyACM0',
                              description='Serial port of the SO101 arm'),
        DeclareLaunchArgument('robot_name',   default_value='so101_follower',
                              description='Arm name used in calibration'),
        DeclareLaunchArgument('recalibrate',  default_value='false',
                              description='Run calibration on startup'),
        DeclareLaunchArgument('publish_rate', default_value='30.0',
                              description='Joint-state publish rate (Hz)'),

        Node(
            package='so101_ros2',
            executable='so101_moveit_bridge',   # must match setup.py entry_points
            name='so101_moveit_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port':         LaunchConfiguration('port'),
                'robot_name':   LaunchConfiguration('robot_name'),
                'recalibrate':  LaunchConfiguration('recalibrate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }],
        ),
    ])