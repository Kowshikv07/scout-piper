"""
Minimal launch file to bring up the environment in Gazebo Sim (Harmonic/Fortress)
and bridge the clock to ROS 2. Place under your package's launch/ folder.

Usage:
  ros2 launch my_robot launch_world.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    world_path = os.path.join(pkg_share, 'worlds', 'nav_world.sdf')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([gz_sim, clock_bridge])
