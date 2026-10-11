import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('my_robot')
    world = os.path.join(pkg, 'worlds', 'vehicle.sdf')
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '3', world],
            output='screen'
        ),
        # LiDAR bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            output='screen'
        ),
        # Command velocity bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/vehicle/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            output='screen'
        ),
        # Odometry bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/vehicle/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ],
            output='screen'
        ),
        # Camera bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen'
        ),
        # Arm joint command bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/vehicle/shoulder_cmd@std_msgs/msg/Float64@gz.msgs.Double'
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/vehicle/elbow_cmd@std_msgs/msg/Float64@gz.msgs.Double'
            ],
            output='screen'
        ),
        # Attach bridge (grab trigger)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/attach@std_msgs/msg/Empty@gz.msgs.Empty'
            ],
            output='screen'
        ),
        # Obstacle avoidance controller
        Node(
            package='my_robot',
            executable='obstacle_avoidance',
            output='screen'
        ),
        Node(
            package='my_robot',
            executable='lidar_visualizer',
            output='screen'
        ),
        # Bottle color detector
        Node(
            package='my_robot',
            executable='bottle_detector',
            output='screen'
        ),
        # Arm controller
        Node(
            package='my_robot',
            executable='arm_controller',
            output='screen'
        ),
    ])
