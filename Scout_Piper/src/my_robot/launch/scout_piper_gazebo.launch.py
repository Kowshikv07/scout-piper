from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    headless = LaunchConfiguration("headless")

    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Launch Gazebo headless?"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            "/home/kowshik0714/ros2_ws/src/my_robot/models/scout_piper/scout_piper.xacro",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description_content,
            }
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ros_gz_sim"),
                "/launch/gz_sim.launch.py"
            ]
        ),
        launch_arguments=[
            (
                "gz_args",
                [
                    "-r -v 3 "
                    "/home/kowshik0714/ros2_ws/src/my_robot/worlds/scout_world.sdf"
                ]
            )
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "scout_piper",
            "-allow_renaming",
            "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.18",
        ],
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
	arguments=[
	    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
	    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
	    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
	    "/camera/depth@sensor_msgs/msg/Image[gz.msgs.Image",
	    "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
	    "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
	    "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
	],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_headless,
            robot_state_publisher_node,
            gazebo,
            spawn_robot,
            gazebo_bridge,
        ]
    )
