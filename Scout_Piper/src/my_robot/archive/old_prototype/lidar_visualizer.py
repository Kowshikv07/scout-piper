import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class LidarVisualizer(Node):

    def __init__(self):
        super().__init__('lidar_visualizer')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/lidar_rays',
            10
        )

    def scan_callback(self, msg):

        marker = Marker()

        marker.header = msg.header
        marker.header.frame_id = msg.header.frame_id

        marker.ns = 'lidar_rays'
        marker.id = 0

        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Thin green rays
        marker.scale.x = 0.015

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        for i, distance in enumerate(msg.ranges):

            if not math.isfinite(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            start = Point()
            start.x = 0.0
            start.y = 0.0
            start.z = 0.0

            end = Point()
            end.x = distance * math.cos(angle)
            end.y = distance * math.sin(angle)
            end.z = 0.0

            marker.points.append(start)
            marker.points.append(end)

        self.marker_pub.publish(marker)


def main():
    rclpy.init()

    node = LidarVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
