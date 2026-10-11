import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class DepthProcessor(Node):

    def __init__(self):
        super().__init__('depth_processor')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/depth_sectors',
            10
        )

        self.num_sectors = 20
        self.max_distance = 8.0

        self.get_logger().info('Depth processor started')

    def depth_callback(self, msg):

        depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='32FC1'
        )

        height, width = depth.shape

        sector_width = width // self.num_sectors

        distances = []

        for i in range(self.num_sectors):

            x_start = i * sector_width

            if i == self.num_sectors - 1:
                x_end = width
            else:
                x_end = (i + 1) * sector_width

            sector = depth[180:360, x_start:x_end]

            valid = sector[
                np.isfinite(sector) &
                (sector > 0.1) &
                (sector <= self.max_distance)
            ]

            if valid.size == 0:
                distance = self.max_distance
            else:
                distance = float(np.percentile(valid, 10))

            distances.append(distance)

        output = Float32MultiArray()
        output.data = distances

        self.publisher.publish(output)


def main(args=None):

    rclpy.init(args=args)

    node = DepthProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
