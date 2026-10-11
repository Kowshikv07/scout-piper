import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


class BottleDistance(Node):

    def __init__(self):
        super().__init__('bottle_distance')

        self.bridge = CvBridge()
        self.latest_bbox = None
        self.last_report = 0

        self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/bottle_bbox',
            self.bbox_callback,
            10
        )

        self.get_logger().info(
            'Bottle distance diagnostic started'
        )

    def bbox_callback(self, msg):

        if len(msg.data) != 4:
            return

        self.latest_bbox = [
            int(msg.data[0]),
            int(msg.data[1]),
            int(msg.data[2]),
            int(msg.data[3])
        ]

    def depth_callback(self, msg):

        if self.latest_bbox is None:
            return

        depth = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='32FC1'
        )

        h, w = depth.shape

        x1, y1, x2, y2 = self.latest_bbox

        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h))

        region = depth[y1:y2, x1:x2]

        valid = region[
            np.isfinite(region) &
            (region > 0.1) &
            (region < 8.0)
        ]

        if valid.size == 0:
            self.get_logger().info(
                'BBox received, but no valid depth pixels'
            )
            return

        self.last_report += 1

        if self.last_report < 10:
            return

        self.last_report = 0

        self.get_logger().info(
            f'BBox=({x1},{y1},{x2},{y2}) | '
            f'valid={valid.size} | '
            f'min={np.min(valid):.3f} m | '
            f'p10={np.percentile(valid, 10):.3f} m | '
            f'median={np.median(valid):.3f} m | '
            f'p90={np.percentile(valid, 90):.3f} m | '
            f'max={np.max(valid):.3f} m'
        )


def main():

    rclpy.init()

    node = BottleDistance()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
