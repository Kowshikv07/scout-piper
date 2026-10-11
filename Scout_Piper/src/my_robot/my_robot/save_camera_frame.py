import cv2
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraSaver(Node):

    def __init__(self):
        super().__init__('camera_saver')

        self.bridge = CvBridge()
        self.saved = False

        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.callback,
            10
        )

        self.get_logger().info('Waiting for camera image...')

    def callback(self, msg):

        if self.saved:
            return

        image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        path = '/tmp/raw_camera.png'

        cv2.imwrite(path, image)

        self.saved = True

        self.get_logger().info(
            f'Saved camera frame to {path}'
        )

        rclpy.shutdown()


def main():
    rclpy.init()
    node = CameraSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
