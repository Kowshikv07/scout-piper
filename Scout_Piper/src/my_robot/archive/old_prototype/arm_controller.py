import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64, Empty


class ArmController(Node):

    def __init__(self):
        super().__init__('arm_controller')

        self.create_subscription(
            Bool,
            '/start_pick',
            self.start_pick_callback,
            10
        )

        self.shoulder_pub = self.create_publisher(
            Float64,
            '/model/vehicle/shoulder_cmd',
            10
        )

        self.elbow_pub = self.create_publisher(
            Float64,
            '/model/vehicle/elbow_cmd',
            10
        )

        self.attach_pub = self.create_publisher(
            Empty,
            '/attach',
            10
        )

        self.done_pub = self.create_publisher(
            Bool,
            '/pick_complete',
            10
        )

        self.get_logger().info('Arm controller ready')

    def send(self, pub, value):
        msg = Float64()
        msg.data = value
        pub.publish(msg)

    def start_pick_callback(self, msg):
        if not msg.data:
            return

        self.get_logger().info('Starting pick sequence')

        # Lower arm down toward the bottle
        self.send(self.shoulder_pub, 1.2)
        self.send(self.elbow_pub, 1.0)
        time.sleep(2.0)

        # Grab
        self.attach_pub.publish(Empty())
        self.get_logger().info('Bottle attached')
        time.sleep(0.5)

        # Lift back up, holding the bottle
        self.send(self.shoulder_pub, 0.2)
        self.send(self.elbow_pub, 0.0)
        time.sleep(2.0)

        done = Bool()
        done.data = True
        self.done_pub.publish(done)

        self.get_logger().info('Pick sequence complete')


def main():
    rclpy.init()

    node = ArmController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
