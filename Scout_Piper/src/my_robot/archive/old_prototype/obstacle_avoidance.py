import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/model/vehicle/odometry', self.odom_callback, 10)
        self.create_subscription(Bool, '/bottle_in_view', self.bottle_callback, 10)
        self.create_subscription(Bool, '/pick_complete', self.pick_complete_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/model/vehicle/cmd_vel', 10)
        self.start_pick_pub = self.create_publisher(Bool, '/start_pick', 10)

        self.yaw = None
        self.turn_start_yaw = None
        self.turning = False
        self.picking = False
        self.carrying = False
        self.carrying = False
        self.bottle_in_view = False

        self.get_logger().info('Obstacle avoidance started')

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def bottle_callback(self, msg):
        self.bottle_in_view = msg.data

def pick_complete_callback(self, msg):        if msg.data:            self.picking = False            self.carrying = True            self.get_logger().info('Pick complete - now carrying bottle, avoidance continues')    def angle_difference(self, a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def scan_callback(self, msg):
        if self.yaw is None:
            return

        if self.picking or self.carrying:
            self.cmd_pub.publish(Twist())
            return

        front = list(msg.ranges[150:211])
        valid = [r for r in front if msg.range_min < r < msg.range_max]

        cmd = Twist()

        if self.turning:
            turned = abs(self.angle_difference(self.yaw, self.turn_start_yaw))
            remaining = math.radians(90) - turned

            if remaining <= 0.01:
                self.turning = False
                cmd.linear.x = 0.3
                cmd.angular.z = 0.0
                self.get_logger().info('90 degree turn completed - moving forward')
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = max(0.15, min(1.0, remaining * 2.0))

        else:
            if valid and min(valid) < 0.3:   # was 0.9 — get much closer before grabbing
                # Only trust the camera as "this is the bottle" when we're
                # this close — otherwise a bottle visible off to the side
                # gets wrongly matched to whatever is actually in front.
                if self.bottle_in_view:
                    self.picking = True
                    self.cmd_pub.publish(Twist())
                    start = Bool()
                    start.data = True
                    self.start_pick_pub.publish(start)
                    self.get_logger().info('Bottle confirmed at close range - picking')
                    return
                else:
                    self.turning = True
                    self.turn_start_yaw = self.yaw
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info('Obstacle detected - starting 90 degree turn')

            elif valid and min(valid) < 3.0 and not self.bottle_in_view:
                self.turning = True
                self.turn_start_yaw = self.yaw
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('Obstacle detected - starting 90 degree turn')

            else:
                cmd.linear.x = 0.3
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
