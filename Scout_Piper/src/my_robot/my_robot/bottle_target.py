"""
bottle_target.py

Combines:
    /bottle_bbox
    /camera/depth
    /camera/camera_info

Publishes:
    /bottle_target
        [goal_distance_m, goal_angle_rad]

    /bottle_target_valid
        True when a recent bottle target is available.

Angle convention:
    positive -> bottle is to the RIGHT
    negative -> bottle is to the LEFT
"""

import math

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32MultiArray
from cv_bridge import CvBridge


class BottleTarget(Node):

    def __init__(self):
        super().__init__('bottle_target')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------

        self.declare_parameter(
            'bbox_topic',
            '/bottle_bbox'
        )

        self.declare_parameter(
            'depth_topic',
            '/camera/depth'
        )

        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera_info'
        )

        self.declare_parameter(
            'percentile',
            10.0
        )

        self.declare_parameter(
            'min_depth',
            0.1
        )

        self.declare_parameter(
            'max_depth',
            8.0
        )

        self.declare_parameter(
            'timeout',
            1.0
        )

        self.bridge = CvBridge()

        # Latest depth image
        self.depth = None

        # Latest camera intrinsics
        # Known Gazebo values used as fallback
        self.fx = 554.3827
        self.cx = 320.0

        # Time of most recent valid target
        self.last_target_time = None

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------

        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter(
                'camera_info_topic'
            ).value,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Float32MultiArray,
            self.get_parameter('bbox_topic').value,
            self.bbox_callback,
            10
        )

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------

        self.target_pub = self.create_publisher(
            Float32MultiArray,
            '/bottle_target',
            10
        )

        self.valid_pub = self.create_publisher(
            Bool,
            '/bottle_target_valid',
            10
        )

        # Check whether target is stale
        self.create_timer(
            0.1,
            self.validity_timer
        )

        self.get_logger().info(
            'Bottle target node started'
        )

    # ======================================================
    # Camera information
    # ======================================================

    def camera_info_callback(self, msg):

        if msg.k[0] > 0:

            self.fx = float(msg.k[0])
            self.cx = float(msg.k[2])

    # ======================================================
    # Depth image
    # ======================================================

    def depth_callback(self, msg):

        try:

            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='32FC1'
            )

            self.depth = np.asarray(
                depth,
                dtype=np.float32
            )

        except Exception as e:

            self.get_logger().error(
                f'Depth conversion failed: {e}'
            )

    # ======================================================
    # YOLO bounding box
    # ======================================================

    def bbox_callback(self, msg):

        if self.depth is None:
            return

        if len(msg.data) != 4:
            return

        x1 = int(msg.data[0])
        y1 = int(msg.data[1])
        x2 = int(msg.data[2])
        y2 = int(msg.data[3])

        height, width = self.depth.shape

        # Clamp bbox to image
        x1 = max(
            0,
            min(x1, width - 1)
        )

        x2 = max(
            0,
            min(x2, width)
        )

        y1 = max(
            0,
            min(y1, height - 1)
        )

        y2 = max(
            0,
            min(y2, height)
        )

        if x2 <= x1 or y2 <= y1:
            return

        # --------------------------------------------------
        # Extract depth inside bottle bbox
        # --------------------------------------------------

        region = self.depth[
            y1:y2,
            x1:x2
        ]

        valid = region[
            np.isfinite(region)
        ]

        min_depth = self.get_parameter(
            'min_depth'
        ).value

        max_depth = self.get_parameter(
            'max_depth'
        ).value

        valid = valid[
            (valid > min_depth) &
            (valid < max_depth)
        ]

        if valid.size == 0:
            return

        # --------------------------------------------------
        # --------------------------------------------------
        # Distance
        #
        # Use the central part of the bounding box so that
        # ground/background pixels near the bbox edges do
        # not determine the bottle distance.
        # --------------------------------------------------

        bbox_width = x2 - x1
        bbox_height = y2 - y1

        cx1 = x1 + int(0.30 * bbox_width)
        cx2 = x1 + int(0.70 * bbox_width)

        cy1 = y1 + int(0.20 * bbox_height)
        cy2 = y1 + int(0.75 * bbox_height)

        center_region = self.depth[
            cy1:cy2,
            cx1:cx2
        ]

        center_valid = center_region[
            np.isfinite(center_region)
        ]

        center_valid = center_valid[
            (center_valid > min_depth) &
            (center_valid < max_depth)
        ]

        if center_valid.size == 0:
            return

        goal_distance = float(
            np.median(center_valid)
        )

        # Direction
        # --------------------------------------------------

        bottle_center_x = (
            x1 + x2
        ) / 2.0

        goal_angle = math.atan2(
            bottle_center_x - self.cx,
            self.fx
        )

        # --------------------------------------------------
        # Publish navigation target
        # --------------------------------------------------

        target = Float32MultiArray()

        target.data = [
            goal_distance,
            goal_angle
        ]

        self.target_pub.publish(
            target
        )

        self.last_target_time = (
            self.get_clock().now()
        )

    # ======================================================
    # Target validity
    # ======================================================

    def validity_timer(self):

        valid = False

        if self.last_target_time is not None:

            age = (
                self.get_clock().now()
                - self.last_target_time
            ).nanoseconds / 1e9

            timeout = self.get_parameter(
                'timeout'
            ).value

            valid = age < timeout

        msg = Bool()
        msg.data = valid

        self.valid_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = BottleTarget()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
