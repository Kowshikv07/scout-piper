import cv2
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO


class BottleDetector(Node):

    def __init__(self):
        super().__init__('bottle_detector')

        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('class_names', ['bottle'])
        self.declare_parameter('conf', 0.05)
        self.declare_parameter('imgsz', 960)
        self.declare_parameter('device', '')
        self.declare_parameter('process_every_n', 2)
        self.declare_parameter('confirm_frames', 2)
        self.declare_parameter('publish_debug', True)

        self.bridge = CvBridge()
        self.model = YOLO(self.get_parameter('model').value)

        wanted = set(self.get_parameter('class_names').value)

        self.class_ids = [
            i for i, n in self.model.names.items()
            if n in wanted
        ]

        if not self.class_ids:
            self.get_logger().error(
                f'None of {wanted} found in model classes: '
                f'{self.model.names}'
            )

        self.get_logger().info(
            f'Detecting classes: {self.class_ids}'
        )

        self.frame_count = 0
        self.hits = 0
        self.last_boxes = []

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            1
        )

        self.pub = self.create_publisher(
            Bool,
            '/bottle_in_view',
            10
        )

        self.pos_pub = self.create_publisher(
            Point,
            '/bottle_position',
            10
        )

        self.bbox_pub = self.create_publisher(
            Float32MultiArray,
            '/bottle_bbox',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/bottle_debug_image',
            10
        )

    def image_callback(self, msg):

        self.frame_count += 1

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        h, w = frame.shape[:2]

        if (
            self.frame_count
            % self.get_parameter('process_every_n').value
            == 0
        ):

            res = self.model.predict(
                frame,
                conf=self.get_parameter('conf').value,
                imgsz=self.get_parameter('imgsz').value,
                device=self.get_parameter('device').value or None,
                classes=self.class_ids,
                verbose=False,
            )[0]

            self.last_boxes = [
                (
                    *map(int, b.xyxy[0].tolist()),
                    float(b.conf[0])
                )
                for b in res.boxes
            ]

            for b in res.boxes:
                cls_id = int(b.cls[0])
                conf = float(b.conf[0])
                name = self.model.names[cls_id]

                self.get_logger().info(
                    f'Detection: {name} '
                    f'({cls_id}) '
                    f'confidence={conf:.3f}'
                )

            self.hits = (
                self.hits + 1
                if self.last_boxes
                else 0
            )

        confirmed = (
            self.hits
            >= self.get_parameter('confirm_frames').value
        )

        out = Bool()
        out.data = bool(confirmed)
        self.pub.publish(out)

        best = max(
            self.last_boxes,
            key=lambda b: b[4],
            default=None
        )

        if confirmed and best is not None:

            x1, y1, x2, y2, _ = best

            bbox = Float32MultiArray()
            bbox.data = [
                float(x1),
                float(y1),
                float(x2),
                float(y2)
            ]

            self.bbox_pub.publish(bbox)

            p = Point()

            p.x = float(
                (x1 + x2) / 2.0 / w * 2.0 - 1.0
            )

            p.y = float(
                (y1 + y2) / 2.0 / h * 2.0 - 1.0
            )

            p.z = float(
                (x2 - x1) * (y2 - y1)
            ) / float(w * h)

            self.pos_pub.publish(p)

        if self.get_parameter('publish_debug').value:

            dbg = frame.copy()

            for x1, y1, x2, y2, c in self.last_boxes:

                color = (
                    (0, 255, 0)
                    if confirmed
                    else (0, 200, 255)
                )

                cv2.rectangle(
                    dbg,
                    (x1, y1),
                    (x2, y2),
                    color,
                    2
                )

                cv2.putText(
                    dbg,
                    f'bottle {c:.2f}',
                    (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2
                )

            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    dbg,
                    encoding='bgr8'
                )
            )


def main():

    rclpy.init()

    node = BottleDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
