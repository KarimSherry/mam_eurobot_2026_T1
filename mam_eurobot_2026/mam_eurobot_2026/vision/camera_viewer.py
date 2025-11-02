#!/usr/bin/env python3
# Execute this file with python3 src/mam_eurobot_2026/mam_eurobot_2026/vision/camera_viewer.py <topic name>
import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self, image_topic: str, info_topic: str | None = None):
        super().__init__('camera_viewer')

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.image_topic = image_topic
        self.info_topic = info_topic if info_topic else image_topic.rstrip('/') + '/camera_info'
        self.window_name = self.image_topic

        self.bridge = CvBridge()
        self.got_info = False
        self.frame_count = 0
        self.last_count = 0
        self.t0 = self.get_clock().now()

        self.get_logger().info(f'=== CameraViewer started ===')
        self.get_logger().info(f'Image topic: {self.image_topic}')
        self.get_logger().info(f'CameraInfo topic: {self.info_topic}')

        self.timer = self.create_timer(1.0, self._log_rate)
        self.sub_img = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.sub_info = self.create_subscription(CameraInfo, self.info_topic, self.on_info, qos)

        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            self.use_window = True
        except Exception as e:
            self.get_logger().warn(f'OpenCV window creation failed (headless env?): {e}')
            self.use_window = False

        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def on_info(self, msg: CameraInfo):
        if not self.got_info:
            self.get_logger().info(
                f'CameraInfo: {msg.width}x{msg.height}, K={[round(k,2) for k in msg.k]}'
            )
            self.got_info = True

    def on_image(self, msg: Image):
        if self.frame_count == 0:
            self.get_logger().info(
                f'Image arrived: {msg.width}x{msg.height}, encoding={msg.encoding}, '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            )

        try:
            if msg.encoding == 'rgb8':
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        if self.use_window:
            try:
                cv2.imshow(self.window_name, cv_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rclpy.shutdown()
            except cv2.error as e:
                self.get_logger().warn(f'cv2.imshow failed (headless?): {e}')
                self.use_window = False
        else:
            if self.frame_count % 60 == 0:
                out = '/tmp/camera_viewer_sample.jpg'
                cv2.imwrite(out, cv_img)
                self.get_logger().info(f'Wrote sample frame to {out}')

        self.frame_count += 1

    def on_shutdown(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    def _log_rate(self):
        now = self.get_clock().now()
        dt = (now - self.t0).nanoseconds / 1e9
        inc = self.frame_count - self.last_count
        hz = inc / dt if dt > 0 else 0.0
        self.get_logger().info(
            f'Recv rate ~ {hz:.2f} Hz (last {dt:.2f}s, +{inc} frames, total {self.frame_count})'
        )
        self.t0 = now
        self.last_count = self.frame_count
        if inc == 0:
            self.get_logger().warn('No frames received in the last interval.')

def parse_args():
    argv = remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(
        description='CameraViewer: Display image from a ROS2 topic'
    )
    parser.add_argument(
        'image_topic',
        help='Image topic to subscribe (required, e.g. /camera/image_raw)'
    )
    parser.add_argument(
        '--info-topic',
        help='CameraInfo topic (default: <image_topic>/camera_info)'
    )
    return parser.parse_args(argv[1:])

def main():
    args = parse_args()
    rclpy.init()
    node = CameraViewer(image_topic=args.image_topic, info_topic=args.info_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
