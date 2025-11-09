#!/usr/bin/env python3
# Execution example : ros2 run mam_eurobot_2026 color_detector.py --image-topic /top_camera/image_2
import sys
import argparse
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.utilities import remove_ros_args

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ColorDetector(Node):
    def __init__(self, image_topic: str, reliable: bool):
        super().__init__('color_detector')

        self.declare_parameter('h_min', 40)
        self.declare_parameter('s_min', 170)
        self.declare_parameter('v_min', 170)
        self.declare_parameter('h_max', 70)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_max', 255)

        self.declare_parameter('erode', 2)
        self.declare_parameter('dilate', 2)
        self.declare_parameter('min_area', 200)  

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos)

        self.get_logger().info(f'Subscribed to: {image_topic}')
        self.get_logger().info(
            'Override HSV via --ros-args, e.g. -p h_min:=35 -p h_max:=85 -p s_min:=50 -p v_min:=50'
        )

    def _get_bounds(self):
        h_min = int(self.get_parameter('h_min').value)
        s_min = int(self.get_parameter('s_min').value)
        v_min = int(self.get_parameter('v_min').value)
        h_max = int(self.get_parameter('h_max').value)
        s_max = int(self.get_parameter('s_max').value)
        v_max = int(self.get_parameter('v_max').value)
        lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
        return lower, upper

    def _morph(self, mask: np.ndarray) -> np.ndarray:
        erode_iter = max(0, int(self.get_parameter('erode').value))
        dilate_iter = max(0, int(self.get_parameter('dilate').value))
        if erode_iter > 0:
            mask = cv2.erode(mask, None, iterations=erode_iter)
        if dilate_iter > 0:
            mask = cv2.dilate(mask, None, iterations=dilate_iter)
        return mask

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge conversion failed: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower, upper = self._get_bounds()
        mask = cv2.inRange(hsv, lower, upper) 
        mask = self._morph(mask)


        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 200:  # remove small noises
                continue

            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            # get the point which has smallest y
            top_point = box[np.argmin(box[:, 1])]

            # visualize the point
            # cv2.circle(mask, tuple(top_point), 5, (255, 255, 255), -1)
            # print("Top point:", top_point)

        cv2.imshow("ColorDetector: binary mask (green)", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Quit requested (q). Shutting down...')
            rclpy.shutdown()


def parse_cli_args():
    cleaned = remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(description='Detect green-ish regions and show binary mask.')
    parser.add_argument('--image-topic', required=True,
                        help='sensor_msgs/msg/Image topic (e.g., /camera/image_raw)')
    parser.add_argument('--reliable', action='store_true',
                        help='Use RELIABLE QoS (default: BEST_EFFORT)')
    return parser.parse_args(cleaned[1:])


def main():
    rclpy.init(args=sys.argv)
    args = parse_cli_args()
    node = ColorDetector(image_topic=args.image_topic, reliable=args.reliable)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
