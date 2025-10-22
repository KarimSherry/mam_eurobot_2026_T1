#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class FrontCamViewer(Node):
    def __init__(self):
        super().__init__('front_cam_viewer')

        # QoS
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.bridge = CvBridge()
        self.got_info = False
        self.frame_count = 0
        self.frame_count = 0
        self.last_count = 0
        self.t0 = self.get_clock().now()
        # publish recieve rate every 1 second
        self.timer = self.create_timer(1.0, self._log_rate)

        self.sub_img = self.create_subscription(
            Image, '/front_camera', self.on_image, qos)

        self.sub_info = self.create_subscription(
            CameraInfo, '/front_camera/camera_info', self.on_info, qos)

        # OpenCVウィンドウ
        try:
            cv2.namedWindow('front_camera', cv2.WINDOW_NORMAL)
            self.use_window = True
        except Exception as e:
            self.get_logger().warn(f'OpenCV window creation failed (headless env?): {e}')
            self.use_window = False
            

        # register correct shutdown handler 
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def on_info(self, msg: CameraInfo):
        if not self.got_info:
            self.get_logger().info(
                f'CameraInfo: {msg.width}x{msg.height}, '
                f'K={[round(k,2) for k in msg.k]}')
            self.got_info = True

    def on_image(self, msg: Image):
        if self.frame_count == 0:
            self.get_logger().info(
                f'Image arrived: {msg.width}x{msg.height}, encoding={msg.encoding}, '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}'
            )

        # ---cv bridge transform and display---
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
                cv2.imshow('front_camera', cv_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rclpy.shutdown()
            except cv2.error as e:
                self.get_logger().warn(f'cv2.imshow failed (headless?): {e}')
                self.use_window = False
        else:
            if self.frame_count % 60 == 0:
                cv2.imwrite('/tmp/front_camera_sample.jpg', cv_img)
        # --- display process ends ---

        self.frame_count += 1
        if self.frame_count % 60 == 0:
            self.get_logger().info(f'Frames received: {self.frame_count}')

    def on_shutdown(self):
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
    def _log_rate(self):
        now = self.get_clock().now()
        dt = (now - self.t0).nanoseconds / 1e9
        new = self.frame_count
        inc = new - self.last_count
        hz = inc / dt if dt > 0 else 0.0
        self.get_logger().info(f'Recv rate ~ {hz:.2f} Hz (last {dt:.2f}s, +{inc} frames, total {new})')
        # refresh window
        self.t0 = now
        self.last_count = new
        if inc == 0:
            self.get_logger().warn('No frames received in the last interval.')  

def main():
    rclpy.init()
    node = FrontCamViewer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
