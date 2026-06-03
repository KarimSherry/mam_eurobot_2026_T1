#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from mam_eurobot_2026.vision.aruco_utils import (
    create_detector_parameters,
    detect_markers,
    estimate_marker_pose,
    get_aruco_dictionary,
    invert_transform,
    load_intrinsics,
    load_yaml_config,
    lookup_camera_config,
    pose_from_transform,
    transform_from_rvec_tvec,
    transform_from_tf_msg,
    transform_from_xyzrpy,
)


class RobotPoseFromBeacon(Node):
    def __init__(self):
        super().__init__("robot_pose_from_beacon")

        self.declare_parameter("config_yaml", "vision_settings.yaml")
        self.declare_parameter("calibration_pkl", "")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pose_topic", "/robot_pose_vision")
        self.declare_parameter("marker_id", -1)
        self.declare_parameter("marker_length_m", 0.0)
        self.declare_parameter("base_to_marker_xyzrpy", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("show_debug_image", True)
        self.declare_parameter("publish_tf", True)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        if not self.image_topic:
            raise ValueError("Parameter 'image_topic' is required.")

        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.camera_frame_param = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.show_debug = bool(self.get_parameter("show_debug_image").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        cfg_path = self.get_parameter("config_yaml").get_parameter_value().string_value
        calibration_pkl = self.get_parameter("calibration_pkl").get_parameter_value().string_value
        cfg = load_yaml_config(cfg_path, "mam_eurobot_2026.vision", "vision_settings.yaml")
        global_cfg = cfg.get("global", {})
        self.camera_cfg = lookup_camera_config(cfg, self.image_topic)

        image_width = int(global_cfg.get("image_width", 640))
        image_height = int(global_cfg.get("image_height", 480))
        hfov_deg = float(global_cfg.get("horizontal_fov_deg", 60.0))
        dict_name = str(global_cfg.get("aruco_dictionary", "DICT_4X4_50"))

        self.camera_matrix, self.dist_coeffs, self.intrinsics_source = load_intrinsics(
            calibration_pkl,
            image_width,
            image_height,
            hfov_deg,
        )
        self.aruco_dict = get_aruco_dictionary(dict_name)
        self.parameters = create_detector_parameters()

        robot_cfg = cfg.get("robot", {})
        marker_id_param = int(self.get_parameter("marker_id").value)
        marker_len_param = float(self.get_parameter("marker_length_m").value)
        base_to_marker_param = list(self.get_parameter("base_to_marker_xyzrpy").value)
        self.marker_id = marker_id_param if marker_id_param >= 0 else int(robot_cfg.get("marker_id", -1))
        self.marker_length_m = (
            marker_len_param if marker_len_param > 0.0 else float(robot_cfg.get("marker_length_m", 0.0))
        )
        self.base_to_marker_xyzrpy = (
            base_to_marker_param
            if any(abs(float(value)) > 1e-12 for value in base_to_marker_param)
            else list(robot_cfg.get("base_to_marker_xyzrpy", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        )
        if self.marker_id < 0 or self.marker_length_m <= 0.0:
            raise ValueError("Robot marker configuration is missing. Set marker_id and marker_length_m.")

        self.camera_frame = (
            self.camera_frame_param
            or str(self.camera_cfg.get("frame_id") or self.camera_cfg.get("camera_frame") or "")
        )
        if not self.camera_frame:
            self.camera_frame = f"{self.camera_cfg.get('name', 'camera')}_optical_frame"

        self.t_base_marker = transform_from_xyzrpy(self.base_to_marker_xyzrpy)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.image_topic, self._image_callback, qos)

        self.get_logger().info(
            f"RobotPoseFromBeacon on {self.image_topic}, camera_frame={self.camera_frame}, "
            f"world_frame={self.world_frame}, base_frame={self.base_frame}"
        )
        self.get_logger().info(
            f"Robot marker id={self.marker_id}, length={self.marker_length_m:.3f} m, "
            f"intrinsics={self.intrinsics_source}"
        )

    def _image_callback(self, img_msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        corners, ids, _ = detect_markers(frame, self.aruco_dict, self.parameters)
        if ids is None:
            return

        ids_flat = ids.flatten()
        hit = np.where(ids_flat == self.marker_id)[0]
        if len(hit) == 0:
            return

        image_points = corners[int(hit[0])].reshape(4, 2).astype(np.float32)
        rvec, tvec = estimate_marker_pose(
            image_points,
            self.marker_length_m,
            self.camera_matrix,
            self.dist_coeffs,
        )
        if rvec is None or tvec is None:
            return

        try:
            tf_cam = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"TF lookup failed ({self.world_frame} -> {self.camera_frame}): {exc}"
            )
            return

        t_world_camera = transform_from_tf_msg(tf_cam.transform)
        t_camera_marker = transform_from_rvec_tvec(rvec, tvec)
        t_marker_base = invert_transform(self.t_base_marker)
        t_world_base = t_world_camera @ t_camera_marker @ t_marker_base

        pose_msg = PoseStamped()
        pose_msg.header.stamp = img_msg.header.stamp
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose = pose_from_transform(t_world_base)
        self.pose_pub.publish(pose_msg)

        if self.publish_tf:
            tf_out = TransformStamped()
            tf_out.header = pose_msg.header
            tf_out.child_frame_id = self.base_frame
            tf_out.transform.translation.x = pose_msg.pose.position.x
            tf_out.transform.translation.y = pose_msg.pose.position.y
            tf_out.transform.translation.z = pose_msg.pose.position.z
            tf_out.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(tf_out)

        if self.show_debug:
            overlay = frame.copy()
            cv2.drawFrameAxes(
                overlay,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.marker_length_m * 0.75,
            )
            center = np.mean(image_points, axis=0).astype(int)
            world_pose = pose_msg.pose
            cv2.putText(
                overlay,
                f"robot ({world_pose.position.x:.2f}, {world_pose.position.y:.2f})",
                (int(center[0]), int(center[1])),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )
            cv2.imshow("robot_pose_from_beacon", overlay)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rclpy.shutdown()


def main():
    rclpy.init()
    node = RobotPoseFromBeacon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
