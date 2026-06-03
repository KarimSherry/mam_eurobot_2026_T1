#!/usr/bin/env python3
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovariance, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

import cv2
import numpy as np

from mam_eurobot_2026.vision.aruco_utils import (
    create_detector_parameters,
    detect_markers,
    estimate_marker_pose,
    get_aruco_dictionary,
    load_intrinsics,
    load_yaml_config,
    lookup_camera_config,
    rvec_tvec_to_pose,
)


class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter("config_yaml", "vision_settings.yaml")
        self.declare_parameter("calibration_pkl", "")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("show_debug_image", True)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        if not self.image_topic:
            raise ValueError("Parameter 'image_topic' is required. Pass via --ros-args -p image_topic:=<topic>.")

        cfg_path = self.get_parameter("config_yaml").get_parameter_value().string_value
        self.show_debug = bool(self.get_parameter("show_debug_image").value)
        self.calibration_pkl = self.get_parameter("calibration_pkl").get_parameter_value().string_value

        cfg = load_yaml_config(cfg_path, "mam_eurobot_2026.vision", "vision_settings.yaml")
        global_cfg = cfg.get("global", {})
        self.camera_cfg = lookup_camera_config(cfg, self.image_topic)
        self.camera_frame = str(
            self.camera_cfg.get("frame_id")
            or self.camera_cfg.get("camera_frame")
            or f"{self.camera_cfg.get('name', 'camera')}_optical_frame"
        )

        image_width = int(global_cfg.get("image_width", 640))
        image_height = int(global_cfg.get("image_height", 480))
        hfov_deg = float(global_cfg.get("horizontal_fov_deg", 60.0))
        dict_name = str(global_cfg.get("aruco_dictionary", "DICT_4X4_50"))

        self.camera_matrix, self.dist_coeffs, self.intrinsics_source = load_intrinsics(
            self.calibration_pkl,
            image_width,
            image_height,
            hfov_deg,
        )
        self.aruco_dict = get_aruco_dictionary(dict_name)
        self.parameters = create_detector_parameters()
        self.aruco_sizes, self.generic_aruco_len, self.allowed_ids = self._parse_arucos(cfg)
        self.default_marker_len = float(self.camera_cfg.get("marker_length_m", 0.13))

        self.bridge = CvBridge()
        qos = QoSProfile(depth=1)
        self.pub = self.create_publisher(Detection3DArray, "/aruco/detections", qos)
        self.create_subscription(Image, self.image_topic, self._image_callback, qos)

        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info(f"Publishing detections in frame: {self.camera_frame}")
        self.get_logger().info(f"Intrinsics source: {self.intrinsics_source}")

    @staticmethod
    def _parse_arucos(cfg: dict) -> tuple[dict[int, float], float | None, set[int]]:
        sizes: dict[int, float] = {}
        generic_len = None
        allowed = set()
        for aru in cfg.get("arucos", []):
            if "marker_length_m" not in aru:
                continue
            try:
                length = float(aru["marker_length_m"])
            except (TypeError, ValueError):
                continue
            if "id" in aru:
                try:
                    marker_id = int(aru["id"])
                except (TypeError, ValueError):
                    continue
                sizes[marker_id] = length
                allowed.add(marker_id)
            else:
                generic_len = length
        return sizes, generic_len, allowed

    def _aruco_length_for_id(self, marker_id: int) -> float:
        if marker_id in self.aruco_sizes:
            return self.aruco_sizes[marker_id]
        if self.generic_aruco_len is not None:
            return self.generic_aruco_len
        return self.default_marker_len

    def _image_callback(self, img_msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        det_array = self._detect_markers(img_msg, frame)
        self.pub.publish(det_array)

    def _detect_markers(self, img_msg: Image, frame: np.ndarray) -> Detection3DArray:
        debug_frame = frame.copy() if self.show_debug else None
        corners, ids, _ = detect_markers(frame, self.aruco_dict, self.parameters)

        det_array = Detection3DArray()
        det_array.header = Header()
        det_array.header.stamp = img_msg.header.stamp
        det_array.header.frame_id = img_msg.header.frame_id or self.camera_frame

        if ids is None or len(ids) == 0:
            return det_array

        for corner, marker_id in zip(corners, ids.flatten()):
            marker_id = int(marker_id)
            if self.allowed_ids and marker_id not in self.allowed_ids:
                continue

            marker_len = self._aruco_length_for_id(marker_id)
            image_points = corner.reshape(4, 2).astype(np.float32)
            rvec, tvec = estimate_marker_pose(
                image_points,
                marker_len,
                self.camera_matrix,
                self.dist_coeffs,
            )
            if rvec is None or tvec is None:
                continue

            if debug_frame is not None:
                try:
                    cv2.drawFrameAxes(
                        debug_frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        marker_len * 0.5,
                    )
                    cv2.polylines(
                        debug_frame,
                        [image_points.astype(np.int32)],
                        True,
                        (0, 255, 255),
                        2,
                    )
                    center = np.mean(image_points, axis=0).astype(int)
                    cv2.putText(
                        debug_frame,
                        f"id={marker_id}",
                        (int(center[0]), int(center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"draw debug overlay failed: {exc}")

            pose = rvec_tvec_to_pose(rvec, tvec)
            pose_cov = PoseWithCovariance()
            pose_cov.pose = pose
            pose_cov.covariance = [0.0] * 36

            det = Detection3D()
            det.header = det_array.header
            det.results = [
                ObjectHypothesisWithPose(
                    hypothesis=ObjectHypothesis(class_id=str(marker_id), score=1.0),
                    pose=pose_cov,
                )
            ]

            bbox = BoundingBox3D()
            bbox.center = pose
            bbox.size = Vector3(x=float(marker_len), y=float(marker_len), z=0.001)
            det.bbox = bbox
            det_array.detections.append(det)

        if debug_frame is not None:
            cv2.imshow("aruco_detector", debug_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("Quit requested (q) from debug window; shutting down...")
                rclpy.shutdown()

        return det_array


def main():
    rclpy.init()
    node = ArucoDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
