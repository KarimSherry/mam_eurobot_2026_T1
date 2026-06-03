#!/usr/bin/env python3
import time
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import StaticTransformBroadcaster

from mam_eurobot_2026.vision.aruco_utils import (
    create_detector_parameters,
    detect_markers,
    estimate_marker_pose,
    get_aruco_dictionary,
    invert_transform,
    load_intrinsics,
    load_yaml_config,
    pose_from_transform,
    rotation_matrix_to_quaternion,
    transform_from_rvec_tvec,
    transform_from_xyzrpy,
)


class WorldToStaticTopcamera(Node):
    def __init__(self):
        super().__init__("world_to_topcamera")

        self.declare_parameter("exit_on_complete", False)
        self.declare_parameter("require_both_cameras", False)
        self.declare_parameter("config_yaml", "vision_settings.yaml")
        self.declare_parameter("calibration_pkl", "")
        self.declare_parameter("world_frame", "map")

        cfg_path = self.get_parameter("config_yaml").get_parameter_value().string_value
        calibration_pkl = self.get_parameter("calibration_pkl").get_parameter_value().string_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value

        cfg = load_yaml_config(cfg_path, "mam_eurobot_2026.vision", "vision_settings.yaml")
        if "global" not in cfg:
            raise ValueError("Config 'global' is missing")
        if "cameras" not in cfg or not isinstance(cfg["cameras"], list) or not cfg["cameras"]:
            raise ValueError("Config 'cameras' is missing or empty")

        global_cfg = cfg.get("global", {})
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
        self.det_params = create_detector_parameters()

        self.cams: dict[str, dict[str, Any]] = {}
        for camera in cfg.get("cameras", []):
            name = str(camera["name"])
            if name in self.cams:
                raise ValueError(f"Duplicate camera name: {name}")
            self.cams[name] = camera

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pose_pubs: dict[str, Any] = {}
        self.static_tfb = StaticTransformBroadcaster(self)
        self.bridge = CvBridge()
        self.done_flags = {name: False for name in self.cams.keys()}
        self.exit_on_complete = bool(self.get_parameter("exit_on_complete").value)
        self.require_all = bool(self.get_parameter("require_both_cameras").value)

        for name, camera in self.cams.items():
            pose_topic, optical_frame = self._names_for_camera(camera)
            self.pose_pubs[name] = self.create_publisher(PoseStamped, pose_topic, pose_qos)
            self.create_subscription(
                Image,
                camera["image_topic"],
                lambda msg, n=name: self._image_callback(msg, n),
                sensor_qos,
            )
            self.get_logger().info(
                f"Subscribe: {camera['image_topic']} -> Pose pub: {pose_topic}, frame: {optical_frame}"
            )

        self.get_logger().info(f"Intrinsics source: {self.intrinsics_source}")

    @staticmethod
    def _names_for_camera(camera_cfg: dict[str, Any]) -> tuple[str, str]:
        cam_name = str(camera_cfg["name"])
        pose_topic = f"/{cam_name}/pose_world"
        optical_frame = str(
            camera_cfg.get("frame_id")
            or camera_cfg.get("camera_frame")
            or f"{cam_name}_optical_frame"
        )
        return pose_topic, optical_frame

    def _finish_node(self):
        if not self.exit_on_complete:
            return
        if self.require_all:
            should_exit = all(self.done_flags.values())
        else:
            should_exit = any(self.done_flags.values())
        if should_exit:
            time.sleep(0.2)
            self.get_logger().info("Camera pose solve complete. Exiting.")
            rclpy.shutdown()

    def _image_callback(self, msg: Image, cam_name: str):
        if self.done_flags[cam_name]:
            return

        camera = self.cams[cam_name]
        marker_id = int(camera["marker_id"])
        marker_len = float(camera["marker_length_m"])
        world_marker = camera["world_marker_xyzrpy"]

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, _ = detect_markers(frame, self.aruco_dict, self.det_params)
        if ids is None:
            return

        ids_flat = ids.flatten()
        hit = np.where(ids_flat == marker_id)[0]
        if len(hit) == 0:
            return

        image_points = corners[int(hit[0])].reshape(4, 2).astype(np.float32)
        rvec, tvec = estimate_marker_pose(
            image_points,
            marker_len,
            self.camera_matrix,
            self.dist_coeffs,
        )
        if rvec is None or tvec is None:
            return

        t_camera_marker = transform_from_rvec_tvec(rvec, tvec)
        t_world_marker = transform_from_xyzrpy(world_marker)
        t_marker_camera = invert_transform(t_camera_marker)
        t_world_camera = t_world_marker @ t_marker_camera

        pose_topic, optical_frame = self._names_for_camera(camera)
        pose = pose_from_transform(t_world_camera)
        quat = rotation_matrix_to_quaternion(t_world_camera[:3, :3])

        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose = pose
        self.pose_pubs[cam_name].publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = optical_frame
        tf_msg.transform.translation.x = float(t_world_camera[0, 3])
        tf_msg.transform.translation.y = float(t_world_camera[1, 3])
        tf_msg.transform.translation.z = float(t_world_camera[2, 3])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.static_tfb.sendTransform(tf_msg)

        self.get_logger().info(
            f"[{cam_name}] Solved world -> {optical_frame}: "
            f"({t_world_camera[0, 3]:.3f}, {t_world_camera[1, 3]:.3f}, {t_world_camera[2, 3]:.3f})"
        )
        self.get_logger().info(f"[{cam_name}] Pose topic published on {pose_topic}")

        self.done_flags[cam_name] = True
        self._finish_node()


def main():
    rclpy.init()
    node = WorldToStaticTopcamera()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
