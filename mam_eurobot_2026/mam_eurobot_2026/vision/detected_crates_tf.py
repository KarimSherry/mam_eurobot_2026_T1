#!/usr/bin/env python3
import copy
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_geometry_msgs  # noqa: F401
import tf2_ros

from geometry_msgs.msg import Pose, PoseStamped
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

# this node transforms /aruco/detections into a target world frame.
# It works for both robot-mounted and fixed beacon cameras as long as the
# detection header frame_id matches a frame available in TF.
class DetectedCratesTF(Node):
    def __init__(self) -> None:
        super().__init__("detected_crates_tf")

        self._source_topic = str(self.declare_parameter("source_topic", "/aruco/detections").value)
        self._output_topic = str(self.declare_parameter("output_topic", "/detected_crates").value)
        self._target_frame = str(self.declare_parameter("target_frame", "map").value)
        self._tf_timeout_sec = float(self.declare_parameter("tf_timeout_sec", 0.2).value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(Detection3DArray, self._output_topic, 10)
        self.create_subscription(Detection3DArray, self._source_topic, self._on_detections, 10)

        self.get_logger().info(
            f"DetectedCratesTF listening on {self._source_topic}, publishing {self._output_topic} in {self._target_frame}"
        )

    def _transform_pose(self, pose: Pose, header) -> Optional[Pose]:
        source_frame = header.frame_id
        if not source_frame:
            return None
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.header.frame_id = source_frame
        pose_stamped.header.stamp = header.stamp if header.stamp.sec or header.stamp.nanosec else Time().to_msg()
        pose_stamped.pose = pose
        try:
            out = self._tf_buffer.transform(
                pose_stamped,
                self._target_frame,
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"TF transform failed {header.frame_id}->{self._target_frame}: {exc}")
            return None
        return out.pose

    def _on_detections(self, msg: Detection3DArray) -> None:
        if not msg.header.frame_id:
            self.get_logger().warn("Detection3DArray has empty frame_id; skipping.")
            return

        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._target_frame

        for det in msg.detections:
            new_results = []
            for res in det.results:
                transformed_pose = self._transform_pose(res.pose.pose, det.header)
                if transformed_pose is None:
                    continue
                pose_cov = copy.deepcopy(res.pose)
                pose_cov.pose = transformed_pose
                new_results.append(ObjectHypothesisWithPose(hypothesis=res.hypothesis, pose=pose_cov))

            if not new_results:
                continue

            det_out = Detection3D()
            det_out.header = out.header
            det_out.results = new_results

            if det.bbox:
                det_out.bbox = copy.deepcopy(det.bbox)
                transformed_center = self._transform_pose(det.bbox.center, det.header)
                if transformed_center is not None:
                    det_out.bbox.center = transformed_center

            out.detections.append(det_out)

        if out.detections:
            self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = DetectedCratesTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
