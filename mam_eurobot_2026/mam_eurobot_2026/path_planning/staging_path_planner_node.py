#!/usr/bin/env python3
import math
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path as NavPath
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from mam_eurobot_2026.vision.aruco_utils import quaternion_from_euler


@dataclass
class StagingPose:
    crate_id: str
    x: float
    y: float
    yaw: float


class StagingPathPlanner(Node):
    def __init__(self) -> None:
        super().__init__("staging_path_planner")

        self.declare_parameter("objects_yaml", "")
        self.declare_parameter("plan_rate_hz", 1.0)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("planner_action_name", "/compute_path_to_pose")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("output_path_topic", "/planned_path")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_frame", "map")

        self._objects_yaml = self.get_parameter("objects_yaml").get_parameter_value().string_value
        self._plan_rate_hz = self.get_parameter("plan_rate_hz").value
        self._tf_timeout_sec = self.get_parameter("tf_timeout_sec").value
        self._planner_action_name = self.get_parameter("planner_action_name").value
        self._planner_id = self.get_parameter("planner_id").value
        self._output_path_topic = self.get_parameter("output_path_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._global_frame = self.get_parameter("global_frame").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._action_client = ActionClient(self, ComputePathToPose, self._planner_action_name)
        self._path_pub = self.create_publisher(NavPath, self._output_path_topic, 10)

        self._last_target_id = None
        self._goal_in_flight = False

        period = 1.0 / max(self._plan_rate_hz, 0.001)
        self._timer = self.create_timer(period, self._plan_timer_cb)

        if not self._objects_yaml:
            self.get_logger().warn("objects_yaml parameter is empty; no staging poses will be loaded")

    def _plan_timer_cb(self) -> None:
        if self._goal_in_flight:
            return

        current_pose = self._get_current_pose()
        if current_pose is None:
            return

        staging_poses = self._load_staging_poses()
        if not staging_poses:
            return

        target = self._select_nearest_pose(current_pose, staging_poses)
        if target is None:
            return

        target_changed = target.crate_id != self._last_target_id
        self._last_target_id = target.crate_id

        goal_pose = self._pose_from_staging(target)
        self._request_plan(current_pose, goal_pose)

        if target_changed:
            self.get_logger().info(f"Selected new target crate: {target.crate_id}")

    def _get_current_pose(self) -> PoseStamped | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=float(self._tf_timeout_sec)),
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF lookup failed ({self._global_frame} -> {self._base_frame}): {exc}")
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._global_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _load_staging_poses(self) -> list[StagingPose]:
        if not self._objects_yaml:
            return []

        yaml_path = Path(self._objects_yaml)
        if not yaml_path.is_file():
            self.get_logger().warn(f"objects_yaml not found: {yaml_path}")
            return []

        try:
            data = yaml.safe_load(yaml_path.read_text()) or {}
        except yaml.YAMLError as exc:
            self.get_logger().warn(f"Failed to parse objects_yaml: {exc}")
            return []

        frame_id = data.get("frame_id", self._global_frame)
        if frame_id != self._global_frame:
            self.get_logger().warn(
                f"objects_yaml frame_id '{frame_id}' does not match global_frame '{self._global_frame}'"
            )

        poses = []
        for crate in data.get("crates", []):
            staging = crate.get("staging") or {}
            try:
                poses.append(
                    StagingPose(
                        crate_id=str(crate.get("id", "unknown")),
                        x=float(staging["x"]),
                        y=float(staging["y"]),
                        yaw=float(staging.get("yaw", 0.0)),
                    )
                )
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warn(f"Invalid staging entry in objects_yaml: {exc}")

        return poses

    def _select_nearest_pose(self, current_pose: PoseStamped, poses: list[StagingPose]) -> StagingPose | None:
        if not poses:
            return None

        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y

        return min(poses, key=lambda p: math.hypot(p.x - current_x, p.y - current_y))

    def _pose_from_staging(self, staging: StagingPose) -> PoseStamped:
        quat = quaternion_from_euler(0.0, 0.0, staging.yaw)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._global_frame
        pose.pose.position.x = staging.x
        pose.pose.position.y = staging.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def _request_plan(self, start_pose: PoseStamped, goal_pose: PoseStamped) -> None:
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"Planner action server not available: {self._planner_action_name}")
            return

        goal = ComputePathToPose.Goal()
        goal.start = start_pose
        goal.goal = goal_pose
        goal.use_start = True
        goal.planner_id = self._planner_id

        self._goal_in_flight = True
        send_future = self._action_client.send_goal_async(goal)
        send_future.add_done_callback(self._handle_goal_response)

    def _handle_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Planner rejected path request")
            self._goal_in_flight = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_result)

    def _handle_result(self, future) -> None:
        self._goal_in_flight = False
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Planner failed with status {result.status}")
            return

        path = result.result.path
        if not path.poses:
            self.get_logger().warn("Planner returned an empty path")
            return

        self._path_pub.publish(path)


def main() -> None:
    rclpy.init()
    node = StagingPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
