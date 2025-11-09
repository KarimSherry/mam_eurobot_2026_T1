#!/usr/bin/env python3
import os
import time
import math
from typing import Dict, Any, List, Tuple

import yaml
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf_transformations as tft
from pathlib import Path
from importlib.resources import files 


"""
This node should executed only once
calculate transformation from world to each camera and broadcast it
we need; sudo apt install ros-humble-tf-transformations
Execute this node with; 
ros2 run mam_eurobot_2026 world_to_topcamera.py 
"""

"""
issue i faced in testing this node was:
Path issue; i took force solution 
aruco marker coordinate issue; aruco on floor was rotated from the world frame by -π/2
parameter K issue; In SIM we can assume fx = fy
"""

def _try_open_yaml(path_str: str):
    """絶対パス or CWD相対パスで見つかれば開いて dict を返す。なければ None。"""
    import yaml
    p = Path(path_str)
    # 絶対パス
    if p.is_absolute() and p.exists():
        with p.open('r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    # CWD相対
    cand = Path.cwd() / p
    if cand.exists():
        with cand.open('r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    return None


def _load_cfg_from_package(pkg_modpath: str, filename: str):
    """
    パッケージ内（例：mam_eurobot_2026.vision）の filename を開く。
    成功時は dict、失敗時は FileNotFoundError。
    """
    import yaml
    path = files(pkg_modpath).joinpath(filename)   # 例: mam_eurobot_2026/vision/vision_settings.yaml
    if not path.is_file():
        raise FileNotFoundError(f"Config not bundled: {pkg_modpath}/{filename}")
    with path.open('r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def compute_intrinsics_from_fov(width: int, height: int, hfov_rad: float) -> Tuple[np.ndarray, np.ndarray]:
    """HFOVと画像サイズからKを計算。歪みDは0（Sim前提）。"""
    fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
    # fy = fx * (height / width)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    K = np.array([[fx, 0.0, cx],
                  [0.0, fy, cy],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.zeros((5, 1), dtype=np.float64)
    return K, D


def get_aruco_dict(name: str):
    table = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    }
    key = table.get(name, cv2.aruco.DICT_4X4_50)
    return cv2.aruco.getPredefinedDictionary(key)


def names_for_camera(cam_name: str) -> Tuple[str, str, str]:
    """Pose出力/フレーム名/ログタグを整備。"""
    pose_topic = f"/{cam_name}/pose_world"
    optical_frame = f"{cam_name}_optical_frame"
    log_tag = f"[{cam_name}]"
    return pose_topic, optical_frame, log_tag


class WorldToStaticTopcamera(Node):
    """
    目的: ArUcoが写った画像から各カメラの world→camera を推定。
    - 各カメラは YAML に定義
    - 見えた時点でそのカメラの姿勢を確定し、/tf_static と Pose をpublish
    - 全カメラ分完了したら終了（exit_on_complete=true時）
    """

    def __init__(self):
        super().__init__("world_to_topcamera")

        # ---- パラメータ ----
        self.declare_parameter("exit_on_complete", True)       # 全部そろったら終了
        self.declare_parameter("require_both_cameras", True)   # YAML列挙の全カメラが必須か
        self.declare_parameter("config_yaml", "vision_settings.yaml")

        from pathlib import Path
        cfg_name_or_path = self.get_parameter("config_yaml").get_parameter_value().string_value

        cfg = _try_open_yaml(cfg_name_or_path)  # 絶対 or CWD相対が来たらそれを優先
        if cfg is None:
            fname = Path(cfg_name_or_path).name if cfg_name_or_path else "vision_settings.yaml"
            cfg = _load_cfg_from_package("mam_eurobot_2026.vision", fname)
            self.get_logger().info(f"Loaded config from package data: mam_eurobot_2026/vision/{fname}")
        else:
            self.get_logger().info(f"Loaded config from path: {cfg_name_or_path}")
        # === ここまで: 方法Bのロード ===

        # （任意）最低限のスキーマ検証
        if "global" not in cfg:
            raise ValueError("Config 'global' is missing")
        if "cameras" not in cfg or not isinstance(cfg["cameras"], list) or not cfg["cameras"]:
            raise ValueError("Config 'cameras' is missing or empty")

        # === ここから: 従来どおり内部パラメータを初期化 ===
        g = cfg.get("global", {})
        self.hfov_rad = math.radians(float(g.get("horizontal_fov_deg", 60.0)))
        self.img_w = int(g.get("image_width", 640))
        self.img_h = int(g.get("image_height", 480))
        self.aruco_dict = get_aruco_dict(str(g.get("aruco_dictionary", "DICT_4X4_50")))
        self.det_params = cv2.aruco.DetectorParameters_create()        # 内部パラメータ（共通前提）
        self.K, self.D = compute_intrinsics_from_fov(self.img_w, self.img_h, self.hfov_rad)
        self.get_logger().info(
            f"K from HFOV={math.degrees(self.hfov_rad):.1f}deg, size=({self.img_w}x{self.img_h}) "
            f"-> fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}"
        )
        # カメラ列挙の取り出し（従来どおり）
        cams = cfg.get("cameras", [])
        self.cams = {}
        for c in cams:
            name = c["name"]
            if name in self.cams:
                raise ValueError(f"Duplicate camera name: {name}")
            self.cams[name] = c

        # 出力
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # ラッチ相当
        )
        self.pose_pubs: Dict[str, Any] = {}
        self.static_tfb = StaticTransformBroadcaster(self)

        # 状態
        self.bridge = CvBridge()
        self.done_flags: Dict[str, bool] = {name: False for name in self.cams.keys()}
        self.exit_on_complete = bool(self.get_parameter("exit_on_complete").value)
        self.require_all = bool(self.get_parameter("require_both_cameras").value)

        # 購読開始
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        for name, c in self.cams.items():
            topic = c["image_topic"]
            pose_topic, _, _ = names_for_camera(name)
            self.pose_pubs[name] = self.create_publisher(PoseStamped, pose_topic, pose_qos)
            self.create_subscription(Image, topic,
                                     lambda msg, n=name: self._image_callback(msg, n),
                                     sensor_qos)
            self.get_logger().info(f"Subscribe: {topic} -> Pose pub: {pose_topic}")

    # ----- 補助 -----
    @staticmethod
    def build_RT_from_xyzrpy(px, py, pz, rr, rp, ry):
        """world->marker の [R|t] を作る（固定軸RPY: X->Y->Z）。"""
        R = tft.euler_matrix(rr, rp, ry, axes="sxyz")[:3, :3]
        t = np.array([[px], [py], [pz]], dtype=np.float64)
        return R, t

    def _finish_node(self):
        if not self.exit_on_complete:
            return
        if self.require_all:
            if all(self.done_flags.values()):
                time.sleep(0.2)  # 吐き出し待ち
                self.get_logger().info("All cameras solved. Exiting.")
                rclpy.shutdown()
        else:
            # どれか1台でもOKで終了、にしたい場合
            if any(self.done_flags.values()):
                time.sleep(0.2)
                self.get_logger().info("At least one camera solved. Exiting.")
                rclpy.shutdown()
        # FIXME: node does not shut down somehow

    # ----- コールバック（各カメラごと） -----
    def _image_callback(self, msg: Image, cam_name: str):
        if self.done_flags[cam_name]:
            return  # すでに確定済み

        cam = self.cams[cam_name]
        marker_id = int(cam["marker_id"])
        marker_len = float(cam["marker_length_m"])
        px, py, pz, rr, rp, ry = cam["world_marker_xyzrpy"]

        # 画像→グレー
        img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        # ArUco検出
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.det_params)
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_len, self.K, self.D)
        ids_flat = ids.flatten()
        hit = np.where(ids_flat == marker_id)[0]
        if len(hit) == 0:
            return
        i = hit[0]

        # T_C_M
        rvec = rvecs[i].astype(np.float64)
        tvec = tvecs[i].reshape(3, 1).astype(np.float64)
        R_C_M, _ = cv2.Rodrigues(rvec)
        t_C_M = tvec

        # 反転 T_M_C
        R_M_C = R_C_M.T
        t_M_C = -R_M_C @ t_C_M

        # 合成 T_W_C = T_W_M * T_M_C
        R_W_M, t_W_M = self.build_RT_from_xyzrpy(px, py, pz, rr, rp, ry)
        R_W_C = R_W_M @ R_M_C
        t_W_C = R_W_M @ t_M_C + t_W_M

        # 出力（Pose + /tf_static）
        pose_topic, optical_frame, log_tag = names_for_camera(cam_name)
        self.get_logger().info(
            f"{log_tag} Camera position in world = "
            f"({t_W_C[0,0]:.3f}, {t_W_C[1,0]:.3f}, {t_W_C[2,0]:.3f}) [m]"
        )
        rpy = tft.euler_from_matrix(np.block([[R_W_C, np.zeros((3,1))],
                                            [np.array([0,0,0,1])]]))
        self.get_logger().info(f"{log_tag} RPY (deg) = ({math.degrees(rpy[0]):.1f}, {math.degrees(rpy[1]):.1f}, {math.degrees(rpy[2]):.1f})")
        q_W_C = tft.quaternion_from_matrix(
            np.block([[R_W_C, np.zeros((3, 1))], [np.array([0, 0, 0, 1])]])
        )

        # --- ★ここから追加ログ出力部分 ---
        msg_text = (
            f"{log_tag} Camera position (world): "
            f"({t_W_C[0,0]:.3f}, {t_W_C[1,0]:.3f}, {t_W_C[2,0]:.3f}) [m], "
            f"RPY(deg)=({math.degrees(rpy[0]):.1f}, "
            f"{math.degrees(rpy[1]):.1f}, "
            f"{math.degrees(rpy[2]):.1f})"
        )
        self.get_logger().info(msg_text)  # ROSログ
        print(msg_text)  # ターミナル標準出力にも表示
        # --- ★ここまで追加 ---

        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = "world"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = t_W_C.flatten().tolist()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q_W_C
        self.pose_pubs[cam_name].publish(ps)

        tf = TransformStamped()
        tf.header = ps.header
        tf.header.frame_id = "world"
        tf.child_frame_id = optical_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = t_W_C.flatten().tolist()
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q_W_C
        self.static_tfb.sendTransform(tf)

        self.get_logger().info(f"{log_tag} pos_world = ({t_W_C[0,0]:.3f}, {t_W_C[1,0]:.3f}, {t_W_C[2,0]:.3f})  (ID={marker_id})")

        # 確定フラグ
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
