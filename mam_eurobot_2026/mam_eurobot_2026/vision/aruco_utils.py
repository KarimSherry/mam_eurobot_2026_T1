#!/usr/bin/env python3
import math
import pickle
from pathlib import Path
from typing import Any

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion
from importlib.resources import files


def resolve_existing_path(path_str: str) -> Path | None:
    if not path_str:
        return None
    path = Path(path_str)
    if path.is_absolute() and path.exists():
        return path
    candidate = Path.cwd() / path
    if candidate.exists():
        return candidate
    return None


def load_yaml_config(path_str: str, package_module: str, default_filename: str) -> dict[str, Any]:
    resolved = resolve_existing_path(path_str)
    if resolved is not None:
        with resolved.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle)

    filename = Path(path_str).name if path_str else default_filename
    packaged = files(package_module).joinpath(filename)
    if not packaged.is_file():
        raise FileNotFoundError(
            f"Config not found at '{path_str}' or packaged file '{filename}'"
        )
    with packaged.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def compute_intrinsics_from_fov(
    width: int,
    height: int,
    hfov_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    camera_matrix = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist_coeffs = np.zeros((5, 1), dtype=np.float64)
    return camera_matrix, dist_coeffs


def load_intrinsics(
    calibration_pkl: str,
    image_width: int,
    image_height: int,
    horizontal_fov_deg: float,
) -> tuple[np.ndarray, np.ndarray, str]:
    resolved = resolve_existing_path(calibration_pkl)
    if resolved is not None:
        with resolved.open("rb") as handle:
            data = pickle.load(handle)
        camera_matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
        dist_coeffs = np.asarray(data["dist_coeffs"], dtype=np.float64)
        return camera_matrix, dist_coeffs, f"calibration:{resolved}"

    return (
        *compute_intrinsics_from_fov(
            image_width,
            image_height,
            math.radians(horizontal_fov_deg),
        ),
        f"hfov:{horizontal_fov_deg:.1f}deg",
    )


def get_aruco_dictionary(name: str):
    value = getattr(aruco, name, aruco.DICT_4X4_50)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(value)
    return aruco.Dictionary_get(value)


def create_detector_parameters():
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()


def detect_markers(
    image: np.ndarray,
    dictionary,
    parameters,
):
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    if hasattr(aruco, "detectMarkers"):
        return aruco.detectMarkers(gray, dictionary, parameters=parameters)
    detector = aruco.ArucoDetector(dictionary, parameters)
    return detector.detectMarkers(gray)


def estimate_marker_pose(
    corners: np.ndarray,
    marker_length: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    if hasattr(aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            np.array([corners], dtype=np.float32),
            float(marker_length),
            camera_matrix,
            dist_coeffs,
        )
        if rvecs is None or tvecs is None or len(rvecs) == 0:
            return None, None
        return rvecs[0].reshape(3), tvecs[0].reshape(3)

    half = float(marker_length) / 2.0
    object_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    solve_flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        np.asarray(corners, dtype=np.float32),
        camera_matrix,
        dist_coeffs,
        flags=solve_flag,
    )
    if not success:
        return None, None
    return rvec.reshape(3), tvec.reshape(3)


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (matrix[2, 1] - matrix[1, 2]) * s
        y = (matrix[0, 2] - matrix[2, 0]) * s
        z = (matrix[1, 0] - matrix[0, 1]) * s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = 2.0 * math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = 2.0 * math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


def quaternion_matrix(quat: list[float] | tuple[float, ...] | np.ndarray) -> np.ndarray:
    x, y, z, w = [float(value) for value in quat]
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    transform = np.eye(4, dtype=np.float64)
    transform[0, 0] = 1.0 - 2.0 * (yy + zz)
    transform[0, 1] = 2.0 * (xy - wz)
    transform[0, 2] = 2.0 * (xz + wy)
    transform[1, 0] = 2.0 * (xy + wz)
    transform[1, 1] = 1.0 - 2.0 * (xx + zz)
    transform[1, 2] = 2.0 * (yz - wx)
    transform[2, 0] = 2.0 * (xz - wy)
    transform[2, 1] = 2.0 * (yz + wx)
    transform[2, 2] = 1.0 - 2.0 * (xx + yy)
    return transform


def euler_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    return transform


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    quat = np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )
    return quat / np.linalg.norm(quat)


def rvec_tvec_to_pose(rvec: np.ndarray, tvec: np.ndarray) -> Pose:
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    quat = rotation_matrix_to_quaternion(rotation)
    pose = Pose()
    pose.position = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
    pose.orientation = Quaternion(
        x=float(quat[0]),
        y=float(quat[1]),
        z=float(quat[2]),
        w=float(quat[3]),
    )
    return pose


def transform_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return transform


def transform_from_xyzrpy(xyzrpy: list[float] | tuple[float, ...]) -> np.ndarray:
    px, py, pz, rr, rp, ry = [float(value) for value in xyzrpy]
    transform = euler_matrix(rr, rp, ry)
    transform[:3, 3] = [px, py, pz]
    return transform


def transform_from_pose_msg(pose) -> np.ndarray:
    quat = [
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    ]
    transform = quaternion_matrix(quat)
    transform[:3, 3] = [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    ]
    return transform


def transform_from_tf_msg(transform_msg) -> np.ndarray:
    quat = [
        float(transform_msg.rotation.x),
        float(transform_msg.rotation.y),
        float(transform_msg.rotation.z),
        float(transform_msg.rotation.w),
    ]
    transform = quaternion_matrix(quat)
    transform[:3, 3] = [
        float(transform_msg.translation.x),
        float(transform_msg.translation.y),
        float(transform_msg.translation.z),
    ]
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    return np.linalg.inv(transform)


def pose_from_transform(transform: np.ndarray) -> Pose:
    quat = rotation_matrix_to_quaternion(transform[:3, :3])
    pose = Pose()
    pose.position = Point(
        x=float(transform[0, 3]),
        y=float(transform[1, 3]),
        z=float(transform[2, 3]),
    )
    pose.orientation = Quaternion(
        x=float(quat[0]),
        y=float(quat[1]),
        z=float(quat[2]),
        w=float(quat[3]),
    )
    return pose


def lookup_camera_config(cfg: dict[str, Any], image_topic: str) -> dict[str, Any]:
    cameras = cfg.get("cameras", [])
    for camera in cameras:
        if camera.get("image_topic") == image_topic:
            return camera
    if cameras:
        return cameras[0]
    raise ValueError("No cameras defined in vision configuration")
