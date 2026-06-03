import argparse
import os
import pickle

import cv2
import cv2.aruco as aruco
import numpy as np


FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_INDEX = 8
DEFAULT_ARUCO_DICT_ID = aruco.DICT_4X4_50
DEFAULT_MARKER_LENGTH = 0.018
DEFAULT_CALIBRATION_FILE = "camera_calibration.pkl"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Detect ArUco markers and display pose without path planning."
    )
    parser.add_argument("--camera-index", type=int, default=CAMERA_INDEX)
    parser.add_argument("--width", type=int, default=FRAME_WIDTH)
    parser.add_argument("--height", type=int, default=FRAME_HEIGHT)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--marker-length", type=float, default=None)
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION_FILE)
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--max-read-failures", type=int, default=5)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Do not open an OpenCV display window.",
    )
    parser.add_argument(
        "--snapshot-every",
        type=int,
        default=0,
        help="In headless mode, save an annotated frame every N frames. 0 disables snapshots.",
    )
    parser.add_argument("--snapshot-path", default="/tmp/aruco_pose_viewer.jpg")
    parser.add_argument(
        "--no-mjpg",
        action="store_true",
        help="Do not force MJPG mode before setting resolution.",
    )
    return parser.parse_args()


def create_aruco_dictionary(dictionary_id):
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dictionary_id)
    return aruco.Dictionary_get(dictionary_id)


def create_detector_parameters():
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()


def detect_markers(gray, dictionary, parameters):
    if hasattr(aruco, "detectMarkers"):
        return aruco.detectMarkers(gray, dictionary, parameters=parameters)
    detector = aruco.ArucoDetector(dictionary, parameters)
    return detector.detectMarkers(gray)


def estimate_marker_pose(corners, marker_length, camera_matrix, dist_coeffs):
    if hasattr(aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            np.array([corners], dtype=np.float32),
            marker_length,
            camera_matrix,
            dist_coeffs,
        )
        if rvecs is None or tvecs is None or len(rvecs) == 0:
            return False, None, None
        return True, rvecs[0], tvecs[0]

    half = marker_length / 2.0
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
        return False, None, None
    return True, rvec, tvec


def rotation_vector_to_yaw_z(rvec):
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    yaw_z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return np.degrees(yaw_z)


def draw_axes(frame, camera_matrix, dist_coeffs, rvec, tvec, axis_length):
    axis_points = np.float32(
        [
            [0, 0, 0],
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, axis_length],
        ]
    )
    imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = np.int32(imgpts)

    origin = tuple(imgpts[0].ravel())
    frame = cv2.line(frame, origin, tuple(imgpts[1].ravel()), (0, 0, 255), 3)
    frame = cv2.line(frame, origin, tuple(imgpts[2].ravel()), (0, 255, 0), 3)
    frame = cv2.line(frame, origin, tuple(imgpts[3].ravel()), (255, 0, 0), 3)
    return frame


def load_calibration(calibration_file, width, height, marker_length_override):
    if os.path.exists(calibration_file):
        print(f"Loading calibration from {calibration_file}...")
        with open(calibration_file, "rb") as handle:
            calibration_data = pickle.load(handle)
        marker_length = calibration_data.get("marker_length_m", DEFAULT_MARKER_LENGTH)
        if marker_length_override is not None:
            marker_length = marker_length_override
        dictionary_id = calibration_data.get("aruco_dictionary", DEFAULT_ARUCO_DICT_ID)
        print(
            "Calibration loaded "
            f"(reprojection error: {calibration_data['reprojection_error']:.4f})"
        )
        return (
            calibration_data["camera_matrix"],
            calibration_data["dist_coeffs"],
            marker_length,
            dictionary_id,
        )

    print("WARNING: No calibration found. Using approximate camera parameters.")
    focal_length = 960
    camera_matrix = np.array(
        [
            [focal_length, 0, width / 2],
            [0, focal_length, height / 2],
            [0, 0, 1],
        ],
        dtype=float,
    )
    dist_coeffs = np.zeros((4, 1))
    marker_length = marker_length_override or DEFAULT_MARKER_LENGTH
    return camera_matrix, dist_coeffs, marker_length, DEFAULT_ARUCO_DICT_ID


def open_camera(args):
    print(f"Opening camera {args.camera_index}...")
    cap = cv2.VideoCapture(args.camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Error: Cannot open camera {args.camera_index}")
        raise SystemExit(1)

    if not args.no_mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Camera mode: {actual_width:.0f}x{actual_height:.0f} @ {actual_fps:.1f} fps")
    return cap


def read_frame(cap, max_failures):
    for _ in range(max(1, max_failures)):
        ret, frame = cap.read()
        if ret:
            return True, frame
    return False, None


def main():
    args = parse_args()
    camera_matrix, dist_coeffs, marker_length, dictionary_id = load_calibration(
        args.calibration,
        args.width,
        args.height,
        args.marker_length,
    )
    aruco_dict = create_aruco_dictionary(dictionary_id)
    parameters = create_detector_parameters()
    cap = open_camera(args)
    for _ in range(max(0, args.warmup_frames)):
        cap.grab()

    frame_count = 0
    while True:
        ret, frame = read_frame(cap, args.max_read_failures)
        if not ret:
            print("Error: Failed to grab frame after retries")
            break

        frame_count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = detect_markers(gray, aruco_dict, parameters)

        if marker_ids is not None and len(marker_ids) > 0:
            aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

            for corners, marker_id in zip(marker_corners, marker_ids.flatten()):
                image_points = corners.reshape(4, 2).astype(np.float32)
                success, rvec, tvec = estimate_marker_pose(
                    image_points,
                    marker_length,
                    camera_matrix,
                    dist_coeffs,
                )
                if not success:
                    continue

                frame = draw_axes(
                    frame,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                    marker_length * 0.75,
                )
                marker_center = np.mean(image_points, axis=0).astype(int)
                position = tvec.flatten()
                yaw_z = rotation_vector_to_yaw_z(rvec)
                text_lines = [
                    f"ID: {marker_id}",
                    f"X: {position[0]:.3f}m Y: {position[1]:.3f}m Z: {position[2]:.3f}m",
                    f"Yaw Z: {yaw_z:.1f}deg",
                ]
                if int(marker_id) == 1:
                    print("ArUco marker ID 1 detected")
                y_offset = marker_center[1] - 35
                for index, text in enumerate(text_lines):
                    cv2.putText(
                        frame,
                        text,
                        (marker_center[0] - 95, y_offset + index * 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )
        else:
            cv2.putText(
                frame,
                "Show 4x4 ArUco markers to estimate pose",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )

        if args.headless:
            if args.snapshot_every > 0 and frame_count % args.snapshot_every == 0:
                cv2.imwrite(args.snapshot_path, frame)
                print(f"Wrote snapshot to {args.snapshot_path}")
        else:
            cv2.imshow("ArUco Pose Viewer", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
