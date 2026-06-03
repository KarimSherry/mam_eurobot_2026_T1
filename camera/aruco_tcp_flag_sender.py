import argparse
import os
import pickle
import socket
import time

import cv2
import cv2.aruco as aruco
import numpy as np


FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_INDEX = 8
DEFAULT_ARUCO_DICT_ID = aruco.DICT_4X4_50
DEFAULT_CALIBRATION_FILE = "camera_calibration.pkl"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send a TCP flag to Arduino when ArUco marker ID 1 is visible."
    )
    parser.add_argument("--arduino-host", required=True, help="Arduino WiFi shield IP address.")
    parser.add_argument("--arduino-port", type=int, default=5000)
    parser.add_argument("--camera-index", type=int, default=CAMERA_INDEX)
    parser.add_argument("--width", type=int, default=FRAME_WIDTH)
    parser.add_argument("--height", type=int, default=FRAME_HEIGHT)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--calibration", default=DEFAULT_CALIBRATION_FILE)
    parser.add_argument("--target-id", type=int, default=1)
    parser.add_argument("--send-period", type=float, default=0.1)
    parser.add_argument("--connect-timeout", type=float, default=3.0)
    parser.add_argument("--retry-delay", type=float, default=1.0)
    parser.add_argument("--max-read-failures", type=int, default=20)
    parser.add_argument("--no-mjpg", action="store_true")
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


def load_dictionary_id(calibration_file):
    if not os.path.exists(calibration_file):
        return DEFAULT_ARUCO_DICT_ID
    with open(calibration_file, "rb") as handle:
        calibration_data = pickle.load(handle)
    return calibration_data.get("aruco_dictionary", DEFAULT_ARUCO_DICT_ID)


def open_camera(args):
    print(f"Opening camera {args.camera_index}...")
    cap = cv2.VideoCapture(args.camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.camera_index}")

    if not args.no_mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    return cap


def read_frame(cap, max_failures):
    for _ in range(max(1, max_failures)):
        ret, frame = cap.read()
        if ret:
            return frame
    return None


class TcpFlagClient:
    def __init__(self, host, port, timeout, retry_delay):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.retry_delay = retry_delay
        self.sock = None
        self.last_connect_attempt = 0.0

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def connect_if_needed(self):
        if self.sock is not None:
            return True

        now = time.monotonic()
        if now - self.last_connect_attempt < self.retry_delay:
            return False
        self.last_connect_attempt = now

        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            sock.settimeout(self.timeout)
        except OSError as exc:
            print(f"TCP connect failed: {exc}")
            return False

        self.sock = sock
        print(f"Connected to Arduino at {self.host}:{self.port}")
        return True

    def send_flag(self, flag):
        if not self.connect_if_needed():
            return False

        try:
            self.sock.sendall(f"{int(flag)}\n".encode("ascii"))
            return True
        except OSError as exc:
            print(f"TCP send failed: {exc}")
            self.close()
            return False


def marker_is_visible(frame, dictionary, parameters, target_id):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, marker_ids, _ = detect_markers(gray, dictionary, parameters)
    if marker_ids is None:
        return False
    return int(target_id) in {int(marker_id) for marker_id in marker_ids.flatten()}


def main():
    args = parse_args()
    dictionary_id = load_dictionary_id(args.calibration)
    aruco_dict = create_aruco_dictionary(dictionary_id)
    parameters = create_detector_parameters()
    cap = open_camera(args)
    client = TcpFlagClient(
        args.arduino_host,
        args.arduino_port,
        args.connect_timeout,
        args.retry_delay,
    )

    last_send_time = 0.0
    last_flag = None

    try:
        while True:
            frame = read_frame(cap, args.max_read_failures)
            if frame is None:
                print("Camera read failed; sending flag 0")
                client.send_flag(0)
                continue

            flag = 1 if marker_is_visible(frame, aruco_dict, parameters, args.target_id) else 0
            now = time.monotonic()
            should_send = flag != last_flag or now - last_send_time >= args.send_period
            if not should_send:
                continue

            if client.send_flag(flag):
                last_send_time = now
                last_flag = flag
                if flag == 1:
                    print(f"Sent flag 1: marker ID {args.target_id} detected")
    except KeyboardInterrupt:
        pass
    finally:
        client.send_flag(0)
        client.close()
        cap.release()


if __name__ == "__main__":
    main()
