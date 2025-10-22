#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion, Vector3
from rclpy.node import Node
from vision_msgs.msg import (
    Detection3DArray,
    Detection3D,
    ObjectHypothesisWithPose,
    ObjectHypothesis,
    BoundingBox3D,
)
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import xml.etree.ElementTree as ET
from cv_bridge import CvBridge

# this node is only for Image, does not assume CameraInfo
# input; sensor_msgs/Image　
# output; vision_msgs/Detection3DArray 

class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info("Aruco Detect Node created!")
        self._set_camera_parameters()

        # aruco detector
        self.marker_len_m = float(min(self.aruco_size)) if self.aruco_size else 0.13
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        # self.detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())　# cannot be used in opencv 4.5.4

        self.parameters = aruco.DetectorParameters_create()  # ← これ！


        self.bridge = CvBridge()

        qos = QoSProfile(depth=1)
        self.pub = self.create_publisher(Detection3DArray, '/aruco/detections', qos)
        # self.create_subscription(Image, "/front_camera/image", self._image_callback, qos)
        self.create_subscription(Image, "/front_camera", self._image_callback, qos)
        # self.create_subscription(CameraInfo, "front_camera/camera_info", self.callback, 10)
    

    def _set_camera_parameters(self):
        # read SDF files
        tree = ET.parse('/home/rosdev/eurobot_2026_ws/src/mam_eurobot_2026/models/simple_robot/model.sdf')
        root = tree.getroot()

        for sensor in root.iter('sensor'):
            if sensor.get('name') == 'front_camera':
                camera = sensor.find('camera')
                hfov = camera.find('horizontal_fov').text
                print(f"Horizontal FOV: {hfov}")
                self.fov = float(hfov)
                img = camera.find('image')
                width = img.find('width').text
                self.width = float(width)
                height = img.find('height').text
                self.height = float(height)
                print(f"img size; {width}x{height}")

        tree_marker = ET.parse('/home/rosdev/eurobot_2026_ws/src/mam_eurobot_2026/models/crate_blue/model.sdf')
        root_marker = tree_marker.getroot()
        aruco_size = None
        for visual in root_marker.iter('visual'):
            if visual.get('name') == 'aruco_top':
                plane = visual.find('.//plane')  
                if plane is not None:
                    size_tag = plane.find('size')
                    if size_tag is not None:
                        aruco_size = tuple(map(float, size_tag.text.split()))
                        break
        print(f"aruco size and type; {aruco_size}, {type(aruco_size)}")
        self.aruco_size = aruco_size

        # fx, fy calculate fx fy from hfov
        self.fx = self.width / (2.0 * math.tan(self.fov / 2.0))
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0


    def _image_callback(self, img_msg: Image):
        self.get_logger().info(f"Received image frame: {img_msg.header.stamp.sec}.{img_msg.header.stamp.nanosec}")
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        det_array = self._detect_markers(img_msg, frame)
        if det_array is not None:
            self.get_logger().info(f"Publishing Detection3DArray with {len(det_array.detections)} detections")
            self.pub.publish(det_array)

    def _detect_markers(self, img_msg: Image, frame: np.ndarray):
        h, w = frame.shape[:2]

        # parameter matrix K and Distortion D
        K = np.array([[self.fx, 0, self.cx],
                      [0, self.fy, self.cy],
                      [0,     0,    1]], np.float32)
        D = np.zeros((5, 1), np.float32)

        # ArUco 検出
        # corners, ids, _ = self.detector.detectMarkers(frame) # cannot be used in opencv 4.5.4
        corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
        det_array = Detection3DArray()
        det_array.header = Header()
        det_array.header.stamp = img_msg.header.stamp
        det_array.header.frame_id = img_msg.header.frame_id or "front_camera_frame"

        if ids is None or len(ids) == 0:
            # if not detected, return empty matrix
            return det_array

        # pose estimation
        rvecs, tvecs, _objpts = aruco.estimatePoseSingleMarkers(
            corners, self.marker_len_m, K, D
        )

        # draw coordinate axis for debug
        # for rvec, tvec in zip(rvecs, tvecs):
        #     cv2.drawFrameAxes(frame, K, D, rvec, tvec, self.marker_len_m * 0.5)

        for idx, (marker_id, rvec, tvec) in enumerate(zip(ids.flatten(), rvecs, tvecs)):
            # rvec/tvec -> Pose
            pose = self._rvec_tvec_to_pose(rvec.reshape(3), tvec.reshape(3))

            # results: ObjectHypothesisWithPose
            hyp = ObjectHypothesis(class_id=str(int(marker_id)), score=1.0)
            pose_cov = PoseWithCovariance()
            pose_cov.pose = pose
            pose_cov.covariance = [0.0] * 36  

            owp = ObjectHypothesisWithPose(hypothesis=hyp, pose=pose_cov)

            det = Detection3D()
            det.header = det_array.header
            det.results = [owp]

            bbox = BoundingBox3D()
            bbox.center = pose
            sx = sy = float(self.marker_len_m)
            sz = 0.001
            bbox.size = Vector3(x=sx, y=sy, z=sz)
            det.bbox = bbox

            det_array.detections.append(det)
            
            # logger for debug
            self.get_logger().info(
                f"\n=== ArUco Detection ===\n"
                f"Marker ID: {marker_id}\n"
                f"rvec: {rvec}\n"
                f"tvec (camera frame, meters): {tvec}\n"
                f"Quaternion: (x={pose.orientation.x:.4f}, y={pose.orientation.y:.4f}, z={pose.orientation.z:.4f}, w={pose.orientation.w:.4f})\n"
            )
        return det_array
    
    @staticmethod
    def _rvec_tvec_to_pose(rvec: np.ndarray, tvec: np.ndarray) -> Pose:
        R, _ = cv2.Rodrigues(rvec.astype(np.float64))
        qw = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw) if qw != 0 else 0.0
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw) if qw != 0 else 0.0
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw) if qw != 0 else 0.0

        pose = Pose()
        pose.position = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return pose



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