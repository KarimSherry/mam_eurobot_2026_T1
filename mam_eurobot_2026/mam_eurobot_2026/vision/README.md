# Vision Nodes
Reference for the ROS 2 nodes that live in `mam_eurobot_2026/vision`. Each section captures what the node does, the topics it touches, and how to launch it with `ros2 run`.

## aruco_detector.py
- **Node**: `aruco_detector` – detects ArUco markers in the front camera feed, estimates each marker pose, and publishes `vision_msgs/Detection3DArray` messages with `Detection3D` entries per marker (including pose, bounding box, and class id).
- **Inputs**: `/front_camera` (`sensor_msgs/Image`, QoS depth 1, best-effort).
- **Outputs**: `/aruco/detections` (`vision_msgs/Detection3DArray`).
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 aruco_detector.py
  ```



## camera_viewer.py
- **Node**: `camera_viewer` – generic image viewer that subscribes to any camera topic, logs CameraInfo, and displays the stream in an OpenCV window 
- **Inputs**:
  - Required image topic argument (e.g. `/front_camera`), message type `sensor_msgs/Image`, QoS depth 5, best-effort/volatile.
  - Optional `--info-topic` CLI argument; defaults to `<image_topic>/camera_info`, message type `sensor_msgs/CameraInfo`.
- **Outputs**: OpenCV window only (no ROS topics published).
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 camera_viewer.py /front_camera --info-topic /front_camera/camera_info
  ```
  if does not work, run at root
  ```bash
  python3 src/mam_eurobot_2026/mam_eurobot_2026/vision/camera_viewer.py <topic name to subscribe>
  ```

## color_detector.py
- **Node**: `color_detector` – subscribes to a camera topic, thresholds the image in HSV space (parameters configurable via ROS parameters), applies morphological filtering, and displays the binary mask to verify detection of colored regions (e.g., green cursor).
- **Inputs**: `--image-topic <topic>` CLI argument (message type `sensor_msgs/Image`). QoS can be toggled with `--reliable` (otherwise BEST_EFFORT).
- **Outputs**: OpenCV window showing the mask (and optional contour visualization); no ROS publications.
- **Runtime parameters**: override HSV bounds or morphology via `--ros-args -p h_min:=... -p h_max:=... -p erode:=...`.
- **Run(example)**:
  ```bash
  ros2 run mam_eurobot_2026 color_detector.py --image-topic /top_camera/image_2
  ```

## show_front_camera.py
 No more in use

## world_to_topcamera.py
- **Node**: `world_to_topcamera` – uses ArUco detections from each overhead camera to solve the static transform from the `world` frame to every top camera, then publishes a `geometry_msgs/PoseStamped` on `/<camera_name>/pose_world` and broadcasts the corresponding static TF (`world` → `<camera_name>_optical_frame`).
- **Inputs**: subscribes to every `sensor_msgs/Image` topic listed under `cameras` in `vision_settings.yaml` (default: `/top_camera/image_1`, `/top_camera/image_2`). Each camera entry also specifies marker IDs/size and world pose hints.
- **Outputs**:
  - `/<camera>/pose_world` (`geometry_msgs/PoseStamped`, QoS TRANSIENT_LOCAL).
  - `/tf_static` entry publishing the transform to `<camera>_optical_frame`.
- **Runtime parameters** (set via `--ros-args`):
  - `config_yaml`: absolute/relative path or packaged `vision_settings.yaml`.
  - `exit_on_complete` (default `true`): shut down automatically after all cameras are solved.
  - `require_both_cameras` (default `true`): whether every listed camera must be solved before exiting.
- **Run**:
  ```bash
  ros2 run mam_eurobot_2026 world_to_topcamera.py
  ```
