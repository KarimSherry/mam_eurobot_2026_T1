import argparse
import heapq
import os
import pickle

import cv2
import cv2.aruco as aruco
import numpy as np

# Camera capture settings used for the live display window.
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
CAMERA_INDEX = 2

# Default ArUco setup. The dictionary must match the printed markers you use.
DEFAULT_ARUCO_DICT_ID = aruco.DICT_4X4_50
DEFAULT_MARKER_LENGTH = 0.018  # meters

# Path planning is done on a coarse image-space grid instead of raw pixels.
# Each cell represents a GRID_STEP_PX x GRID_STEP_PX square in the camera image.
GRID_STEP_PX = 12

# Every obstacle marker is enlarged by this many pixels before being marked blocked.
# This acts like a safety margin so the path does not pass too close to other markers.
OBSTACLE_PADDING_PX = 24


def parse_args():
    # Read the start and goal marker IDs from the command line so the same script
    # can be reused for different path planning demonstrations.
    parser = argparse.ArgumentParser(
        description="Detect ArUco markers, estimate pose, and plan an image-space path."
    )
    parser.add_argument("start_id", type=int, help="ArUco marker ID for the start point")
    parser.add_argument("goal_id", type=int, help="ArUco marker ID for the goal point")
    parser.add_argument(
        "--debug-grid",
        action="store_true",
        help="Show the planning grid and padded obstacle areas on the camera overlay",
    )
    return parser.parse_args()


def create_aruco_dictionary(dictionary_id):
    # OpenCV exposes different helper names depending on version. This keeps the
    # script compatible with older and newer bindings.
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dictionary_id)
    return aruco.Dictionary_get(dictionary_id)


def create_detector_parameters():
    # DetectorParameters stores thresholds and tuning values used by the ArUco detector.
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()


def detect_markers(gray, dictionary, parameters):
    # Detect candidate 4-corner marker shapes and decode their IDs from the grayscale image.
    return aruco.detectMarkers(gray, dictionary, parameters=parameters)


def estimate_marker_pose(corners, marker_length, camera_matrix, dist_coeffs):
    # Pose estimation returns the rotation and translation of one marker relative
    # to the camera. We pass one marker at a time to get one rvec/tvec pair back.
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        np.array([corners], dtype=np.float32),
        marker_length,
        camera_matrix,
        dist_coeffs,
    )
    if rvecs is None or tvecs is None or len(rvecs) == 0:
        return False, None, None
    return True, rvecs[0], tvecs[0]


def rotation_vector_to_yaw_z(rvec):
    # Convert the compact Rodrigues rotation vector into a rotation matrix, then
    # extract yaw around Z so we can display a simple orientation angle.
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    yaw_z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return np.degrees(yaw_z)


def draw_axes(frame, camera_matrix, dist_coeffs, rvec, tvec, axis_length):
    # Define a tiny 3D coordinate frame anchored at the marker center, project it
    # into the image, and draw the X/Y/Z axes on the live camera frame.
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


def clamp_cell(cell, grid_width, grid_height):
    # Keep a cell inside the valid grid range so later indexing cannot go out of bounds.
    x = min(max(cell[0], 0), grid_width - 1)
    y = min(max(cell[1], 0), grid_height - 1)
    return (x, y)


def point_to_cell(point, grid_step, grid_width, grid_height):
    # Convert a pixel location in the image into a planning cell index.
    cell = (int(point[0] / grid_step), int(point[1] / grid_step))
    return clamp_cell(cell, grid_width, grid_height)


def cell_to_point(cell, grid_step):
    # Convert a planning cell back to the pixel coordinates of that cell's center.
    x = int(cell[0] * grid_step + grid_step / 2)
    y = int(cell[1] * grid_step + grid_step / 2)
    return (x, y)


def create_occupancy_grid(frame_shape, markers, start_id, goal_id):
    # Build a binary occupancy grid over the camera image:
    # 0 = free space, 1 = blocked space.
    #
    # The planner uses this grid as its map. Other markers become obstacles, while
    # the chosen start and goal markers are left free so the path can begin and end there.
    frame_height, frame_width = frame_shape[:2]
    grid_width = max(1, frame_width // GRID_STEP_PX)
    grid_height = max(1, frame_height // GRID_STEP_PX)
    occupancy = np.zeros((grid_height, grid_width), dtype=np.uint8)
    obstacle_bounds = []

    for marker_id, marker in markers.items():
        # The start and goal are targets, not obstacles.
        if marker_id in {start_id, goal_id}:
            continue

        corners = marker["corners"]

        # Compute a padded bounding box around each obstacle marker in pixel space.
        # Padding makes the planner behave more conservatively around obstacles.
        min_xy = np.floor(np.min(corners, axis=0) - OBSTACLE_PADDING_PX).astype(int)
        max_xy = np.ceil(np.max(corners, axis=0) + OBSTACLE_PADDING_PX).astype(int)

        # Convert the padded pixel box into grid coordinates so we can mark blocked cells.
        min_cell = point_to_cell(min_xy, GRID_STEP_PX, grid_width, grid_height)
        max_cell = point_to_cell(max_xy, GRID_STEP_PX, grid_width, grid_height)

        # Fill the rectangle of blocked cells covered by this padded obstacle.
        occupancy[min_cell[1] : max_cell[1] + 1, min_cell[0] : max_cell[0] + 1] = 1

        # Save the bounds so the optional debug overlay can show exactly what was blocked.
        obstacle_bounds.append(
            {
                "marker_id": marker_id,
                "min_xy": min_xy,
                "max_xy": max_xy,
                "min_cell": min_cell,
                "max_cell": max_cell,
            }
        )

    return occupancy, grid_width, grid_height, obstacle_bounds


def heuristic(cell, goal):
    # A* uses straight-line distance to the goal as its heuristic estimate.
    return np.hypot(goal[0] - cell[0], goal[1] - cell[1])


def astar_path(occupancy, start_cell, goal_cell):
    # Standard A* graph search over the occupancy grid.
    #
    # Each cell is a node. The planner can move to 8 neighbors:
    # horizontal, vertical, and diagonal.
    neighbors = [
        (-1, -1), (0, -1), (1, -1),
        (-1, 0),            (1, 0),
        (-1, 1),  (0, 1),   (1, 1),
    ]
    height, width = occupancy.shape
    open_heap = [(heuristic(start_cell, goal_cell), 0.0, start_cell)]
    came_from = {}
    g_score = {start_cell: 0.0}
    visited = set()

    while open_heap:
        # Pop the cell with lowest estimated total cost = real cost so far + heuristic.
        _, current_cost, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)

        if current == goal_cell:
            # Reconstruct the path by walking backward from goal to start.
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dx, dy in neighbors:
            nx = current[0] + dx
            ny = current[1] + dy

            # Ignore neighbors outside the grid.
            if not (0 <= nx < width and 0 <= ny < height):
                continue

            # Ignore blocked cells.
            if occupancy[ny, nx]:
                continue

            # Diagonal steps cost sqrt(2), straight steps cost 1.
            step_cost = np.hypot(dx, dy)
            candidate_cost = current_cost + step_cost
            neighbor = (nx, ny)

            # Only keep this new route if it improves the best cost known for that cell.
            if candidate_cost >= g_score.get(neighbor, float("inf")):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = candidate_cost
            priority = candidate_cost + heuristic(neighbor, goal_cell)
            heapq.heappush(open_heap, (priority, candidate_cost, neighbor))

    return None


def simplify_path(points):
    # A* returns many small cell-by-cell steps. For display, we keep only the key
    # turning points so the overlay looks like a clean route instead of a jagged trace.
    if len(points) <= 2:
        return points

    simplified = [points[0]]
    last_direction = None

    for idx in range(1, len(points)):
        dx = points[idx][0] - points[idx - 1][0]
        dy = points[idx][1] - points[idx - 1][1]
        direction = (np.sign(dx), np.sign(dy))
        if last_direction is None:
            last_direction = direction
            continue
        if direction != last_direction:
            simplified.append(points[idx - 1])
            last_direction = direction

    simplified.append(points[-1])
    return simplified


def plan_marker_path(frame_shape, markers, start_id, goal_id):
    # This is the main planning function used by the live loop.
    #
    # It creates the occupancy grid, converts the start and goal marker centers
    # into cells, runs A*, and converts the resulting cell path back to pixel points.
    if start_id not in markers or goal_id not in markers:
        return None, "Start or goal marker not visible", None

    occupancy, grid_width, grid_height, obstacle_bounds = create_occupancy_grid(
        frame_shape,
        markers,
        start_id,
        goal_id,
    )

    start_point = markers[start_id]["center"]
    goal_point = markers[goal_id]["center"]
    start_cell = point_to_cell(start_point, GRID_STEP_PX, grid_width, grid_height)
    goal_cell = point_to_cell(goal_point, GRID_STEP_PX, grid_width, grid_height)

    # Make sure the planner is allowed to stand on the start and goal cells even if
    # rounding or overlap would otherwise mark them as blocked.
    occupancy[start_cell[1], start_cell[0]] = 0
    occupancy[goal_cell[1], goal_cell[0]] = 0

    path_cells = astar_path(occupancy, start_cell, goal_cell)
    if path_cells is None:
        debug_info = {
            "occupancy": occupancy,
            "grid_width": grid_width,
            "grid_height": grid_height,
            "obstacle_bounds": obstacle_bounds,
            "start_cell": start_cell,
            "goal_cell": goal_cell,
        }
        return None, "No obstacle-free path found", debug_info

    path_points = [cell_to_point(cell, GRID_STEP_PX) for cell in path_cells]

    # Replace the first and last cell centers with the actual marker centers so the
    # drawn path touches the visible start and goal markers more naturally.
    path_points[0] = tuple(start_point.astype(int))
    path_points[-1] = tuple(goal_point.astype(int))
    debug_info = {
        "occupancy": occupancy,
        "grid_width": grid_width,
        "grid_height": grid_height,
        "obstacle_bounds": obstacle_bounds,
        "start_cell": start_cell,
        "goal_cell": goal_cell,
    }
    return simplify_path(path_points), None, debug_info


def draw_path_overlay(frame, path_points, start_id, goal_id):
    # Draw the planned route as a polyline on top of the camera image.
    if not path_points or len(path_points) < 2:
        return frame

    polyline = np.array(path_points, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [polyline], False, (255, 255, 0), 3)

    for point in path_points[1:-1]:
        cv2.circle(frame, point, 4, (255, 200, 0), -1)

    cv2.putText(
        frame,
        f"Path {start_id} -> {goal_id}",
        (20, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 0),
        2,
    )
    return frame


def draw_debug_grid_overlay(frame, debug_info):
    # Optional debug visualization that makes the planner's internal map visible:
    # blocked cells, the underlying grid, padded obstacle boxes, and start/goal cells.
    if debug_info is None:
        return frame

    overlay = frame.copy()
    occupancy = debug_info["occupancy"]
    grid_height, grid_width = occupancy.shape

    for cell_y in range(grid_height):
        for cell_x in range(grid_width):
            if not occupancy[cell_y, cell_x]:
                continue

            # Paint every blocked grid cell in orange so the occupancy map is easy to see.
            x0 = cell_x * GRID_STEP_PX
            y0 = cell_y * GRID_STEP_PX
            x1 = min(x0 + GRID_STEP_PX, frame.shape[1] - 1)
            y1 = min(y0 + GRID_STEP_PX, frame.shape[0] - 1)
            cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 90, 255), -1)

    # Blend the blocked-cell overlay with the camera image so the scene stays visible.
    cv2.addWeighted(overlay, 0.22, frame, 0.78, 0, frame)

    # Draw the full planning grid for explanation/debugging purposes.
    for x in range(0, frame.shape[1], GRID_STEP_PX):
        cv2.line(frame, (x, 0), (x, frame.shape[0] - 1), (70, 70, 70), 1)
    for y in range(0, frame.shape[0], GRID_STEP_PX):
        cv2.line(frame, (0, y), (frame.shape[1] - 1, y), (70, 70, 70), 1)

    for obstacle in debug_info["obstacle_bounds"]:
        # Outline the padded obstacle region in pixel space so it is obvious how the
        # original marker area was enlarged before planning.
        min_xy = tuple(obstacle["min_xy"])
        max_xy = tuple(obstacle["max_xy"])
        cv2.rectangle(frame, min_xy, max_xy, (0, 165, 255), 2)
        cv2.putText(
            frame,
            f"pad {obstacle['marker_id']}",
            (min_xy[0], max(min_xy[1] - 6, 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 165, 255),
            1,
        )

    # Mark the start and goal cells used by the planner.
    start_point = cell_to_point(debug_info["start_cell"], GRID_STEP_PX)
    goal_point = cell_to_point(debug_info["goal_cell"], GRID_STEP_PX)
    cv2.circle(frame, start_point, 6, (0, 255, 0), -1)
    cv2.circle(frame, goal_point, 6, (0, 255, 255), -1)
    cv2.putText(
        frame,
        "Debug grid: blocked cells in orange, padded obstacle boxes outlined",
        (20, frame.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    return frame


# Parse command-line options once at startup.
args = parse_args()

# Try to load the real camera calibration so pose estimation uses measured intrinsics.
if os.path.exists("camera_calibration.pkl"):
    print("Loading calibration from camera_calibration.pkl...")
    with open("camera_calibration.pkl", "rb") as f:
        calibration_data = pickle.load(f)
    camera_matrix = calibration_data["camera_matrix"]
    dist_coeffs = calibration_data["dist_coeffs"]
    reprojection_error = calibration_data["reprojection_error"]
    marker_length = calibration_data.get("marker_length_m", DEFAULT_MARKER_LENGTH)
    dictionary_id = calibration_data.get("aruco_dictionary", DEFAULT_ARUCO_DICT_ID)
    print(f"✓ Calibration loaded (reprojection error: {reprojection_error:.4f})")
else:
    # Fall back to a rough pinhole camera model if no calibration file exists.
    # This keeps the demo running, but the pose numbers will be less accurate.
    print("WARNING: No calibration found. Using default camera parameters.")
    print("Run camera_calibration.py to generate calibration data.")
    focal_length = 960
    cx = FRAME_WIDTH / 2
    cy = FRAME_HEIGHT / 2
    camera_matrix = np.array(
        [
            [focal_length, 0, cx],
            [0, focal_length, cy],
            [0, 0, 1],
        ],
        dtype=float,
    )
    dist_coeffs = np.zeros((4, 1))
    marker_length = DEFAULT_MARKER_LENGTH
    dictionary_id = DEFAULT_ARUCO_DICT_ID

# Build the detector once so it can be reused for every frame.
aruco_dict = create_aruco_dictionary(dictionary_id)
parameters = create_detector_parameters()

print(f"Planning path from marker {args.start_id} to marker {args.goal_id}")
print(f"Opening camera {CAMERA_INDEX}...")
cap = cv2.VideoCapture(CAMERA_INDEX)
if not cap.isOpened():
    print(f"Error: Cannot open camera {CAMERA_INDEX}")
    raise SystemExit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

# Main live-processing loop:
# 1. grab a frame
# 2. detect markers
# 3. estimate pose for each marker
# 4. build a path from start marker to goal marker
# 5. draw overlays and show the result
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to grab frame")
        break

    # ArUco detection works on grayscale images.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = detect_markers(gray, aruco_dict, parameters)

    # This dictionary stores the image-space geometry needed by the path planner.
    detected_markers = {}

    if marker_ids is not None and len(marker_ids) > 0:
        # Draw marker outlines and IDs as detected by OpenCV.
        aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

        for corners, marker_id in zip(marker_corners, marker_ids.flatten()):
            image_points = corners.reshape(4, 2).astype(np.float32)
            success, rvec, tvec = estimate_marker_pose(
                image_points,
                marker_length,
                camera_matrix,
                dist_coeffs,
            )

            # The marker center and corners are stored even if pose estimation fails,
            # because the path planner only needs the marker geometry in the image.
            marker_center = np.mean(image_points, axis=0)
            detected_markers[int(marker_id)] = {
                "corners": image_points,
                "center": marker_center,
            }

            if not success:
                continue

            # Draw pose information only when OpenCV successfully estimates it.
            frame = draw_axes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length * 0.75)
            position = tvec.flatten()
            yaw_z = rotation_vector_to_yaw_z(rvec)

            text_lines = [
                f"ID: {marker_id}",
                f"X: {position[0]:.3f}m Y: {position[1]:.3f}m Z: {position[2]:.3f}m",
                f"Yaw Z: {yaw_z:.1f}deg",
            ]

            center_int = marker_center.astype(int)
            y_offset = center_int[1] - 35
            for index, text in enumerate(text_lines):
                cv2.putText(
                    frame,
                    text,
                    (center_int[0] - 95, y_offset + index * 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        # Plan a path in image space from the chosen start marker to the chosen goal marker.
        path_points, path_error, debug_info = plan_marker_path(
            frame.shape,
            detected_markers,
            args.start_id,
            args.goal_id,
        )

        # Optional debug overlay that reveals the occupancy grid and padded obstacles.
        if args.debug_grid:
            frame = draw_debug_grid_overlay(frame, debug_info)

        if path_points is not None:
            frame = draw_path_overlay(frame, path_points, args.start_id, args.goal_id)
        else:
            # If a route cannot be produced, tell the user why on the live image.
            cv2.putText(
                frame,
                path_error,
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
    else:
        # No markers visible at all, so neither pose estimation nor planning can run.
        cv2.putText(
            frame,
            "Show 4x4 ArUco markers to estimate pose and plan a path",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

    # Show the final composited frame and exit when ESC is pressed.
    cv2.imshow("ArUco Marker Pose and Path", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release camera resources and close the OpenCV window on exit.
cap.release()
cv2.destroyAllWindows()
