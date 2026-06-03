import glob
import os
import pickle
import shutil

import cv2
import numpy as np

CHECKERBOARD_COLS = 8
CHECKERBOARD_ROWS = 6
SQUARE_LENGTH = 0.025  # 2.5 cm

CALIBRATION_DIR = "calibration_images"
CALIBRATION_FILE = "camera_calibration.pkl"
CAMERA_INDEX = 2
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

os.makedirs(CALIBRATION_DIR, exist_ok=True)


def create_object_points():
    object_points = np.zeros((CHECKERBOARD_ROWS * CHECKERBOARD_COLS, 3), np.float32)
    grid = np.mgrid[0:CHECKERBOARD_COLS, 0:CHECKERBOARD_ROWS].T.reshape(-1, 2)
    object_points[:, :2] = grid * SQUARE_LENGTH
    return object_points


def refine_corners(gray, corners):
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def find_checkerboard_corners(gray):
    pattern_size = (CHECKERBOARD_COLS, CHECKERBOARD_ROWS)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return False, None
    return True, refine_corners(gray, corners)


def reset_calibration_dir():
    """Start each calibration run from a clean image directory."""
    if os.path.isdir(CALIBRATION_DIR):
        shutil.rmtree(CALIBRATION_DIR)
    os.makedirs(CALIBRATION_DIR, exist_ok=True)


def capture_calibration_images():
    """Capture checkerboard images from the camera."""
    print("\n=== STEP 1: CAPTURE CALIBRATION IMAGES ===")
    print("Instructions:")
    print("1. Hold your printed 8x6 checkerboard in front of the camera")
    print("2. Press SPACE to capture an image")
    print("3. Move the board to a different angle/distance")
    print("4. Repeat 20-30 times from varied views")
    print("5. Press ESC when done")
    print("\nTips for good calibration:")
    print("- Keep the whole checkerboard visible in many shots")
    print("- Include tilted and off-center views")
    print("- Use sharp, well-lit images with limited glare")
    print("- Mix close and far distances")
    print("- Make sure the printed checkerboard matches this script's 8x6 inner-corner layout")
    print("-" * 50)

    reset_calibration_dir()
    print(f"Cleared and recreated {CALIBRATION_DIR}/ for a fresh calibration run.")

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return False

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    preview_window = "Camera Preview - Press SPACE to capture, ESC to finish"
    cv2.namedWindow(preview_window)

    count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame")
            break

        preview = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = find_checkerboard_corners(gray)
        if found:
            cv2.drawChessboardCorners(
                preview,
                (CHECKERBOARD_COLS, CHECKERBOARD_ROWS),
                corners,
                found,
            )

        cv2.putText(
            preview,
            f"Captured: {count}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
        )
        cv2.imshow(preview_window, preview)

        key = cv2.waitKey(1) & 0xFF
        if key == 32:
            filename = os.path.join(CALIBRATION_DIR, f"image_{count:02d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"✓ Captured image {count}: {filename}")
            count += 1
        elif key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

    print(f"\nTotal images captured: {count}")
    return count > 0


def calibrate_camera():
    """Calibrate camera intrinsics from checkerboard images."""
    print("\n=== STEP 2: CALIBRATE CAMERA ===")
    images = sorted(glob.glob(os.path.join(CALIBRATION_DIR, "*.jpg")))
    print(f"Processing {len(images)} calibration images...")

    if not images:
        print("Error: No calibration images found!")
        return False

    template_object_points = create_object_points()
    object_points = []
    image_points = []
    expected_image_size = None

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"✗ {os.path.basename(fname)} - Could not read image")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        if expected_image_size is None:
            expected_image_size = image_size
        elif image_size != expected_image_size:
            print(
                f"✗ {os.path.basename(fname)} - Skipped due to mismatched image size "
                f"{image_size}, expected {expected_image_size}"
            )
            continue

        found, corners = find_checkerboard_corners(gray)
        if found:
            object_points.append(template_object_points.copy())
            image_points.append(corners)
            print(
                f"✓ {os.path.basename(fname)} - "
                f"{CHECKERBOARD_COLS * CHECKERBOARD_ROWS} checkerboard corners"
            )
        else:
            print(f"✗ {os.path.basename(fname)} - Checkerboard not detected")

    if len(object_points) < 5:
        print("Error: Not enough valid calibration images (need at least 5)")
        return False

    if expected_image_size is None:
        print("Error: No readable calibration images were found.")
        return False

    print("\nRunning checkerboard calibration...")
    reprojection_error, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        object_points,
        image_points,
        expected_image_size,
        None,
        None,
    )

    print("\n" + "=" * 50)
    print("CALIBRATION SUCCESSFUL!")
    print("=" * 50)
    print(f"Reprojection Error: {reprojection_error:.4f} (lower is better, <1.0 is good)")
    print(f"\nCamera Matrix:\n{camera_matrix}")
    print(f"\nDistortion Coefficients:\n{dist_coeffs.flatten()}")

    calibration_data = {
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
        "reprojection_error": reprojection_error,
        "board_type": "checkerboard",
        "checkerboard_cols": CHECKERBOARD_COLS,
        "checkerboard_rows": CHECKERBOARD_ROWS,
        "square_length_m": SQUARE_LENGTH,
    }

    with open(CALIBRATION_FILE, "wb") as f:
        pickle.dump(calibration_data, f)
    print(f"\n✓ Calibration saved to {CALIBRATION_FILE}")
    return True


def main():
    print("\n" + "=" * 50)
    print("CHECKERBOARD CAMERA CALIBRATION TOOL")
    print("=" * 50)

    if not capture_calibration_images():
        print("Calibration cancelled")
        return

    if calibrate_camera():
        print("\n" + "=" * 50)
        print("Calibration complete! Ready to use with cv_test.py")
        print("=" * 50)
    else:
        print("\nCalibration failed. Please try again with better images.")


if __name__ == "__main__":
    main()
