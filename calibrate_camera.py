"""
Camera calibration script for Arducam OV9782.
Uses a checkerboard pattern to compute lens intrinsics (fx, fy, cx, cy).

Usage:
    python calibrate_camera.py

Controls:
    SPACE  - capture current frame as calibration sample
    c      - run calibration with captured samples
    r      - reset all captured samples
    q      - quit
"""

import cv2
import numpy as np
import json
import time
from pathlib import Path

# --- Configuration ---
CHECKERBOARD = (8, 5)       # inner corners (cols, rows) — adjust to your board
SQUARE_SIZE  = 25.0         # physical square size in mm (or any unit you want)
CAMERA_ID    = 0            # change if OV9782 is not /dev/video0
MIN_SAMPLES  = 25           # minimum captures before calibration is allowed
SAVE_PATH    = "camera_intrinsics.json"


def build_object_points():
    """3-D coordinates of checkerboard corners in the board's own frame."""
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE
    return objp


def draw_overlay(frame, msg, color=(0, 255, 0)):
    cv2.putText(frame, msg, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)


def save_results(mtx, dist, img_size):
    fx, fy = float(mtx[0, 0]), float(mtx[1, 1])
    cx, cy = float(mtx[0, 2]), float(mtx[1, 2])
    k1, k2, p1, p2, k3 = [float(v) for v in dist[0]]

    result = {
        "image_size": {"width": img_size[0], "height": img_size[1]},
        "camera_matrix": {
            "fx": fx, "fy": fy,
            "cx": cx, "cy": cy,
        },
        "distortion_coefficients": {
            "k1": k1, "k2": k2, "p1": p1, "p2": p2, "k3": k3,
        },
        "raw_camera_matrix": mtx.tolist(),
        "raw_dist_coeffs": dist.tolist(),
    }

    with open(SAVE_PATH, "w") as f:
        json.dump(result, f, indent=2)

    print("\n=== Calibration Results ===")
    print(f"  Image size : {img_size[0]} x {img_size[1]}")
    print(f"  fx={fx:.4f}  fy={fy:.4f}")
    print(f"  cx={cx:.4f}  cy={cy:.4f}")
    print(f"  k1={k1:.6f}  k2={k2:.6f}  k3={k3:.6f}")
    print(f"  p1={p1:.6f}  p2={p2:.6f}")
    print(f"  Saved to: {SAVE_PATH}")
    return result


def calibrate(obj_pts, img_pts, img_size):
    print(f"\nRunning calibration on {len(img_pts)} samples...")
    objp_template = build_object_points()
    obj_pts_full  = [objp_template] * len(img_pts)

    flags = (
        cv2.CALIB_RATIONAL_MODEL   # fit k1-k6 + tangential; drop if you want simple
    )

    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts_full, img_pts, img_size, None, None
    )
    print(f"  RMS reprojection error: {rms:.4f} px  (< 1.0 is good)")
    return mtx, dist, rms


def main():
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {CAMERA_ID}")

    # OV9782 native resolution — change if you use a different mode
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)

    objp    = build_object_points()
    img_pts = []          # 2-D corner lists per frame
    img_size = None
    last_capture_time = 0

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print("=== Arducam OV9782 Calibration ===")
    print(f"  Checkerboard: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} inner corners")
    print(f"  Square size : {SQUARE_SIZE} mm")
    print(f"  SPACE=capture  c=calibrate  r=reset  q=quit\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed — check camera connection.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])

        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        display = frame.copy()
        status_color = (0, 200, 0) if found else (0, 0, 200)

        if found:
            corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners_sub, found)
            draw_overlay(display, f"Board found  |  samples: {len(img_pts)}", status_color)
        else:
            draw_overlay(display, f"No board     |  samples: {len(img_pts)}", status_color)

        cv2.imshow("Calibration — Arducam OV9782", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):  # capture
            if not found:
                print("No board detected — move the checkerboard into view.")
            else:
                now = time.time()
                if now - last_capture_time < 0.5:   # debounce
                    continue
                last_capture_time = now
                img_pts.append(corners_sub)
                print(f"  Captured sample {len(img_pts)}")
                # flash green
                flash = display.copy()
                flash[:] = (0, 220, 0)
                cv2.addWeighted(flash, 0.3, display, 0.7, 0, display)
                cv2.imshow("Calibration — Arducam OV9782", display)
                cv2.waitKey(100)

        elif key == ord('c'):  # calibrate
            if len(img_pts) < MIN_SAMPLES:
                print(f"  Need at least {MIN_SAMPLES} samples (have {len(img_pts)}).")
            else:
                mtx, dist, rms = calibrate(None, img_pts, img_size)
                save_results(mtx, dist, img_size)

        elif key == ord('r'):  # reset
            img_pts.clear()
            print("  Samples cleared.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()