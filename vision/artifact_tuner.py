"""
Artifact HSV Tuner
==================
Plug in your webcam and run this script to tune the HSV thresholds for
purple and green 5-inch balls in real time.

Usage:
    python vision/artifact_tuner.py
    python vision/artifact_tuner.py --camera 1   # if webcam index isn't 0

Controls:
    [s]  Print current values as Java constants (HSV + lockCameraControls args)
    [r]  Reset all sliders to the current robot defaults
    [q]  Quit

Camera controls (Exposure / Gain):
    Tune these to match what you will pass to VisionManager.lockCameraControls()
    on the robot. Locking exposure and gain prevents the camera from auto-adjusting
    to changing venue lighting, which would shift your tuned HSV values.

    NOTE: exposure and gain control requires a USB webcam that supports manual
    control. If the sliders have no effect, your camera or OS driver does not
    expose manual control — skip those sliders and rely on auto on this computer,
    but still set lockCameraControls() on the robot using the Arducam's range.

Mirrors the exact KColorBlobProcessor pipeline:
  - Downsamples to 320x240
  - Converts RGB → HSV
  - Thresholds each channel
  - Morphological OPEN (3x3 ellipse) then CLOSE (7x7 ellipse)
  - Finds external contours, filters by area and circularity
  - Scales bounding boxes back to full resolution
"""

import argparse
import sys
import cv2
import numpy as np

# ── Pipeline constants matching KColorBlobProcessor ───────────────────────────
PROCESSING_W, PROCESSING_H = 320, 240
MORPH_OPEN_SIZE  = (3, 3)
MORPH_CLOSE_SIZE = (7, 7)

# ── Robot defaults (sync with ArtifactDetectionProcessor.java) ────────────────
DEFAULTS = {
    "Purple": {"h_lo": 117, "h_hi": 180, "s_lo": 58,  "s_hi": 255, "v_lo": 54,  "v_hi": 255},
    "Green":  {"h_lo": 68,  "h_hi": 92,  "s_lo": 70,  "s_hi": 255, "v_lo": 22,  "v_hi": 255},
}

# ── Camera control defaults — match what you pass to lockCameraControls() ─────
# Arducam on the robot: exposure 15–50 ms, gain 200–400 are typical starting points.
DEFAULT_EXPOSURE_MS = 20   # milliseconds (robot range: ~5–200)
DEFAULT_GAIN        = 250  # unitless     (robot range: ~0–500)

# Detection thresholds — match KColorBlobProcessor fields
MIN_AREA = 250
MAX_AREA = 30_000
MIN_CIRCULARITY = 0.55

# ── Window names ──────────────────────────────────────────────────────────────
WIN_MAIN   = "Artifact Tuner — Live Feed  [s=save  r=reset  q=quit]"
WIN_PURPLE = "Mask: Purple"
WIN_GREEN  = "Mask: Green"
WIN_CTRL   = "HSV Controls"
WIN_CAM    = "Camera Controls  (match these in lockCameraControls)"

MORPH_OPEN  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, MORPH_OPEN_SIZE)
MORPH_CLOSE = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, MORPH_CLOSE_SIZE)


def compute_circularity(area: float, perimeter: float) -> float:
    return (4.0 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0.0


def create_trackbars(window: str, color: str) -> None:
    d = DEFAULTS[color]
    cv2.createTrackbar(f"{color} H lo", window, d["h_lo"], 180, lambda _: None)
    cv2.createTrackbar(f"{color} H hi", window, d["h_hi"], 180, lambda _: None)
    cv2.createTrackbar(f"{color} S lo", window, d["s_lo"], 255, lambda _: None)
    cv2.createTrackbar(f"{color} S hi", window, d["s_hi"], 255, lambda _: None)
    cv2.createTrackbar(f"{color} V lo", window, d["v_lo"], 255, lambda _: None)
    cv2.createTrackbar(f"{color} V hi", window, d["v_hi"], 255, lambda _: None)


def read_trackbars(window: str, color: str) -> tuple:
    h_lo = cv2.getTrackbarPos(f"{color} H lo", window)
    h_hi = cv2.getTrackbarPos(f"{color} H hi", window)
    s_lo = cv2.getTrackbarPos(f"{color} S lo", window)
    s_hi = cv2.getTrackbarPos(f"{color} S hi", window)
    v_lo = cv2.getTrackbarPos(f"{color} V lo", window)
    v_hi = cv2.getTrackbarPos(f"{color} V hi", window)
    return (h_lo, h_hi, s_lo, s_hi, v_lo, v_hi)


def reset_trackbars(window: str, color: str) -> None:
    d = DEFAULTS[color]
    cv2.setTrackbarPos(f"{color} H lo", window, d["h_lo"])
    cv2.setTrackbarPos(f"{color} H hi", window, d["h_hi"])
    cv2.setTrackbarPos(f"{color} S lo", window, d["s_lo"])
    cv2.setTrackbarPos(f"{color} S hi", window, d["s_hi"])
    cv2.setTrackbarPos(f"{color} V lo", window, d["v_lo"])
    cv2.setTrackbarPos(f"{color} V hi", window, d["v_hi"])


def detect_blobs(hsv_small: np.ndarray, lower: np.ndarray, upper: np.ndarray,
                 width_scale: float, height_scale: float):
    """Returns (mask_small, list of (bbox_full_res, area, circularity))."""
    mask = cv2.inRange(hsv_small, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  MORPH_OPEN)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_CLOSE)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA:
            continue
        perimeter   = cv2.arcLength(cnt, True)
        circularity = compute_circularity(area, perimeter)
        if circularity < MIN_CIRCULARITY:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        full_bbox = (
            int(x * width_scale), int(y * height_scale),
            int(w * width_scale), int(h * height_scale),
        )
        blobs.append((full_bbox, area, circularity))

    blobs.sort(key=lambda b: b[1], reverse=True)
    return mask, blobs


def draw_blobs(frame: np.ndarray, blobs: list, color_bgr: tuple, label: str) -> None:
    for i, (bbox, area, circ) in enumerate(blobs):
        x, y, w, h = bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), color_bgr, 2)
        cx, cy = x + w // 2, y + h // 2
        cv2.drawMarker(frame, (cx, cy), color_bgr, cv2.MARKER_CROSS, 14, 2)
        tag = f"{'[LARGEST] ' if i == 0 else ''}{label}  A:{area:.0f}  C:{circ:.2f}"
        cv2.putText(frame, tag, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_bgr, 1)


def draw_hud(frame: np.ndarray,
             purple_vals: tuple, green_vals: tuple,
             n_purple: int, n_green: int) -> None:
    ph, ps, pv = purple_vals[0], purple_vals[2], purple_vals[4]
    ph2, ps2, pv2 = purple_vals[1], purple_vals[3], purple_vals[5]
    gh, gs, gv = green_vals[0], green_vals[2], green_vals[4]
    gh2, gs2, gv2 = green_vals[1], green_vals[3], green_vals[5]

    lines = [
        f"PURPLE  H:[{ph},{ph2}]  S:[{ps},{ps2}]  V:[{pv},{pv2}]  blobs:{n_purple}",
        f"GREEN   H:[{gh},{gh2}]  S:[{gs},{gs2}]  V:[{gv},{gv2}]  blobs:{n_green}",
        f"thresholds: area [{MIN_AREA},{MAX_AREA}]  circ >= {MIN_CIRCULARITY}",
        "[s]=save  [r]=reset  [q]=quit",
    ]
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (8, 18 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)


def apply_camera_controls(cap: cv2.VideoCapture, exposure_ms: int, gain: int) -> None:
    """Apply manual exposure and gain to the capture device."""
    # Disable auto-exposure (value 0.25 = manual on V4L2 / some AVFoundation drivers)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure_ms)
    cap.set(cv2.CAP_PROP_GAIN, gain)


def read_camera_controls() -> tuple:
    exp = cv2.getTrackbarPos("Exposure (ms)", WIN_CAM)
    gain = cv2.getTrackbarPos("Gain", WIN_CAM)
    return exp, gain


def reset_camera_controls() -> None:
    cv2.setTrackbarPos("Exposure (ms)", WIN_CAM, DEFAULT_EXPOSURE_MS)
    cv2.setTrackbarPos("Gain",          WIN_CAM, DEFAULT_GAIN)


def print_java_constants(purple_vals: tuple, green_vals: tuple,
                         exposure_ms: int, gain: int) -> None:
    ph, ph2, ps, ps2, pv, pv2 = purple_vals
    gh, gh2, gs, gs2, gv, gv2 = green_vals
    print("\n── ArtifactDetectionProcessor.java ──────────────────────────────────")
    print(f"    private static final Scalar PURPLE_HSV_LOWER = new Scalar({ph}, {ps}, {pv});")
    print(f"    private static final Scalar PURPLE_HSV_UPPER = new Scalar({ph2}, {ps2}, {pv2});")
    print(f"    private static final Scalar GREEN_HSV_LOWER  = new Scalar({gh}, {gs}, {gv});")
    print(f"    private static final Scalar GREEN_HSV_UPPER  = new Scalar({gh2}, {gs2}, {gv2});")
    print()
    print("── In your OpMode (after VisionManager.build()) ─────────────────────")
    print(f"    visionManager.lockCameraControls({exposure_ms}, {gain});")
    print("─────────────────────────────────────────────────────────────────────\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Artifact HSV Tuner")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default 0)")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"ERROR: Could not open camera {args.camera}")
        print("Try --camera 1 or --camera 2 if index 0 is your built-in camera.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Create windows
    cv2.namedWindow(WIN_MAIN,   cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_CTRL,   cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_CAM,    cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_PURPLE, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_GREEN,  cv2.WINDOW_NORMAL)

    cv2.resizeWindow(WIN_MAIN,   640, 480)
    cv2.resizeWindow(WIN_CTRL,   500, 240)
    cv2.resizeWindow(WIN_CAM,    500,  80)
    cv2.resizeWindow(WIN_PURPLE, 320, 240)
    cv2.resizeWindow(WIN_GREEN,  320, 240)

    create_trackbars(WIN_CTRL, "Purple")
    create_trackbars(WIN_CTRL, "Green")

    # Camera control trackbars — Exposure 0–200 ms, Gain 0–500
    cv2.createTrackbar("Exposure (ms)", WIN_CAM, DEFAULT_EXPOSURE_MS, 200, lambda _: None)
    cv2.createTrackbar("Gain",          WIN_CAM, DEFAULT_GAIN,        500, lambda _: None)

    print("Artifact Tuner running. Point camera at a purple or green ball.")
    print("[s] save  [r] reset  [q] quit")
    print("NOTE: if the Exposure/Gain sliders have no effect your camera/driver")
    print("      does not support manual control via OpenCV — that is fine.\n")

    ret, frame = cap.read()
    if not ret:
        print("ERROR: Could not read first frame.")
        sys.exit(1)

    full_h, full_w = frame.shape[:2]
    width_scale  = full_w / PROCESSING_W
    height_scale = full_h / PROCESSING_H

    prev_exp, prev_gain = -1, -1  # track changes to avoid spamming cap.set()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Apply camera controls only when sliders change
        exp, gain = read_camera_controls()
        if exp != prev_exp or gain != prev_gain:
            apply_camera_controls(cap, exp, gain)
            prev_exp, prev_gain = exp, gain

        small = cv2.resize(frame, (PROCESSING_W, PROCESSING_H), interpolation=cv2.INTER_LINEAR)
        hsv   = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        purple_vals = read_trackbars(WIN_CTRL, "Purple")
        green_vals  = read_trackbars(WIN_CTRL, "Green")

        p_lo = np.array([purple_vals[0], purple_vals[2], purple_vals[4]])
        p_hi = np.array([purple_vals[1], purple_vals[3], purple_vals[5]])
        g_lo = np.array([green_vals[0],  green_vals[2],  green_vals[4]])
        g_hi = np.array([green_vals[1],  green_vals[3],  green_vals[5]])

        purple_mask, purple_blobs = detect_blobs(hsv, p_lo, p_hi, width_scale, height_scale)
        green_mask,  green_blobs  = detect_blobs(hsv, g_lo, g_hi, width_scale, height_scale)

        annotated = frame.copy()
        draw_blobs(annotated, purple_blobs, (180, 0, 255), "Purple")
        draw_blobs(annotated, green_blobs,  (0, 200, 60),  "Green")
        draw_hud(annotated, purple_vals, green_vals,
                 len(purple_blobs), len(green_blobs))

        cv2.imshow(WIN_MAIN,   annotated)
        cv2.imshow(WIN_PURPLE, purple_mask)
        cv2.imshow(WIN_GREEN,  green_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print_java_constants(purple_vals, green_vals, exp, gain)
        elif key == ord('r'):
            reset_trackbars(WIN_CTRL, "Purple")
            reset_trackbars(WIN_CTRL, "Green")
            reset_camera_controls()
            print("Sliders reset to robot defaults.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
