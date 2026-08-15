"""
Pollen Ball HSV Tuner (BioBuzz) - Optimized for Wiffle/Pollen Ball
==================================================================
Tuned specifically for yellow perforated balls against neutral backgrounds.
"""

import argparse
import sys
import cv2
import numpy as np

# ── Resolution and Morphology ────────────────────────────────────────────────
PROCESSING_W, PROCESSING_H = 640, 480
MORPH_OPEN_SIZE  = (3, 3)
MORPH_CLOSE_SIZE = (9, 9)  # Larger size bridges holes in wiffle-style balls

# ── Updated Defaults based on image sample ───────────────────────────────────
DEFAULTS = {
    "Yellow": {"h_lo": 18, "h_hi": 32, "s_lo": 73, "s_hi": 255, "v_lo": 149, "v_hi": 255},
}

# Detection thresholds
MIN_AREA        = 100
MAX_AREA        = 60_000
MIN_CIRCULARITY = 0.45

# ── Window names ──────────────────────────────────────────────────────────────
WIN_MAIN   = "Pollen Tuner — Live Feed  [s=save  r=reset  q=quit]"
WIN_YELLOW = "Mask: Yellow"
WIN_CTRL   = "HSV Controls"

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
        cv2.drawMarker(frame, (cx, cy), color_bgr, cv2.MARKER_CROSS, 12, 2)
        tag = f"{'[LARGEST] ' if i == 0 else ''}{label}  A:{area:.0f}  C:{circ:.2f}"
        cv2.putText(frame, tag, (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_bgr, 1)


def print_java_constants(yellow_vals: tuple) -> None:
    yh, yh2, ys, ys2, yv, yv2 = yellow_vals
    print("\n── Java Constants ─────────────────────────────────────────────────")
    print(f"    private static final Scalar YELLOW_HSV_LOWER = new Scalar({yh}, {ys}, {yv});")
    print(f"    private static final Scalar YELLOW_HSV_UPPER = new Scalar({yh2}, {ys2}, {yv2});")
    print("───────────────────────────────────────────────────────────────────\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Pollen Ball HSV Tuner")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default 0)")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"ERROR: Could not open camera {args.camera}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    cv2.namedWindow(WIN_MAIN,   cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_CTRL,   cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_YELLOW, cv2.WINDOW_NORMAL)

    cv2.resizeWindow(WIN_MAIN,   640, 480)
    cv2.resizeWindow(WIN_CTRL,   500, 160)
    cv2.resizeWindow(WIN_YELLOW, 320, 240)

    create_trackbars(WIN_CTRL, "Yellow")

    ret, frame = cap.read()
    if not ret:
        print("ERROR: Could not read first frame.")
        sys.exit(1)

    full_h, full_w = frame.shape[:2]
    width_scale  = full_w / PROCESSING_W
    height_scale = full_h / PROCESSING_H

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        small = cv2.resize(frame, (PROCESSING_W, PROCESSING_H), interpolation=cv2.INTER_LINEAR)
        hsv   = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        yellow_vals = read_trackbars(WIN_CTRL, "Yellow")

        y_lo = np.array([yellow_vals[0], yellow_vals[2], yellow_vals[4]])
        y_hi = np.array([yellow_vals[1], yellow_vals[3], yellow_vals[5]])

        yellow_mask, yellow_blobs = detect_blobs(hsv, y_lo, y_hi, width_scale, height_scale)

        annotated = frame.copy()
        draw_blobs(annotated, yellow_blobs, (0, 255, 255), "Yellow")

        cv2.imshow(WIN_MAIN,   annotated)
        cv2.imshow(WIN_YELLOW, yellow_mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print_java_constants(yellow_vals)
        elif key == ord('r'):
            reset_trackbars(WIN_CTRL, "Yellow")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()