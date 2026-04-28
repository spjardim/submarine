import cv2
import numpy as np
import os
import csv
import argparse
import time

# ---------------------------
# ARGS
# ---------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--playback", type=str, default=None,
                    help="Path to a run directory (e.g. runs/run_001) to replay instead of live capture")
parser.add_argument("--fps", type=int, default=None,
                    help="Playback FPS (omit for step mode: press Enter to advance)")
args = parser.parse_args()

PLAYBACK_MODE = args.playback is not None

# ---------------------------
# CONFIG (tune these defaults)
# ---------------------------
LOWER_HSV = np.array([0, 120, 20])
UPPER_HSV  = np.array([50, 255, 200])
RESOLUTION = (320, 240)
MIN_AREA   = 500
KERNEL_SIZE = 5

# ---------------------------
# TRACKBARS
# ---------------------------
TUNING_WINDOW = "HSV Tuning"
cv2.namedWindow(TUNING_WINDOW)

def nothing(x): pass

cv2.createTrackbar("H Low",  TUNING_WINDOW, int(LOWER_HSV[0]), 179, nothing)
cv2.createTrackbar("S Low",  TUNING_WINDOW, int(LOWER_HSV[1]), 255, nothing)
cv2.createTrackbar("V Low",  TUNING_WINDOW, int(LOWER_HSV[2]), 255, nothing)
cv2.createTrackbar("H High", TUNING_WINDOW, int(UPPER_HSV[0]), 179, nothing)
cv2.createTrackbar("S High", TUNING_WINDOW, int(UPPER_HSV[1]), 255, nothing)
cv2.createTrackbar("V High", TUNING_WINDOW, int(UPPER_HSV[2]), 255, nothing)

# Invisible 1x1 image so the trackbar window actually renders
cv2.imshow(TUNING_WINDOW, np.zeros((1, 300, 3), dtype=np.uint8))

def get_hsv_from_trackbars():
    hl = cv2.getTrackbarPos("H Low",  TUNING_WINDOW)
    sl = cv2.getTrackbarPos("S Low",  TUNING_WINDOW)
    vl = cv2.getTrackbarPos("V Low",  TUNING_WINDOW)
    hh = cv2.getTrackbarPos("H High", TUNING_WINDOW)
    sh = cv2.getTrackbarPos("S High", TUNING_WINDOW)
    vh = cv2.getTrackbarPos("V High", TUNING_WINDOW)
    return np.array([hl, sl, vl]), np.array([hh, sh, vh])

# ---------------------------
# SHARED PROCESSING FUNCTION
# ---------------------------
kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)
prev_cx, prev_cy = None, None
alpha = 0.7  # smoothing factor

def process_frame(frame):
    """
    Run color segmentation on a BGR frame using current trackbar HSV values.
    Returns (annotated_frame, mask, cx, cy) where cx/cy are None if not detected.
    """
    global prev_cx, prev_cy

    lower, upper = get_hsv_from_trackbars()

    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx, cy = None, None
    annotated = frame.copy()

    if contours:
        c    = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > MIN_AREA:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                if prev_cx is not None:
                    cx = int(alpha * cx + (1 - alpha) * prev_cx)
                    cy = int(alpha * cy + (1 - alpha) * prev_cy)

                prev_cx, prev_cy = cx, cy

                cv2.drawContours(annotated, [c], -1, (0, 255, 0), 2)
                cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(annotated, f"({cx},{cy})", (cx + 8, cy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    lower_cur, upper_cur = get_hsv_from_trackbars()
    cv2.putText(annotated,
                f"HSV Low: {lower_cur}  High: {upper_cur}",
                (5, annotated.shape[0] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

    return annotated, mask, cx, cy

# ===========================================================
# PLAYBACK MODE
# ===========================================================
if PLAYBACK_MODE:
    run_dir   = args.playback
    csv_path  = os.path.join(run_dir, "log.csv")
    img_dir   = os.path.join(run_dir, "captured_frames")

    if not os.path.exists(csv_path):
        print(f"[ERROR] No log.csv found in {run_dir}")
        exit(1)

    data = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(row)

    print(f"[PLAYBACK] Loaded {len(data)} frames from {run_dir}")
    if args.fps:
        print(f"[PLAYBACK] Running at {args.fps} FPS  |  SPACE = pause  |  ESC = quit")
        delay = int(1000 / args.fps)
    else:
        print("[PLAYBACK] Step mode  |  ENTER = next frame  |  ESC = quit")

    i = 0
    paused = False

    while i < len(data):
        row        = data[i]
        img_path   = row.get("image_path", os.path.join(img_dir, f"{int(row['frame']):06d}.jpg"))
        frame_id   = int(row["frame"])

        frame = cv2.imread(img_path)
        if frame is None:
            print(f"[WARN] Could not read {img_path}, skipping")
            i += 1
            continue

        frame = cv2.resize(frame, RESOLUTION)

        # Re-run segmentation with current trackbar values
        annotated, mask, cx, cy = process_frame(frame)

        # Overlay original logged centroid for comparison
        if "actual_x" in row and "actual_y" in row:
            orig_x = float(row["actual_x"])
            orig_y = float(row["actual_y"])
            if orig_x != -1:
                cv2.circle(annotated, (int(orig_x), int(orig_y)), 7, (255, 165, 0), 2)
                cv2.putText(annotated, "orig", (int(orig_x) + 8, int(orig_y) + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 165, 0), 1)

        # Frame counter overlay
        label = "PAUSED  " if paused else ""
        cv2.putText(annotated, f"{label}Frame {frame_id}  ({i+1}/{len(data)})",
                    (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Replay", annotated)
        cv2.imshow("Mask",   mask)

        # ----- key handling -----
        if args.fps and not paused:
            key = cv2.waitKey(delay) & 0xFF
        else:
            key = cv2.waitKey(0) & 0xFF

        if key == 27:           # ESC → quit
            break
        elif key == ord(' '):   # SPACE → toggle pause
            paused = not paused
            if paused:
                print(f"[PAUSED] frame {frame_id}  |  adjust trackbars, then SPACE to resume")
            continue            # re-process same frame so mask updates live while paused
        elif key == 13 or (args.fps and not paused):
            # Enter (step mode) or auto-advance (FPS mode)
            i += 1
        # any other key: stay on same frame (lets you tweak trackbars while paused in step mode)

    # Print final HSV values for copy-paste into your config
    lower_final, upper_final = get_hsv_from_trackbars()
    print("\n=== Final HSV values ===")
    print(f"LOWER_HSV = np.array({lower_final.tolist()})")
    print(f"UPPER_HSV  = np.array({upper_final.tolist()})")

    cv2.destroyAllWindows()

# ===========================================================
# LIVE CAPTURE MODE
# ===========================================================
else:
    try:
        from picamera2 import Picamera2
    except ImportError:
        print("[ERROR] picamera2 not found. Run with --playback <run_dir> for replay mode.")
        exit(1)

    picam2 = Picamera2(camera_num=0)
    config = picam2.create_still_configuration(main={"size": RESOLUTION})
    picam2.configure(config)
    picam2.start()

    print("[LIVE] Press ESC to quit. Adjust trackbars to tune HSV.")

    while True:
        raw = picam2.capture_array()
        frame = cv2.resize(raw, RESOLUTION)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        annotated, mask, cx, cy = process_frame(frame)

        if cx is not None:
            print(f"Target: ({cx}, {cy})")

        cv2.imshow("Live", annotated)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    lower_final, upper_final = get_hsv_from_trackbars()
    print("\n=== Final HSV values ===")
    print(f"LOWER_HSV = np.array({lower_final.tolist()})")
    print(f"UPPER_HSV  = np.array({upper_final.tolist()})")

    cv2.destroyAllWindows()
