import cv2
import numpy as np
import os
from picamera2 import Picamera2
import time
from PIL import Image
import re
# ---------------------------
# CONFIG (tune this)
# ---------------------------

# Example HSV range (you WILL tune this)
LOWER_HSV = np.array([0, 120, 20])
UPPER_HSV = np.array([50, 255, 200])
RESOLUTION = (320, 320)     # capture resolution

MIN_AREA = 500  # ignore small blobs (fish)
KERNEL_SIZE = 5

# ---------------------------
# INIT
# ---------------------------

picam2 = Picamera2(camera_num=0)
config = picam2.create_still_configuration(main={"size": RESOLUTION})
picam2.configure(config)
picam2.start()

kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)

# Optional: for smoothing
prev_cx, prev_cy = None, None
alpha = 0.7  # smoothing factor

# ---------------------------
# LOOP
# ---------------------------
print("Hello")
while True:
    frame = picam2.capture_array()

    # Resize for speed (important on Pi)
    frame = cv2.resize(frame, (320, 240))
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)
    # Threshold
    mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)

    # Morphological filtering (removes fish noise / bubbles)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx, cy = None, None

    if contours:
        # Pick largest contour (assumes statue is biggest colored object)
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > MIN_AREA:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Smooth centroid (helps control stability)
                if prev_cx is not None:
                    cx = int(alpha * cx + (1 - alpha) * prev_cx)
                    cy = int(alpha * cy + (1 - alpha) * prev_cy)

                prev_cx, prev_cy = cx, cy

                # Draw results
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                # 👉 THIS is your control signal
                print(f"Target: ({cx}, {cy})")

    # Show debug windows
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
