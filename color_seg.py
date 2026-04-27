import cv2
import csv
import os
import argparse
import matplotlib.pyplot as plt

BASE_DIR = "runs"

parser = argparse.ArgumentParser()
parser.add_argument("--run_num", type=int, required=True)
parser.add_argument("--fps", type=int, default=None)
args = parser.parse_args()

RUN_DIR = os.path.join(BASE_DIR, f"run_{args.run_num:03d}")
IMAGE_FOLDER = os.path.join(RUN_DIR, "captured_frames")
CSV_PATH = os.path.join(RUN_DIR, "log.csv")

# ---------------------------
# LOAD CSV
# ---------------------------
data = []
with open(CSV_PATH, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        data.append(row)

# ---------------------------
# INIT PLOTS
# ---------------------------
plt.ion()

fig, axs = plt.subplots(4, 1, figsize=(8, 10))
# Data buffers
frames = []
error_xs = []
error_ys = []
freqs = []
m1s, m2s, m3s, m4s = [], [], [], []
target_xs, target_ys = [], []
actual_xs, actual_ys = [], []

if args.fps:
    delay = int(1000 / args.fps)
else:
    delay = None
# ---------------------------
# LOOP
# ---------------------------
for i, row in enumerate(data):

    frame_id = int(row["frame"])
    image_path = row["image_path"]

    frame = cv2.imread(image_path)
    if frame is None:
        continue

    # Extract
    # print(type(row["target_x"]))
    tx = float(row["target_x"])
    ty = float(row["target_y"])
    ax = float(row["actual_x"])
    ay = float(row["actual_y"])

    m1 = float(row["motor_1"])
    m2 = float(row["motor_2"])
    m3 = float(row["motor_3"])
    m4 = float(row["motor_4"])

    freq = float(row["loop_freq"])

    # Compute error
    if ax != -1 and ay != -1:
        ex = tx - ax
        ey = ty - ay
    else:
        ex, ey = 0, 0

    # Store data
    frames.append(frame_id)
    error_xs.append(ex)
    error_ys.append(ey)
    freqs.append(freq)

    m1s.append(m1)
    m2s.append(m2)
    m3s.append(m3)
    m4s.append(m4)

    target_xs.append(tx)
    target_ys.append(ty)

    # handle missing detection cleanly
    actual_xs.append(ax if ax != -1 else None)
    actual_ys.append(ay if ay != -1 else None)

    # ---------------------------
    # DRAW VIDEO
    # ---------------------------
    cv2.circle(frame, (int(tx), int(ty)), 6, (0, 255, 0), -1)
    if ax != -1:
        cv2.circle(frame, (int(ax), int(ay)), 6, (0, 0, 255), -1)

    cv2.putText(frame, f"Frame: {frame_id}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    cv2.imshow("Replay", frame)

    # ---------------------------
    # UPDATE PLOTS
    # ---------------------------
    for ax_plot in axs:
        ax_plot.clear()

    # Position plot (NEW TOP)
    axs[0].plot(frames, target_xs, label="Target X", linestyle="--")
    axs[0].plot(frames, target_ys, label="Target Y", linestyle="--")

    axs[0].plot(frames, actual_xs, label="Actual X")
    axs[0].plot(frames, actual_ys, label="Actual Y")

    axs[0].set_title("Target vs Actual Position")
    axs[0].legend()

    # Error plot
    axs[1].plot(frames, error_xs, label="Error X")
    axs[1].plot(frames, error_ys, label="Error Y")
    axs[1].set_title("Tracking Error")
    axs[1].legend()

    # Motor plot
    axs[2].plot(frames, m1s, label="M1")
    axs[2].plot(frames, m2s, label="M2")
    axs[2].plot(frames, m3s, label="M3")
    axs[2].plot(frames, m4s, label="M4")
    axs[2].set_title("Motor Outputs")
    axs[2].legend()

    # Frequency plot
    axs[3].plot(frames, freqs, label="Loop Hz")
    axs[3].set_title("Loop Frequency")
    axs[3].legend()

    plt.pause(0.001)

    # ---------------------------
    # CONTROL
    # ---------------------------
    if args.fps:
        key = cv2.waitKey(delay)
    else:
        # Step mode: wait indefinitely for key
        key = cv2.waitKey(0)

    # Controls
    if key == 27:  # ESC
        break

    # In step mode: only advance on Enter
    if not args.fps:
        if key == 13:  # Enter key
            continue
        else:
            # ignore all other keys except ESC
            continue

    # In FPS mode: allow pause with space
    if args.fps and key == ord(' '):
        while True:
            k = cv2.waitKey(0)
            if k == ord(' '):
                break
cv2.destroyAllWindows()
plt.ioff()
plt.show()