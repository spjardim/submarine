from ultra_sonic_sensor import DistanceSensor
from imu_sensor import IMUSensor
from arduino_connection import MotorSender
from controllers import SubController
import csv 
import os
import json
import numpy as np
import cv2
from picamera2 import Picamera2
import time

if __name__ == "__main__":
    BASE_DIR = "runs"
    os.makedirs(BASE_DIR, exist_ok=True)

    # Find next run number
    existing_runs = [d for d in os.listdir(BASE_DIR) if d.startswith("run_")]
    run_numbers = []

    for d in existing_runs:
        try:
            num = int(d.split("_")[1])
            run_numbers.append(num)
        except:
            pass

    next_run = max(run_numbers, default=0) + 1
    RUN_DIR = os.path.join(BASE_DIR, f"run_{next_run:03d}")
    os.makedirs(RUN_DIR)

    # Create image folder inside run folder
    IMAGE_DIR = os.path.join(RUN_DIR, "captured_frames")
    os.makedirs(IMAGE_DIR)

    distance_sensor = DistanceSensor()
    imu = IMUSensor()
    gyro_offset = [0,0,0]

    if not os.path.exists("./gyro_calibration.json"):
        gyro_offset = imu.calibrate_gyro()
        with open("gyro_calibration.json", "w") as f:
            json.dump(gyro_offset, f)
    else:
        with open("gyro_calibration.json", "r") as f:
            gyro_offset = json.load(f)

    imu.set_gyro_offset(gyro_offset)
    sender = MotorSender()
    controller = SubController(distance_sensor, imu, sender)

    
    LOWER_HSV = np.array([0, 120, 20])
    UPPER_HSV = np.array([5, 255, 200])
    # LOWER_HSV = np.array([100, 120, 50])
    # UPPER_HSV = np.array([130, 255, 255])

    MIN_AREA = 350  # ignore small blobs (fish)
    KERNEL_SIZE = 5
    prev_cx, prev_cy = None, None
    alpha = 0.7  # smoothing factor

    RESOLUTION = (360, 360)     # capture resolution

    IMAGE_CENTER_U = RESOLUTION[0] / 2.0   # 180.0 — IBVS target in image space
    IMAGE_CENTER_V = RESOLUTION[1] / 2.0   # 180.0

    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"size": RESOLUTION})
    picam2.configure(config)
    picam2.start()
    kernel = np.ones((KERNEL_SIZE, KERNEL_SIZE), np.uint8)

    target_position=[]
    actual_position=[]
    motors_sent=[]
    target_pos = 800

    input("Press enter to start")
    testudo_found = False

    loop_freqs = []

    frame_count = 0
    try:
        prev_time = time.time()
        while True:
            current_time = time.time()
            dt = current_time - prev_time

            if dt > 0:
                freq = 1.0 / dt
            else:
                freq = 0

            prev_time = current_time

            frame = picam2.capture_array()
            # frame = cv2.resize(frame, (320, 320))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, LOWER_HSV, UPPER_HSV)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
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
                        testudo_found = True
            # Show debug windows
            # cv2.imshow("frame", frame)
            # cv2.imshow("mask", mask)
            if cv2.waitKey(1) == 27:
                break

            distance = distance_sensor.read_distance()
            gyro = imu.read_gyro()
            target_visible_this_frame = cx is not None
            if not target_visible_this_frame:
                testudo_found = False
                controller.reset_ibvs()

            if testudo_found:
                # Fall back to last known position if centroid was lost this frame
                u = cx if cx is not None else prev_cx if prev_cx is not None else IMAGE_CENTER_U
                v = cy if cy is not None else prev_cy if prev_cy is not None else IMAGE_CENTER_V
 
                # NOTE: pixel_v is kept in image convention (y increases downward).
                # The IBVS Jacobian already accounts for this — no manual flip needed.
                motors = controller.update_ibvs(
                    pixel_u=u,
                    pixel_v=v,
                    target_u=IMAGE_CENTER_U,
                    target_v=IMAGE_CENTER_V,
                )
                # ymotors = [255, 255, 255, 0]
                m3 = motors[0]
                m4 = motors[1]
                m1 = motors[2]
                m2 = motors[3]
                sender.send_to_arduino([m1, m2, m3, m4])
                print(
                    f"IBVS | pixel=({u}, {v}) | error=({u - IMAGE_CENTER_U:.1f}, {v - IMAGE_CENTER_V:.1f}) | "
                    f"motors={motors}"
                )
 
                target_position.append((IMAGE_CENTER_U, IMAGE_CENTER_V))
                actual_position.append((u, v))
                motors_sent.append(motors)
                loop_freqs.append(freq)
 
                image_filename = os.path.join(IMAGE_DIR, f"frame_{frame_count:05d}.jpg")
                cv2.imwrite(image_filename, frame)
                frame_count += 1
                

    except KeyboardInterrupt:
        sender.send_to_arduino([0, 0, 0, 0])
    finally:
        sender.send_to_arduino([0, 0, 0, 0])
        
    log_path = os.path.join(RUN_DIR, "log.csv")
    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "frame",
            "target_x", "target_y",
            "actual_x", "actual_y",
            "motor_1", "motor_2", "motor_3", "motor_4",
            "loop_freq",
            "image_path"
        ])        
        for i, (t, a, m, f) in enumerate(zip(target_position, actual_position, motors_sent, loop_freqs)):
            image_path = os.path.join(IMAGE_DIR, f"frame_{i:05d}.jpg")

            writer.writerow([
                i,
                t[0], t[1],
                a[0] if a[0] is not None else -1,
                a[1] if a[1] is not None else -1,
                m[0], m[1], m[2], m[3],
                f,
                image_path
            ])    
    metadata = {
        "lower_hsv": LOWER_HSV.tolist(),
        "upper_hsv": UPPER_HSV.tolist(),
        "min_area": int(MIN_AREA),
        "kernel_size": int(KERNEL_SIZE),
        "alpha": float(alpha),
        "resolution": list(RESOLUTION)    
    }
    meta_path = os.path.join(RUN_DIR, "metadata.json")
    with open(meta_path, "w") as f:
        json.dump(metadata, f, indent=4)


    if loop_freqs:
        print(f"Avg freq: {np.mean(loop_freqs):.2f} Hz")
        print(f"Min freq: {np.min(loop_freqs):.2f} Hz")
        print(f"Max freq: {np.max(loop_freqs):.2f} Hz")
