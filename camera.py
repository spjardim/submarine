import os
from picamera2 import Picamera2
import time
from PIL import Image
import re

# -----------------------------
# CONFIGURATION
# -----------------------------
OUTPUT_DIR = "dataset"
IMAGE_DIR = os.path.join(OUTPUT_DIR, "images")

NUM_IMAGES = 1000            # total images to capture
RESOLUTION = (720, 720)     # capture resolution
CROP_BOX = None             # e.g., (x, y, width, height), or None for full frame
CAPTURE_INTERVAL = 0.3      # seconds between frames

# -----------------------------
# CREATE FOLDER
# -----------------------------
os.makedirs(IMAGE_DIR, exist_ok=True)

# -----------------------------
# DETERMINE STARTING INDEX
# -----------------------------
existing_files = os.listdir(IMAGE_DIR)
existing_numbers = []

for f in existing_files:
    match = re.match(r"image_(\d+)\.jpg", f)
    if match:
        existing_numbers.append(int(match.group(1)))

start_index = max(existing_numbers, default=-1) + 1

# -----------------------------
# INITIALIZE CAMERA
# -----------------------------
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": RESOLUTION})
picam2.configure(config)
picam2.start()
time.sleep(2)  # let camera adjust

# -----------------------------
# CAPTURE LOOP
# -----------------------------
for i in range(start_index, start_index + NUM_IMAGES):
    filename = f"image_{i}.jpg"
    filepath = os.path.join(IMAGE_DIR, filename)
    
    # Capture image
    image = picam2.capture_array()
    
    # Optional cropping
    if CROP_BOX:
        x, y, w, h = CROP_BOX
        image = image[y:y+h, x:x+w]
    
    # Save image
    Image.fromarray(image).save(filepath, quality=90)
    
    print(f"Saved {filename}")
    time.sleep(CAPTURE_INTERVAL)

picam2.stop()
print("Capture complete! Images saved in dataset/images.")
