import os
import cv2
import numpy as np

IMAGE_DIR = "dataset/images"
MASK_DIR  = "dataset/masks"
USE_LAB = True   # switch between HSV and LAB

# -----------------------------
# LOAD DATA
# -----------------------------
image_files = sorted([f for f in os.listdir(IMAGE_DIR) if f.endswith(".jpg")])

target_pixels = []

for fname in image_files:
    img_path  = os.path.join(IMAGE_DIR, fname)
    mask_path = os.path.join(MASK_DIR, fname.replace(".jpg", ".png"))

    if not os.path.exists(mask_path):
        continue

    img  = cv2.imread(img_path)
    mask = cv2.imread(mask_path, 0)

    if USE_LAB:
        img_cs = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    else:
        img_cs = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Extract target pixels
    pixels = img_cs[mask == 255]

    if len(pixels) > 0:
        target_pixels.append(pixels)

# Stack all pixels
target_pixels = np.vstack(target_pixels)

print(f"Collected {len(target_pixels)} target pixels")

# -----------------------------
# TRAIN GAUSSIAN MODEL
# -----------------------------
mean = np.mean(target_pixels, axis=0)
cov  = np.cov(target_pixels.T)

# Regularization (important!)
cov += np.eye(3) * 1e-6

inv_cov = np.linalg.inv(cov)

print("\n=== MODEL PARAMETERS ===")
print("Mean:\n", mean)
print("Covariance:\n", cov)

# -----------------------------
# SAVE MODEL
# -----------------------------
np.savez("color_model.npz", mean=mean, cov=cov, inv_cov=inv_cov, use_lab=USE_LAB)

print("\nModel saved to color_model.npz")
