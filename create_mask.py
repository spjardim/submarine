import cv2
import numpy as np
import os

IMAGE_DIR = "dataset/images"
MASK_DIR  = "dataset/masks"

os.makedirs(MASK_DIR, exist_ok=True)

image_files = sorted([f for f in os.listdir(IMAGE_DIR) if f.endswith(".jpg")])

drawing = False
mode = "draw"  # "draw" or "erase"
brush_size = 8

def draw(event, x, y, flags, param):
    global drawing, mode, mask

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        mode = "draw"
    elif event == cv2.EVENT_RBUTTONDOWN:
        drawing = True
        mode = "erase"
    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        if mode == "draw":
            cv2.circle(mask, (x, y), brush_size, 255, -1)
        else:
            cv2.circle(mask, (x, y), brush_size, 0, -1)
    elif event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_RBUTTONUP:
        drawing = False

def overlay(image, mask):
    overlay = image.copy()
    overlay[mask == 255] = (0, 0, 255)  # red overlay
    return cv2.addWeighted(image, 0.7, overlay, 0.3, 0)

i = 0

while i < len(image_files):
    filename = image_files[i]
    path = os.path.join(IMAGE_DIR, filename)

    image = cv2.imread(path)
    mask_path = os.path.join(MASK_DIR, filename.replace(".jpg", ".png"))

    if os.path.exists(mask_path):
        mask = cv2.imread(mask_path, 0)
    else:
        mask = np.zeros(image.shape[:2], dtype=np.uint8)

    cv2.namedWindow("Mask Painter")
    cv2.setMouseCallback("Mask Painter", draw)

    while True:
        display = overlay(image, mask)

        cv2.putText(display, f"{filename} | s=save n=next q=quit",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        cv2.imshow("Mask Painter", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            cv2.imwrite(mask_path, mask)
            print(f"Saved mask: {mask_path}")

        elif key == ord('n'):
            i += 1
            break

        elif key == ord('q'):
            cv2.destroyAllWindows()
            exit()

        elif key == ord('c'):
            mask[:] = 0  # clear mask

        elif key == ord('+') or key == ord('='):
            brush_size += 2

        elif key == ord('-') and brush_size > 2:
            brush_size -= 2

    cv2.destroyAllWindows()

print("Done labeling!")
