import cv2
import time
import os

# Change this to your camera device index or video stream URL if needed
camera_index = 0

# Directory to save images
save_dir = "calibration_images"
os.makedirs(save_dir, exist_ok=True)

cap = cv2.VideoCapture(camera_index)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

img_count = 1

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        filename = os.path.join(save_dir, f"img{img_count:02d}.jpg")
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")

        img_count += 1
        time.sleep(1)  # wait 1 second between captures

except KeyboardInterrupt:
    print("Capture stopped by user")

cap.release()
cv2.destroyAllWindows()
