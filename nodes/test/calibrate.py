import cv2
import numpy as np
import glob

# Chessboard settings (inner corners)
board_width = 8
board_height = 5
square_size = 0.020  # meters (e.g., 20 mm squares on your phone screen)

# Termination criteria for cornerSubPix
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points like (0,0,0), (1,0,0), (2,0,0), ..., multiplied by square size
objp = np.zeros((board_height * board_width, 3), np.float32)
objp[:, :2] = np.mgrid[0:board_width, 0:board_height].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points from all images
objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane.

# Load calibration images
images = glob.glob('calibration_images/img*.jpg')
if len(images) == 0:
    print("No images found in calibration_images/")
    exit()

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (board_width, board_height), None)

    # If found, add object points, refine corners and add image points
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display corners
        cv2.drawChessboardCorners(img, (board_width, board_height), corners2, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey(100)
    else:
        print(f"Chessboard not found in {fname}")

cv2.destroyAllWindows()

# Run camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print("Calibration successful!" if ret else "Calibration failed.")
print("\nCamera matrix:")
print(camera_matrix)

print("\nDistortion coefficients:")
print(dist_coeffs.ravel())

# Extract fx, fy, cx, cy
fx = camera_matrix[0,0]
fy = camera_matrix[1,1]
cx = camera_matrix[0,2]
cy = camera_matrix[1,2]

print(f"\nfx = {fx}")
print(f"fy = {fy}")
print(f"cx = {cx}")
print(f"cy = {cy}")
