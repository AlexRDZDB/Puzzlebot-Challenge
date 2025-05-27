import cv2
import numpy as np
import math

# ---------- Step 1: Load Image ----------
image_path = "src/image_processing/image_processing/GenerateMap/intersection.jpeg"  # <- change to your image path
img = cv2.imread(image_path)
if img is None:
    raise FileNotFoundError(f"Image not found at: {image_path}")

img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
clone = img.copy()
points = []

# ---------- Step 2: Select Two Points for Known Distance ----------
def click_points(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
        points.append((x, y))
        cv2.circle(clone, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Select Line", clone)

cv2.imshow("Select Line", clone)
cv2.setMouseCallback("Select Line", click_points)
print("[INFO] Click two points corresponding to a known real-world distance...")
cv2.waitKey(0)
cv2.destroyAllWindows()

if len(points) != 2:
    raise ValueError("You must select exactly two points.")

(x1, y1), (x2, y2) = points
pixel_length = math.hypot(x2 - x1, y2 - y1)

real_world_length = 1.0  # <-- CHANGE THIS to your known distance (meters)
pixels_per_meter = pixel_length / real_world_length
resolution = 1.0 / pixels_per_meter
print(f"[INFO] Pixels per meter: {pixels_per_meter:.2f}")
print(f"[INFO] ROS map resolution: {resolution:.6f} meters/pixel")

# ---------- Step 3: Select Four Corners for Homography ----------
homography_points = []
clone2 = img.copy()

def click_homography(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(homography_points) < 4:
        homography_points.append([x, y])
        cv2.circle(clone2, (x, y), 5, (255, 0, 0), -1)
        cv2.imshow("Select Corners", clone2)

cv2.imshow("Select Corners", clone2)
cv2.setMouseCallback("Select Corners", click_homography)
print("[INFO] Select four points (corners of the area to rectify)...")
cv2.waitKey(0)
cv2.destroyAllWindows()

if len(homography_points) != 4:
    raise ValueError("You must select exactly four points for homography.")

pts_src = np.array(homography_points, dtype="float32")

# ---------- Step 4: Define Destination Points Based on Real-World Dimensions ----------
# Estimate a reasonable size in meters (based on your known track size)
width_m = 1.0  # meters
height_m = 1.0  # meters

width_px = int(width_m * pixels_per_meter)
height_px = int(height_m * pixels_per_meter)

pts_dst = np.array([
    [0, 0],
    [width_px - 1, 0],
    [width_px - 1, height_px - 1],
    [0, height_px - 1]
], dtype="float32")

# ---------- Step 5: Compute Homography ----------
H = cv2.getPerspectiveTransform(pts_src, pts_dst)

# ---------- Step 6: Compute Bounds for Entire Image Warp ----------
h_img, w_img = img.shape[:2]
img_corners = np.array([
    [0, 0],
    [w_img - 1, 0],
    [w_img - 1, h_img - 1],
    [0, h_img - 1]
], dtype='float32').reshape(-1, 1, 2)

# Warp corners through homography
warped_corners = cv2.perspectiveTransform(img_corners, H).reshape(-1, 2)

[x_min, y_min] = np.floor(np.min(warped_corners, axis=0)).astype(int)
[x_max, y_max] = np.ceil(np.max(warped_corners, axis=0)).astype(int)

# Translation to keep image in positive space
tx = -x_min if x_min < 0 else 0
ty = -y_min if y_min < 0 else 0

T = np.array([
    [1, 0, tx],
    [0, 1, ty],
    [0, 0, 1]
])

H_full = T @ H

new_width = x_max - x_min
new_height = y_max - y_min

# ---------- Step 7: Warp the Full Image Without Cropping ----------
warped_full = cv2.warpPerspective(img, H_full, (new_width, new_height))

# ---------- Step 8: Show and Save Output ----------
cv2.imshow("Warped Full (Top-Down)", warped_full)
cv2.imwrite("rectified_full_map.png", warped_full)
cv2.waitKey(0)
cv2.destroyAllWindows()

print("[INFO] Full rectified image saved as 'rectified_full_map.png'")
print(f"[INFO] Image resolution for ROS map.yaml: {resolution:.6f} meters/pixel")
