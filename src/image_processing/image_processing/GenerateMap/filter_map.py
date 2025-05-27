import cv2
import numpy as np

# ---------- Step 1: Load Rectified Image ----------
image_path = "newmap.png"  # <- Use your warped image here
img = cv2.imread(image_path)

if img is None:
    raise FileNotFoundError(f"Image not found at: {image_path}")

# ---------- Step 2: Convert to Grayscale ----------
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# ---------- Step 3: Apply Threshold to Extract Black Lines ----------
# Tune the threshold value depending on your lighting
# Lower threshold = allows more dark gray to count as "black"
_, binary_mask = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY_INV)

# ---------- Step 4: Optional Morphology (clean mask) ----------
kernel = np.ones((3, 3), np.uint8)
mask_cleaned = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)

# ---------- Step 5: Save and Show ----------
cv2.imshow("Black Line Mask", mask_cleaned)
cv2.imwrite("black_line_mask.png", mask_cleaned)
cv2.waitKey(0)
cv2.destroyAllWindows()

print("[INFO] Mask with black lines saved as 'black_line_mask.png'")
