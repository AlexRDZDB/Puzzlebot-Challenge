import cv2
import numpy as np

# Load binary image (white = track, black = background)
img = cv2.imread("black_line_mask.png", cv2.IMREAD_GRAYSCALE)

# Ensure binary (0 or 255)
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

# Define a kernel — adjust size based on how aggressive you want the cleanup
kernel = np.ones((3, 3), np.uint8)

# Apply erosion followed by dilation (opening)
eroded = cv2.erode(binary, kernel, iterations=2)
dilated = cv2.dilate(eroded, kernel, iterations=2)

# Save and show result
cv2.imshow("Opened Image", dilated)
cv2.imwrite("track_cleaned.png", dilated)
cv2.waitKey(0)
cv2.destroyAllWindows()
