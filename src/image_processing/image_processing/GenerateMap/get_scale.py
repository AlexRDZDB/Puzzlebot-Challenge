import cv2
import math

points = []

def click_event(event, x, y, flags, param):
    global points, img_copy
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
        points.append((x, y))
        # Draw a circle on clicked point
        cv2.circle(img_copy, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Image", img_copy)

def calculate_distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def main():
    global points, img_copy
    image_path = "src/image_processing/image_processing/GenerateMap/track_cleaned.png"  # Change this to your image path
    img = cv2.imread(image_path)
    if img is None:
        print(f"Could not load image {image_path}")
        return

    img_copy = img.copy()
    cv2.imshow("Image", img_copy)
    cv2.setMouseCallback("Image", click_event)

    print("Click two points on the image with known real-world distance.")

    while True:
        key = cv2.waitKey(1) & 0xFF
        if len(points) == 2:
            pixel_dist = calculate_distance(points[0], points[1])
            print(f"Pixel distance between points: {pixel_dist:.2f}")
            real_dist = float(input("Enter the real-world distance (meters) between the points: "))
            meters_per_pixel = real_dist / pixel_dist
            print(f"Meters per pixel: {meters_per_pixel:.6f}")
            break
        if key == 27:  # ESC to exit
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
