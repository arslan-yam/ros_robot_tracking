import cv2
import numpy as np

lower_bound = np.array([90, 150, 150]) # blue marker
upper_bound = np.array([120, 255, 255])

def find_marker_center_bgr(frame: np.ndarray):
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("No marker was found")
        return None, mask

    marker = max(contours, key=cv2.contourArea)
    marker_area = cv2.contourArea(marker)
    if marker_area < 50:
        print("Noise")
        return None, mask

    M = cv2.moments(marker)
    if M["m00"] == 0:
        return None, mask

    center_x = int(M["m10"] / M["m00"])
    center_y = int(M["m01"] / M["m00"])
    print(f"Marker coordinates: x={center_x}, y={center_y}, area={marker_area:.2f}")
    return (center_x, center_y), mask

def main():
    image_path = "test.png"
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error occurred uploading {image_path}")
        return
    (center, mask) = find_marker_center_bgr(frame)
    if center is not None:
        cx, cy = center
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
    cv2.imshow("Original with marker", frame)
    cv2.imshow("Mask", mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
