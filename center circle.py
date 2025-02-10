import cv2
import numpy as np

def find_circle_center(img):
    # Resize frame for faster processing
    small_img = cv2.resize(img, (320, 240))

    # Convert to grayscale
    gray = cv2.cvtColor(small_img, cv2.COLOR_BGR2GRAY)

    # Apply brightness normalization
    gray = cv2.equalizeHist(gray)

    # Apply Gaussian Blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Use a mask to detect circles only in the center region
    height, width = blurred.shape
    mask = np.zeros_like(blurred)
    cv2.circle(mask, (width//2, height//2), min(width, height)//3, 255, -1)
    blurred = cv2.bitwise_and(blurred, blurred, mask=mask)

    # Detect circles with strict parameters
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                               param1=100, param2=70, minRadius=30, maxRadius=100)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        return max(circles, key=lambda c: c[2])  # Return the largest detected circle
    return None

def calculate_distance(center1, center2):
    return np.sqrt((center2[0] - center1[0])**2 + (center2[1] - center1[1])**2)

def main():
    cap = cv2.VideoCapture(0)

    # Set lower resolution for better performance
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    frame_skip = 2  # Process every 2nd frame
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        if frame_count % frame_skip != 0:
            continue  # Skip processing some frames

        height, width, _ = frame.shape
        camera_center = (width // 2, height // 2)
        camera_center_radius = 10  # **Small solid camera center**

        # Always show the camera center as a **solid dot**
        cv2.circle(frame, camera_center, camera_center_radius, (0, 0, 255), -1)

        circle = find_circle_center(frame)
        if circle is not None:
            x, y, r = circle
            circle_center = (x * 2, y * 2)  # Scale back to original frame size
            distance = calculate_distance(camera_center, circle_center)
            x_move = circle_center[0] - camera_center[0]
            y_move = circle_center[1] - camera_center[1]

            # Draw detected circle
            cv2.circle(frame, circle_center, r * 2, (0, 255, 0), 2)
            cv2.circle(frame, circle_center, 5, (0, 255, 0), -1)

            text_distance = f"Distance: {distance:.2f}"
            text_move = f"Move: ({x_move}, {y_move})"
        else:
            text_distance = "Distance: N/A"
            text_move = "Move: (N/A, N/A)"

        # Always keep text visible
        cv2.putText(frame, text_distance, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, text_move, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
