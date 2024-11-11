import cv2
import numpy as np

def find_circle_center(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1.2, 100)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            return (x, y)
    else:
        return None

def calculate_distance(center1, center2):
    x_diff = center2[0] - center1[0]
    y_diff = center2[1] - center1[1]
    distance = np.sqrt(x_diff**2 + y_diff**2)
    return distance

def main():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        circle_center = find_circle_center(frame)

        if circle_center is not None:
            camera_center = (frame.shape[1] // 2, frame.shape[0] // 2)
            distance = calculate_distance(camera_center, circle_center)
            x_move = circle_center[0] - camera_center[0]
            y_move = circle_center[1] - camera_center[1]

            cv2.circle(frame, camera_center, 5, (0, 0, 255), -1)
            cv2.circle(frame, circle_center, 5, (0, 255, 0), -1)

            cv2.putText(frame, f"Distance: {distance:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, f"Move: ({x_move}, {y_move})", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()