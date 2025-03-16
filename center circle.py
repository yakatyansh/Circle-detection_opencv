import cv2
import numpy as np

def find_circle_center(img):

    small_img = cv2.resize(img, (320, 240))


    gray = cv2.cvtColor(small_img, cv2.COLOR_BGR2GRAY)


    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)


    kernel = np.ones((3, 3), np.uint8)
    cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)


    blurred = cv2.GaussianBlur(cleaned, (5, 5), 0)


    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    valid_circles = []
    
    for cnt in contours:
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        radius = int(radius)

        
        perimeter = cv2.arcLength(cnt, True)
        approx_area = cv2.contourArea(cnt)
        circularity = (4 * np.pi * approx_area) / (perimeter ** 2 + 1e-5)

      
        if 0.75 < circularity < 1.2 and 25 < radius < 70:
            valid_circles.append((center[0], center[1], radius))

   
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                               param1=100, param2=50, minRadius=20, maxRadius=70)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        valid_circles.extend(circles)

   
    return max(valid_circles, key=lambda c: c[2]) if valid_circles else None

def calculate_distance(center1, center2):
    return np.sqrt((center2[0] - center1[0])**2 + (center2[1] - center1[1])**2)

def main():
    cap = cv2.VideoCapture(0)

   
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        height, width, _ = frame.shape
        camera_center = (width // 2, height // 2)
        camera_center_radius = 10  

        cv2.circle(frame, camera_center, camera_center_radius, (0, 0, 255), -1)

        circle = find_circle_center(frame)
        if circle is not None:
            x, y, r = circle
            circle_center = (x * 2, y * 2)  
            distance = calculate_distance(camera_center, circle_center)
            x_move = circle_center[0] - camera_center[0]
            y_move = circle_center[1] - camera_center[1]


            cv2.circle(frame, circle_center, r * 2, (0, 255, 0), 2)
            cv2.circle(frame, circle_center, 5, (0, 255, 0), -1)

            text_distance = f"Distance: {distance:.2f}"
            text_move = f"Move: ({x_move}, {y_move})"
        else:
            text_distance = "Distance: N/A"
            text_move = "Move: (N/A, N/A)"


        cv2.putText(frame, text_distance, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, text_move, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
