import cv2
import numpy as np

# Open webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Resize (optional)
    frame = cv2.resize(frame, (640, 480))

    # Convert BGR → HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # COLOR RANGE FOR GREEN BALL
    lower = np.array([35, 50, 50])
    upper = np.array([85, 255, 255])

    # Create mask
    mask = cv2.inRange(hsv, lower, upper)

    # Remove noise
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) > 0:

        # Largest contour
        c = max(contours, key=cv2.contourArea)

        # Get enclosing circle
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        # Get center
        M = cv2.moments(c)

        if M["m00"] != 0:
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])

            # Draw circle
            cv2.circle(frame, (center_x, center_y),
                       int(radius), (0,255,0), 2)

            # Draw center
            cv2.circle(frame, (center_x, center_y),
                       5, (0,0,255), -1)

            print("Ball position:", center_x, center_y)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    # ESC key exits
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
