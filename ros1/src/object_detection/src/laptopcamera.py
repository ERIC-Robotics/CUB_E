#!/usr/bin/env python3
import cv2

# Create a VideoCapture object
cap = cv2.VideoCapture(2)  # 0 is the default camera index, you can change it if needed

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Check if the frame was captured successfully
    if not ret:
        print("Failed to capture frame")
        break

    # Display the captured frame
    cv2.imshow("Camera Feed", frame)

    # Exit the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the OpenCV window
cap.release()
cv2.destroyAllWindows()
