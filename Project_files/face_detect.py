import cv2
from picamera import PiCamera
from time import sleep

camera = PiCamera()
# Load the pre-trained face detection classifier
face_cascade = cv2.CascadeClassifier('/haarcascade_frontalface_default.xml')

# Set up the camera
cap = cv2.VideoCapture(1)
cap.set(3, 320)  # Set the width of the camera
cap.set(4, 240)  # Set the height of the camera

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the frame
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Draw bounding boxes around the detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('Real-time Face Detection', frame)

    # Wait for the user to press 'q' to exit
    if cv2.waitKey(1) == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()