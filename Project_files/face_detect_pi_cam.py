from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
# initialize the camera and grab a reference to the raw camera capture
cap = PiCamera()
cap.resolution = (640, 480)
cap.framerate = 32
rawCapture = PiRGBArray(cap, size=(640, 480))
face_cascade = cv2.CascadeClassifier('/haarcascade_frontalface_default.xml')
# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera
for frame in cap.capture_continuous(rawCapture, format="bgr", use_video_port=True):
  image = np.array(frame)
  gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
  faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
  for (x, y, w, h) in faces:
    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
	# show the frame
    cv2.imshow("Frame", image)
    key = cv2.waitKey(1) & 0xFF
	# clear the stream in preparation for the next frame
    rawCapture.truncate(0)
	# if the `q` key was pressed, break from the loop
  if key == ord("q"):
    break