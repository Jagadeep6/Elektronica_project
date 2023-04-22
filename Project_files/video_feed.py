from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import util as ut

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(20,GPIO.OUT)

ut.init_gpio()
i = 0
GPIO.output(20, True)
GPIO.output(21, True)

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

i = 0

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Draw bounding boxes around the detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        print("got coords")
        # show the frame
    cv2.imshow("Frame", image)
    if(i < 20):
      ut.forward()
    elif(i > 20 and i < 40):
      ut.stop()
    elif(i > 40 and i <60):
      ut.left()
    elif(i > 60 and i < 80):
      ut.stop()

    print("started")
    key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        #rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
    if key == ord("q"):
      break
    rawCapture.truncate(0)
    i+=1