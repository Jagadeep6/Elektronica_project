from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import time
from threading import Thread
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


tolerance=0.1
x_deviation=0
y_max=0

i = 0

def track_object(gray,face_cascade,image):
    
    global delay
    global x_deviation, y_max, tolerance
    x_min = 0
    y_min = 0
    w = 0
    h = 0
    # Detect faces in the frame
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    for (x_min, y_min, w, h) in faces:
        cv2.rectangle(image, (x_min, y_min), (x_min+w, y_min+h), (0, 255, 0), 2)
        cv2.imshow('Frame', image)
    
    x_max = x_min+w
    y_max = y_min+h  
        
    obj_x_center=x_min+(w/2)
    obj_x_center=round(obj_x_center,3)
    
    obj_y_center=y_min+(h/2)
    obj_y_center=round(obj_y_center,3)
    
    x_deviation=round(0.5-obj_x_center,3)
    y_max=round(y_max,3)
        
    print("{",x_deviation,y_max,"}")
   
    thread = Thread(target = move_robot)
    thread.start()

def move_robot():
    global x_deviation, y_max, tolerance
    
    y=1-y_max #distance from bottom of the frame
    
    if(abs(x_deviation)<tolerance):
        if(y<0.1):
            ut.red_light("ON")
            ut.stop()
            print("reached person...........")
    
        else:
            ut.red_light("OFF")
            ut.forward()
            print("moving robot ...FORWARD....!!!!!!!!!!!!!!")
    
    
    else:
        ut.red_light("OFF")
        if(x_deviation>=tolerance):
            delay1=get_delay(x_deviation)
                
            ut.left()
            time.sleep(delay1)
            ut.stop()
            print("moving robot ...Left....<<<<<<<<<<")
    
                
        if(x_deviation<=-1*tolerance):
            delay1=get_delay(x_deviation)
                
            ut.right()
            time.sleep(delay1)
            ut.stop()
            print("moving robot ...Right....>>>>>>>>")

def get_delay(deviation):
    
    deviation=abs(deviation)
    
    if(deviation>=0.4):
        d=0.080
    elif(deviation>=0.35 and deviation<0.40):
        d=0.060
    elif(deviation>=0.20 and deviation<0.35):
        d=0.050
    else:
        d=0.040
    
    return d


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
    track_object(gray,face_cascade,image)
    key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        #rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
    if key == ord("q"):
      break
    rawCapture.truncate(0)
    i+=1