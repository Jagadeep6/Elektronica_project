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
while i in range(50000):
	ut.stop()
	i+=1