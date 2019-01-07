import RPi.GPIO as GPIO
import time
# from AlphaBot2 import AlphaBot2

#Ab = AlphaBot2()
Ab = None

DR = 16
DL = 19

def setup(robot):
	Ab = None
	#Ab = AlphaBot2()
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(DR,GPIO.IN,GPIO.PUD_UP)
	GPIO.setup(DL,GPIO.IN,GPIO.PUD_UP)

def getStatus():
	DR_status = (GPIO.input(DR) + 1) % 2
	DL_status = (GPIO.input(DL) + 1) % 2
	return DL_status, DR_status

def obstacleAvoidance():
	while True:
		DL_status, DR_status = getStatus()
		#	print(DR_status,DL_status)
		if((DL_status == 0) or (DR_status == 0)):
			Ab.left()
			#Ab.right()
			time.sleep(0.002)
			Ab.stop()
		#	print("object")
		else:
			Ab.forward()
		#	print("forward")

'''
try:
	while True:
		DR_status = GPIO.input(DR)
		DL_status = GPIO.input(DL)
#		print(DR_status,DL_status)
		if((DL_status == 0) or (DR_status == 0)):
			Ab.left()
			#Ab.right()
			time.sleep(0.002)
			Ab.stop()
		#	print("object")
		else:
			Ab.forward()
		#	print("forward")

except KeyboardInterrupt:
	GPIO.cleanup();
'''
