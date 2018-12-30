#!/usr/bin/python
# -*- coding:utf-8 -*-
import RPi.GPIO as GPIO
from AlphaBot2 import AlphaBot2
from neopixel import *
from TRSensors import TRSensor
import time

Button = 7

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Button,GPIO.IN,GPIO.PUD_UP)

# LED strip configuration:
LED_COUNT      = 4      # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (must support PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 5       # DMA channel to use for generating signal (try 5)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)	

maximum = 100;
j = 0
integral = 0;
last_proportional = 0

def Wheel(pos):
#	"""Generate rainbow colors across 0-255 positions."""
	if pos < 85:
		return Color(pos * 3, 255 - pos * 3, 0)
	elif pos < 170:
		pos -= 85
		return Color(255 - pos * 3, 0, pos * 3)
	else:
		pos -= 170
		return Color(0, pos * 3, 255 - pos * 3)

# Create NeoPixel object with appropriate configuration.
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
# Intialize the library (must be called once before other functions).
strip.begin()
strip.setPixelColor(0, Color(100, 0, 0))       #Red
strip.setPixelColor(1, Color(0, 100, 0))       #Blue
strip.setPixelColor(2, Color(0, 0, 100))       #Green
strip.setPixelColor(3, Color(100, 100, 0))     #Yellow
strip.show()

TR = TRSensor()
Ab = AlphaBot2()
Ab.stop()
print("Line follow Example")
time.sleep(0.5)
for i in range(0,100):
	if(i<25 or i>= 75):
		Ab.right()
		Ab.setPWMA(30)
		Ab.setPWMB(30)
	else:
		Ab.left()
		Ab.setPWMA(30)
		Ab.setPWMB(30)
	TR.calibrate()
Ab.stop()
print(TR.calibratedMin)
print(TR.calibratedMax)
while (GPIO.input(Button) != 0):
	position,Sensors = TR.readLine()
	print(position,Sensors)
	time.sleep(0.05)
Ab.forward()

while True:
	position,Sensors = TR.readLine()
	#print(position)
	if(Sensors[0] >900 and Sensors[1] >900 and Sensors[2] >900 and Sensors[3] >900 and Sensors[4] >900):
		Ab.setPWMA(0)
		Ab.setPWMB(0);
	else:
		# The "proportional" term should be 0 when we are on the line.
		proportional = position - 2000
		
		# Compute the derivative (change) and integral (sum) of the position.
		derivative = proportional - last_proportional
		integral += proportional
		
		# Remember the last position.
		last_proportional = proportional

		'''
		// Compute the difference between the two motor power settings,
		// m1 - m2.  If this is a positive number the robot will turn
		// to the right.  If it is a negative number, the robot will
		// turn to the left, and the magnitude of the number determines
		// the sharpness of the turn.  You can adjust the constants by which
		// the proportional, integral, and derivative terms are multiplied to
		// improve performance.
		'''
		power_difference = proportional/30  + integral/10000 + derivative*2;  

		if (power_difference > maximum):
			power_difference = maximum
		if (power_difference < - maximum):
			power_difference = - maximum
		print(position,power_difference)
		if (power_difference < 0):
			Ab.setPWMA(maximum + power_difference)
			Ab.setPWMB(maximum);
		else:
			Ab.setPWMA(maximum);
			Ab.setPWMB(maximum - power_difference)
		
	for i in range(0,strip.numPixels()):
		strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255))
	strip.show();
	j += 1
	if(j > 256*4): 
		j= 0
