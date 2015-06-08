import lib.GPIO_Intel as GPIO
import time

GPIO = GPIO.Intel()

GPIO.setup("IO2", "out")
GPIO.setup("IO3", "out")

while True:
	Ultra = GPIO.ultrasonic("IO2", "IO3")
	print Ultra
	time.sleep(1)
