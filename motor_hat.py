import time
import board
from adafruit_motorkit import MotorKit

kit = MotorKit()

while True:
	kit.motor1.throttle = 1.0
	time.sleep(0.5)
	kit.motor1.throttle = 0
	time.sleep(0.5)
	kit.motor2.throttle = 1.0
	time.sleep(0.5)
	kit.motor2.throttle = 0
	time.sleep(0.5)
	kit.motor3.throttle = 1.0
	time.sleep(0.5)
	kit.motor3.throttle = 0
	time.sleep(1)