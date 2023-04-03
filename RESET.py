

"""
RESET EVERYTHING

Run:
sudo pigpiod
sudo pip3 install adafruit-circuitpython-motorkit
sudo pip3 install giozero
*AND*
sudo apt-get install python-pigpio python3-pigpio
sudo apt install python3-pip


IP: 192.168.0.10
PW = 1234

"""

# ------------ Imports -----------

#General
from time import sleep
import math
import time
from signal import pause

#Board
import board
from adafruit_motorkit import MotorKit
import RPi.GPIO as GPIO

#Motor Control
from gpiozero.tools import sin_values
from gpiozero import Servo
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
pigpio_factory = PiGPIOFactory()


# ----- Global Variables -----
ULTRASONIC_GPIO_TRIGGER = 17
ULTRASONIC_GPIO_ECHO = 22
MICRO_SERVO_PIN = 27
MICRO_SERVO_CENTER_POINT = 0
BIG_SERVO_PIN = 12
DEFAULT_HAPTIC_VELOCITY = 0.0
WHEEL_OBJECT_OFFSET_ANGLE = 45 #in degrees
WHEEL_DISTANCE_THRESHOLD = 100 #in cm
HAPTIC_DISTANCE_THRESHOLD = 20 #in cm

# ------ Class ----------
class RESET():

    def __init__(self):
        self.kit = MotorKit()   #HAT Controller Kit
        self.micro_servo = AngularServo(MICRO_SERVO_PIN, min_angle=-90, max_angle=90, pin_factory=pigpio_factory)
        self.big_servo = AngularServo(BIG_SERVO_PIN, min_angle=-90, max_angle=90, pin_factory=pigpio_factory)

        #Ultrasonic
        GPIO.setmode(GPIO.BCM)
        # set GPIO Pins
        self.GPIO_TRIGGER = ULTRASONIC_GPIO_TRIGGER
        self.GPIO_ECHO = ULTRASONIC_GPIO_ECHO
        # set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)





        self.location = [0,0] #Array to store [angle, distance]

        #Main loop of code
        self.set_haptic_1(0)
        self.set_haptic_2(0)
        self.set_haptic_3(0)
        self.set_big_servo(0)
        self.set_micro_servo(0)
        self.micro_servo.detach()
        self.big_servo.detach()


    def set_micro_servo(self, degree):

        self.micro_servo.angle = degree

    def set_big_servo(self, degree):
        self.big_servo.angle = degree

    def set_haptic_1(self, velocity=DEFAULT_HAPTIC_VELOCITY):
        self.kit.motor1.throttle = velocity

    def set_haptic_2(self, velocity=DEFAULT_HAPTIC_VELOCITY):
        self.kit.motor2.throttle = velocity

    def set_haptic_3(self, velocity=DEFAULT_HAPTIC_VELOCITY):
        self.kit.motor3.throttle = velocity


if __name__ == "__main__":
    cane = RESET()



