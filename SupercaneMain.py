

"""
Loop(){
    - Input
    1. Turn ultrasnoic servo
    1.5 Check ultrasonic Distance
    2. Check camera distance
    2.5 (Optional) Check GPS location

    - Logic
    If no object detected continue - cary on
    Elif object detected - use array [distance, angle (0 - 180)]

    - Output
    3. Haptic
    4. Servo
    5. Audio
}




"""
# ------------ Imports -----------
from gpiozero import Servo
from gpiozero import AngularServo
from time import sleep
from gpiozero.tools import sin_values
from signal import pause

import time
import board
from adafruit_motorkit import MotorKit

import RPi.GPIO as GPIO
import time

# ----- Global Variables -----
ULTRASONIC_GPIO_TRIGGER = 17
ULTRASONIC_GPIO_ECHO = 22

# ------ Class ----------
class Supercane():

    def __init__(self):
        print("do nothing yet")
        dist = self.get_ultrasonic_distance()
        print("Measured Distance = %.1f cm" % dist)


    def get_ultrasonic_distance(self):
        GPIO.setmode(GPIO.BCM)

        # set GPIO Pins
        GPIO_TRIGGER = ULTRASONIC_GPIO_TRIGGER
        GPIO_ECHO = ULTRASONIC_GPIO_ECHO

        # set GPIO direction (IN / OUT)
        GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(GPIO_ECHO, GPIO.IN)


        GPIO.output(GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()

        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2

        return distance

    def get_camera_data(self):

        return None


    def get_GPS_data(self):

        return None

    def set_micro_servo(self, degree):
        servo = AngularServo(17, min_angle=-90, max_angle=90)

        # servo.source = sin_values()
        servo.angle = degree
        # servo.source_delay = 0.3

        # pause()


    def big_servo_test(self):

        servo = Servo(12)

        servo.source = sin_values()
        servo.source_delay = 0.03

        pause()

    def micro_servp_test(self):

        servo = Servo(17)

        servo.source = sin_values()
        servo.source_delay = 0.3

        pause()


    def motor_hat_test(self):


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


    def ultrasonic_sensor_test(self):
        # GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)

        # set GPIO Pins
        GPIO_TRIGGER = 17
        GPIO_ECHO = 22

        # set GPIO direction (IN / OUT)
        GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(GPIO_ECHO, GPIO.IN)

        try:
            while True:

                GPIO.output(GPIO_TRIGGER, True)

                # set Trigger after 0.01ms to LOW
                time.sleep(0.00001)
                GPIO.output(GPIO_TRIGGER, False)

                StartTime = time.time()
                StopTime = time.time()

                # save StartTime
                while GPIO.input(GPIO_ECHO) == 0:
                    StartTime = time.time()

                # save time of arrival
                while GPIO.input(GPIO_ECHO) == 1:
                    StopTime = time.time()

                # time difference between start and arrival
                TimeElapsed = StopTime - StartTime
                # multiply with the sonic speed (34300 cm/s)
                # and divide by 2, because there and back
                distance = (TimeElapsed * 34300) / 2

                dist = distance
                print("Measured Distance = %.1f cm" % dist)
                time.sleep(1)

            # Reset by pressing CTRL + C
        except KeyboardInterrupt:
            print("Measurement stoppe")


if __name__ == "__main__":
    cane = Supercane()
    # cane.ultrasonic_sensor_test()




