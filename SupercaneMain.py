

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
DISTANCE_THRESHOLD = 100 #in cm
HAPTIC_DISTANCE_THRESHOLD = 20 #in cm

# ------ Class ----------
class Supercane():

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

        try:
            self.main()

        except:
            raise Exception("encountered an error")
            self.set_haptic_1(0)
            self.set_haptic_2(0)
            self.set_haptic_3(0)
            self.set_micro_servo(0)
            self.set_big_servo(0)



    def main(self):
        angle = 0
        angle_polarity = 0
        while True:
            sleep(1)

            #Micro Servo
            ANG_UPPER_LIMIT = 140 + MICRO_SERVO_CENTER_POINT
            ANG_LOWER_LIMIT = 40 + MICRO_SERVO_CENTER_POINT
            INCRIMENT_BY = 10

            if angle_polarity == 0:
                angle += INCRIMENT_BY
                if angle >= ANG_UPPER_LIMIT:
                    angle_polarity = 1
            else:
                angle -= INCRIMENT_BY
                if angle <= ANG_LOWER_LIMIT:
                    angle_polarity = 0

            ang = angle - 90
            self.set_micro_servo(ang)
            print(ang)
            self.location[0] = ang

            #Read Distance
            distance_ultra = self.get_ultrasonic_distance()
            distance_camera = self.get_camera_data()
            self.location[1] = distance_ultra

            #Wheel feedback
            if self.location[1] < DISTANCE_THRESHOLD:
                big_servo_angle = 0
                if ang > 0:
                    big_servo_angle = self.location[0] - WHEEL_OBJECT_OFFSET_ANGLE
                else:
                    big_servo_angle = self.location[0] + WHEEL_OBJECT_OFFSET_ANGLE
                self.set_big_servo(big_servo_angle)

            #Haptic Feedback
            if self.location[1] < HAPTIC_DISTANCE_THRESHOLD:
                self.haptic_handler() #Handles haptic feedback proportions


            else:
                self.set_haptic_1(0)
                self.set_haptic_2(0)
                self.set_haptic_3(0)

            #Return audio
            print(self.location)


    def get_ultrasonic_distance(self):

        GPIO.output(self.GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        StartTime = time.time()
        StopTime = time.time()

        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            StartTime = time.time()

        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
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

        self.micro_servo.angle = degree


    def set_big_servo(self, degree):
        self.big_servo.angle = degree

    def set_haptic_1(self, velocity=DEFAULT_HAPTIC_VELOCITY):
        self.kit.motor1.throttle = velocity

    def set_haptic_2(self, velocity=DEFAULT_HAPTIC_VELOCITY):
        self.kit.motor2.throttle = velocity

    def set_haptic_3(self, velocity=DEFAULT_HAPTIC_VELOCITY):
        self.kit.motor3.throttle = velocity

    def set_audio(self, command):
        print("doesnt work yet")

    def haptic_handler(self):
        angle = self.location[0]
        distance = self.location[1]

        front = math.sin(angle) * distance
        side = math.cos(angle) * distance

        front_magnitude = (HAPTIC_DISTANCE_THRESHOLD - front)/HAPTIC_DISTANCE_THRESHOLD
        side_magnitude = (HAPTIC_DISTANCE_THRESHOLD - abs(side))/HAPTIC_DISTANCE_THRESHOLD

        try:
            self.set_haptic_2(front_magnitude) #Front Haptic

            if side < 0:
                self.set_haptic_3(side_magnitude)
            elif side > 0:
                self.set_haptic_1(side_magnitude)

        except:
            print("got magnitude bigger than 1")


    #---------------TESTS---------------
    def big_servo_test(self):

        servo = Servo(12)

        servo.source = sin_values()
        servo.source_delay = 0.03

        pause()

    def micro_servo_test(self):

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
            print("Measurement stopped")


if __name__ == "__main__":
    cane = Supercane()
    # cane.micro_servo_test()
    # cane.ultrasonic_sensor_test()




