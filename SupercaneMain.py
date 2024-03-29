

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
from adafruit_motorkit import MotorKit
import RPi.GPIO as GPIO

#Motor Control
from gpiozero.tools import sin_values
from gpiozero import Servo
from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory




# ----- Global Variables -----
ULTRASONIC_GPIO_TRIGGER = 17
ULTRASONIC_GPIO_ECHO = 22
MICRO_SERVO_PIN = 27
MICRO_SERVO_CENTER_POINT = 0
BIG_SERVO_PIN = 12
DEFAULT_HAPTIC_VELOCITY = 0.0
WHEEL_OBJECT_OFFSET_ANGLE = 45 #in degrees
WHEEL_DISTANCE_THRESHOLD = 150 #in cm
HAPTIC_DISTANCE_THRESHOLD = 70 #in cm
BUTTON_PIN = 18 #Button Pin
STAIR_ULTRASONIC_GPIO_TRIGGER = 21  #ultrasonic sensor for stairs
STAIR_ULTRASONIC_GPIO_ECHO = 20
STAIR_DISTANCE_THRESHOLD = 180 #Stair distance threshold

pigpio_factory = PiGPIOFactory()
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ------ Class ----------
class Supercane():

    def __init__(self):
        self.kit = MotorKit()   #HAT Controller Kit
        self.micro_servo = AngularServo(MICRO_SERVO_PIN, min_angle=-90, max_angle=90, pin_factory=pigpio_factory)
        self.big_servo = AngularServo(BIG_SERVO_PIN, min_angle=-90, max_angle=90, pin_factory=pigpio_factory)
        self.run = True
        self.set = 0
        self.big_servo_previous_val = 0
        self.stairs_found = False


        #Ultrasonic
        GPIO.setmode(GPIO.BCM)
        # set GPIO Pins
        self.GPIO_TRIGGER = ULTRASONIC_GPIO_TRIGGER
        self.GPIO_ECHO = ULTRASONIC_GPIO_ECHO
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

        #for stair ultrasonic sensor
        GPIO.setmode(GPIO.BCM)
        self.STAIR_GPIO_TRIGGER = STAIR_ULTRASONIC_GPIO_TRIGGER
        self.STAIR_GPIO_ECHO = STAIR_ULTRASONIC_GPIO_ECHO
        GPIO.setup(self.STAIR_GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.STAIR_GPIO_ECHO, GPIO.IN)



        self.location = [0,0] #Array to store [angle, distance]

        #Main loop of code
        # self.main()

        self.initial_state()


    def main(self):
        angle = 0
        angle_polarity = 0
        while self.run:
            sleep(0.1)
            temp_location = [0,0]

            #Check Button
            self.button_check()

            #Micro Servo
            ANG_UPPER_LIMIT = 140 + MICRO_SERVO_CENTER_POINT
            ANG_LOWER_LIMIT = 40 + MICRO_SERVO_CENTER_POINT
            INCRIMENT_BY = 20

            if angle_polarity == 0:
                angle += INCRIMENT_BY
                if angle >= ANG_UPPER_LIMIT:
                    angle_polarity = 1
            else:
                angle -= INCRIMENT_BY
                if angle <= ANG_LOWER_LIMIT:
                    angle_polarity = 0

            ang = angle - 90
            try:
                self.set_micro_servo(ang)
            except ValueError:
                print("couldn't set servo angle")
                pass

            # print(ang)
            temp_location[0] = ang

            #Read Distance
            try:
                distance_ultra = self.get_ultrasonic_distance()

            except ValueError:
                distance_ultra = 300
                print("couldn't read ultrasonic distance")
                pass

            #Check for stairs
            try:
                self.stair_check()
            except:
                print("stair check failed")
                pass


            # distance_camera = self.get_camera_data()

            #Assess distance/angle
            temp_location[1] = distance_ultra
            self.location = self.update_location(temp_location)


            #Wheel feedback
            if self.location[1] < WHEEL_DISTANCE_THRESHOLD and self.stairs_found == False:
                self.wheel_handler()
            else:
                servo_val = self.big_servo_previous_val / 2 #brings the servo back to 0 slowly
                self.set_big_servo(servo_val)


            #Haptic Feedback
            if self.stairs_found == True:
                try:
                    self.set_haptic_1(1)
                    self.set_haptic_2(1)
                    self.set_haptic_3(1)

                except ValueError:
                    print("haptic did not worked")
                    pass

            elif self.location[1] < HAPTIC_DISTANCE_THRESHOLD:
                self.haptic_handler() #Handles haptic feedback proportions

            else:
                try:
                    self.set_haptic_1(0)
                    self.set_haptic_2(0)
                    self.set_haptic_3(0)
                except ValueError:
                    print("Couldn't reset haptic feedback")
                    pass

            print(self.location)

        #RESET PI
        self.reset()
        sleep(2)
        self.initial_state()    #Return to initial state

            #Return audio


    def initial_state(self):
        button_status = True

        while button_status:
            input_state = GPIO.input(BUTTON_PIN)
            if input_state == False and self.set == 0:

                print("set = 0")
                time.sleep(1)
                self.set = 1
                self.run = True
                button_status = False

        self.main()


    def button_check(self):

        input_state = GPIO.input(BUTTON_PIN)
        if input_state == False and self.set == 1:
            print("set = 1")
            time.sleep(1)
            self.set = 0
            self.run = False


    def reset(self):
        self.set_haptic_1(0)
        self.set_haptic_2(0)
        self.set_haptic_3(0)
        self.set_big_servo(0)
        self.set_micro_servo(0)
        self.micro_servo.detach()
        self.big_servo.detach()


    def stop(self):
        self.run = False


    def start(self):
        self.run = True
        self.main()


    def stair_check(self):
        try:
            stair_dist = self.get_stair_ultrasonic_distance()
            if stair_dist > STAIR_DISTANCE_THRESHOLD:
                self.stairs_found = True
            else:
                self.stairs_found = False

            print(stair_dist)
        except:
            print("stair distance not found")
            pass



    def get_stair_ultrasonic_distance(self):

        GPIO.output(self.STAIR_GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.STAIR_GPIO_TRIGGER, False)
        StartTime = time.time()
        StopTime = time.time()

        try:
            # save StartTime
            while GPIO.input(self.STAIR_GPIO_ECHO) == 0:
                StartTime = time.time()

            # save time of arrival
            while GPIO.input(self.STAIR_GPIO_ECHO) == 1:
                StopTime = time.time()

        except ValueError:
            print("Ultrasonic Reading didn't work inside 'get_ultrasonic_distance()''")
            StartTime = 0
            StopTime = 1
            pass

        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        distance = round(distance, 3)

        return distance


    def get_ultrasonic_distance(self):

        GPIO.output(self.GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        StartTime = time.time()
        StopTime = time.time()

        try:
            # save StartTime
            while GPIO.input(self.GPIO_ECHO) == 0:
                StartTime = time.time()

            # save time of arrival
            while GPIO.input(self.GPIO_ECHO) == 1:
                StopTime = time.time()

        except ValueError:
            print("Ultrasonic Reading didn't work inside 'get_ultrasonic_distance()''")
            StartTime = 0
            StopTime = 1
            pass

        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
        distance = round(distance, 3)

        if distance > 400:
            distance = 400

        return distance


    def get_camera_data(self):

        return None

    def get_GPS_data(self):

        return None

    def set_micro_servo(self, degree):

        self.micro_servo.angle = degree

    def set_big_servo(self, degree):
        if degree > 90:
            degree = 90
        elif degree < -90:
            degree = -90

        degree = -degree
        self.big_servo.angle = degree

    def set_haptic_1(self, velocity=DEFAULT_HAPTIC_VELOCITY):
        self.kit.motor1.throttle = velocity

    def set_haptic_2(self, velocity=DEFAULT_HAPTIC_VELOCITY):
        self.kit.motor2.throttle = velocity

    def set_haptic_3(self, velocity=DEFAULT_HAPTIC_VELOCITY):
        self.kit.motor3.throttle = velocity

    def set_audio(self, command):
        print("doesnt work yet")

    def update_location(self, temp_location):

        if self.location == [0,0]: #Initial State
            return temp_location

        elif temp_location == [0,0]: #Handle if sensor data fails
            return self.location

        elif temp_location[1] < self.location[1]: #if new distance is less than old distance
            return temp_location

        elif temp_location[0] == self.location[0] and temp_location[1] > self.location[1]: #if the angle is the same and the new distance is further than the old distance
            return temp_location

        else:
            return self.location


    def haptic_handler(self):
        angle = self.location[0]
        distance = self.location[1]


        front = math.sin(angle) * distance
        side = math.cos(angle) * distance

        front_magnitude = (HAPTIC_DISTANCE_THRESHOLD - front)/HAPTIC_DISTANCE_THRESHOLD
        side_magnitude = (HAPTIC_DISTANCE_THRESHOLD - abs(side))/HAPTIC_DISTANCE_THRESHOLD
        front_magnitude = round(front_magnitude, 2)
        side_magnitude = round(side_magnitude, 2)


        try:

            self.set_haptic_2(front_magnitude) #Front Haptic

            if side < 0:
                self.set_haptic_3(side_magnitude)
            elif side > 0:
                self.set_haptic_1(side_magnitude)

        except ValueError:
            print("invalid haptic value: " + str(side_magnitude))
            pass

    def wheel_handler(self):
        angle = self.location[0]
        distance = self.location[1]

        percent_reduction = 1.42 #reducing the dramatic turn of the wheel
        big_servo_angle = 0

        #Formula says that:     degree response from the wheel = servo_angle - 90degrees * (threshold-distance)/threshold * 0.90
        if angle >= 0:
            big_servo_angle = angle - 90*(WHEEL_DISTANCE_THRESHOLD - abs(distance))/WHEEL_DISTANCE_THRESHOLD * percent_reduction

        elif angle < 0:
            big_servo_angle = angle + 90 * (WHEEL_DISTANCE_THRESHOLD - abs(distance)) / WHEEL_DISTANCE_THRESHOLD * percent_reduction

        # print("wheel angle: " + str(big_servo_angle))
        try:
            # print("big servo angle: " + str(big_servo_angle))
            # big_servo_angle = 0
            self.set_big_servo(big_servo_angle)
            self.big_servo_previous_val = big_servo_angle
        except ValueError:
            print("servo value not valid : " + str(big_servo_angle))
            pass



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





