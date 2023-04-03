#Script for running main script
import signal

from SupercaneMain import *

#Coppied from SupercaneMain

# from time import sleep
# import math
# import time
# from signal import pause
#
# #Board
# import board
# from adafruit_motorkit import MotorKit
# import RPi.GPIO as GPIO
#
# #Motor Control
# from gpiozero.tools import sin_values
# from gpiozero import Servo
# from gpiozero import AngularServo
# from gpiozero.pins.pigpio import PiGPIOFactory
# pigpio_factory = PiGPIOFactory()




import RPi.GPIO as GPIO
import time
import subprocess
import os

BUTTON_PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
set = 0
while True:
    input_state = GPIO.input(BUTTON_PIN)
    if input_state == False and set == 0:
        # subprocess.("/home/pi/securipi-rpicamtd.sh", shell=True)
        # p=subprocess.Popen( "/home/pi/securipi-picamtd.sh",shell=True,preexec_fn=os.setsid)
        # p=subprocess.Popen( "/home/pi/Supercane/SupercaneMain.py",shell=True,preexec_fn=os.setsid)
        # subprocess.call(['cd Supercane'])
        # subprocess.call(['python3 SupercaneMain.py'])
        # subprocess.run(["python", "SupercaneMain.py"])
        cane = Supercane()
        print("set = 0")
        time.sleep(1)
        set = 1

    Input_state = GPIO.input(BUTTON_PIN)
    if input_state == False and set == 1:
        # cane = Supercane()
        # os.killpg(p.pid, signal.SIGTERM)

        print("set = 1")
        time.sleep(1)
        set = 0



