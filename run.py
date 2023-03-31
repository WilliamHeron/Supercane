#Script for running main script

from SupercaneMain import *


import RPi.GPIO as GPIO
import time
import subprocess

BUTTON_PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO, pull_up_down=GPIO.PUD_UP)
set = 0
while True:
   input_state = GPIO.input(BUTTON_PIN)
    if input_state == False and set == 0:
        # subprocess.("/home/pi/securipi-rpicamtd.sh", shell=True)
        # p=subprocess.Popen( "/home/pi/securipi-picamtd.sh",shell=True,preexec_fn=os.setsid)

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



