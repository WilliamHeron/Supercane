#Script for running main script
import signal

from SupercaneMain import *
from RESET import *
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
import signal
import threading
import logging
import sys

BUTTON_PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pressed = False

lock = None



# class RunPython(threading.Thread):
#     def __init__(self):
#         threading.Thread.__init__(self)
#         self.set = 0
#         self.cane = Supercane()
#         # self.process = None
#         # self._lock = toggle_lock
#
    # def run(self):
    #     self.cane.start()
    #
    #     while True:
    #         input_state = GPIO.input(BUTTON_PIN)
    #         if input_state == False and self.set == 0:
    #             self.cane.main()
    #             print("set = 0")
    #             time.sleep(1)
    #             self.set = 1
    #             pressed = True
    #
    #         Input_state = GPIO.input(BUTTON_PIN)
    #
    #         if input_state == False and self.set == 1:
    #             self.cane.stop()
    #
    #             print("set = 1")
    #             time.sleep(1)
    #             self.set = 0



        # self.process = subprocess.run(["python3", "SupercaneMain.py"], capture_output=True, text=True)
        # self.process = subprocess.Popen(["python3", "SupercaneMain.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


    # def close(self):
    #     # output, errors = self.process.communicate()
    #     # self.process.kill()
    #
    #     # subprocess.run(["^c"], capture_output=True, text=True)
    #
    #     # for message in self._args:
    #     #     if self._lock:
    #     #         self._lock.acquire()
    #     #     print(message)
    #     #     if self._lock:
    #     #         self._lock.release()
    #     #     time.sleep(self._kwargs.get("delay", 1.0))

# # mp1 = MessagePrinter("Hello", "Good day!", lock=lock)
# # mp2 = MessagePrinter("A", "B", "C", delay=3, lock=lock)
# # mp1.start()
# # mp2.start()
# # mp1.join()
# # mp2.join()






# # py_script = RunPython()
# cane = None
# first_run = False
#

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

        # py_script.start()
        # if first_run == False:
        #     cane = Supercane()
        #     cane.start()
        #     cane.main()
        #     first_run = True
        # else:
        #     cane.start()
        # cmd = ['python3 SupercaneMain.py']
        # cmd = ["python3", "SupercaneMain.py"]
        cmd = [sys.executable, "SupercaneMain.py"]

        # pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        pro = subprocess.Popen("SupercaneMain.py", shell=True)
        pro.communicate()
        print("set = 0")

        time.sleep(1)
        set = 1


    Input_state = GPIO.input(BUTTON_PIN)
    if input_state == False and set == 1:
        # cane.stop()
        # cane.join()

        os.killpg(os.getpgid(pro.pid), signal.SIGTERM)

        # cane = Supercane()
        # py_script.setDaemo(False)
        # py_script.close()

        # result = subprocess.run(["python3", "RESET.py"], capture_output=True, text=True)
        # sleep(0.5)
        # os.killpg(result.pid, signal.SIGTERM)

        # print(result.stdout)


        print("set = 1")
        time.sleep(1)
        set = 0



#


