from gpiozero import Servo
from time import sleep
from gpiozero.tools import sin_values
from signal import pause

servo = Servo(12, pin_factory=pigpio_factory)

servo.source = sin_values()
servo.source_delay = 0.03

pause()