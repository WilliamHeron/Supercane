from gpiozero import Servo
from time import sleep
from gpiozero.tools import sin_values
from signal import pause

# from gpiozero.pins.pigpio import PiGPIOFactory
# pigpio_factory = PiGPIOFactory()
#
# servo = Servo(12, pin_factory=pigpio_factory)

# servo = Servo(12)
# servo.source = sin_values()
# servo.source_delay = 0.03
#
# pause()



#------Using GPIO LIBRARY-------

from gpiozero.pins.pigpio import PiGPIOFactory
pigpio_factory = PiGPIOFactory()

micro_servo = AngularServo(12, min_angle=-90, max_angle=90, pin_factory=pigpio_factory)

micro_servo.angle = 0
