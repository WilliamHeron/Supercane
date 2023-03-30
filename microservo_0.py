from gpiozero import Servo
from time import sleep
from gpiozero.tools import sin_values
from signal import pause



from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
pigpio_factory = PiGPIOFactory()

# servo = Servo(27, pin_factory=pigpio_factory)
micro_servo = AngularServo(27, min_angle=-90, max_angle=90, pin_factory=pigpio_factory)

sleep(1)
micro_servo.angle = 0
sleep(1)