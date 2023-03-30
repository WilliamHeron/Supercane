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


from gpiozero import Servo
from time import sleep

servo = Servo(12)

try:
    while True:
        servo.min()
        sleep(0.5)
        servo.mid()
        sleep(0.5)
        servo.max()
        sleep(0.5)

except KeyboardInterrupt:
    print("Program stopped")