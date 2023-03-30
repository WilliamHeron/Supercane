from gpiozero import Servo
from time import sleep
from gpiozero.tools import sin_values
from signal import pause



from gpiozero import Servo
from time import sleep

servo = Servo(27)

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