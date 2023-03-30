from gpiozero import Servo
from time import sleep
from gpiozero.tools import sin_values
from signal import pause



from gpiozero import Servo
from time import sleep

servo = Servo(27)
val = -1

try:
    while True:
        servo.value = val*10
        sleep(0.1)
        val = val + 0.1
        if val > 1:
            val = -1
        print(val)
except KeyboardInterrupt:
    print("Program stopped")