from gpiozero import Servo
from time import sleep
from gpiozero.tools import sin_values
from signal import pause



from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
pigpio_factory = PiGPIOFactory()

# servo = Servo(27, pin_factory=pigpio_factory)
micro_servo = AngularServo(27, min_angle=-90, max_angle=90, pin_factory=pigpio_factory)

angle = 0
angle_polarity = 0

while True:
    sleep(1)

    #Micro Servo
    ANG_UPPER_LIMIT = 140
    ANG_LOWER_LIMIT = 60
    INCRIMENT_BY = 10

    if angle_polarity == 0:
        angle += INCRIMENT_BY
        if angle >= ANG_UPPER_LIMIT:
            angle_polarity = 1
    else:
        angle -= INCRIMENT_BY
        if angle <= ANG_LOWER_LIMIT:
            angle_polarity = 0

    ang = angle - 90
    print(ang)
    micro_servo.angle = ang