import time
from machine import Pin, PWM

pwm = PWM(Pin(15))
pwm.freq(50)

# Define function to set position
def setServoAngle(angle):
    position = int(8000*(angle/180) + 1000)     # Convert angle into [1000, 9000] range
    pwm.duty_u16(position)