import serial
from Protocol import *

s = serial.Serial('COM3', 250000)
servo = Servo(s, 1, 1)

time.sleep(4)

while True:
    servo.getPosition()
