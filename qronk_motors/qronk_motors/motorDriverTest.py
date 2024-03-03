#Modified from https://docs.circuitpython.org/projects/pca9685/en/latest/examples.html#servo-example
#2024

import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50
# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
sleepTime = 0.01
n = 12
servos = n*[None]
for i in range(n):
    servos[i] = servo.Servo(pca.channels[i],min_pulse = 500, max_pulse = 2500,actuation_range = 180)

# We sleep in the loops to give the servo time to move into position.
for i in range(180):
    for testServo in servos:
        testServo.angle = i
    time.sleep(sleepTime)

for i in range(180):
    for testServo in servos:
        testServo.angle = 180 - i
    time.sleep(sleepTime)

#Close the I2C connection
pca.deinit()