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
pca.frequency = 100
# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
tester = servo.Servo(pca.channels[0],min_pulse = 500, max_pulse = 2500,actuation_range=270)
tester.angle = 0
time.sleep(2)
#Test speed
"""for i in range(10):
    print(i)
    tester.angle = 270
    time.sleep(5)
    tester.angle = 0
    time.sleep(5)"""
#Test angle
tester.angle = 0
time.sleep(10)
for i in range(270):
    tester.angle= i+1
    time.sleep(0.02)
time.sleep(3)
#Close the I2C connection
pca.deinit()