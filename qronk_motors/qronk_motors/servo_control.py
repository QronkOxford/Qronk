#!/usr/bin/python
import RPi.GPIO as GPIO
import time

"""
servo_control.py
"""

class MG996R():
    '''
    Motor Class for MG996R motor. Each object maps to inidividual motor.
    ::params
    '''
    def __init__(self, id, pin) -> None:
        self.id = id
        self.pin = pin
    
    def set_angle() -> None:
        return None
    
class PWM():
    '''
    PWM signal class. Use for PWM duty cycle manipulation
    '''
    def __init__(self, pins = [int]) -> None:
        self.pins = pins
        self.signals = [None for entry in pins]
    
    def start_pwm(self, id, freq) -> id:
        '''
        Start one PWM signal at id.
        '''
        # Setup GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pins[id], GPIO.OUT)

        # Create GPIO.PWM object at given freq and store in self.signals
        self.signals[id] = GPIO.PWM(self.pins[id], freq)

        # Set initial duty cycle
        self.signals[id].ChangeDutyCycle(20)
    
    def change_duty_cycle(self, id, dc) -> None:
        '''
        Change duty cycle param of single GPIO.PWM object
        '''
        self.signals[id].ChangeDutyCycle(dc)
