#2024
#Subscribe to the JointState message in ROS2 and pass it to a PCA9685 driver board, currently takes an
#angle and passes it on. Need to make an equivalent for angular velocity for continuous rotation servos.
#Only runs on raspberry pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from math import pi

class JointSub(Node):

    def __init__(self,nServos):
        super().__init__('JointSub')
        
        # Subscribe to joint_states topic
        self.create_subscription(JointState,'joint_states',self.listener_callback,10)
        
        _i2c = busio.I2C(SCL, SDA) # Connect to i2c bus
        self.pca = PCA9685(_i2c) # Create PCA9685 instance

        self.pca.frequency = 50 # Set PCA frequncy to 50Hz
        self.topic_freq = 30 # Hz
        self.max_speed = 50 * 2 * pi # Angular vel. when servo is fully throttled (~ 50rpm, should be tested)
        
        # Initialize continuous servos
        self.servos = [servo.ContinuousServo(self.pca.channels[i],
                                             min_pulse = 1300, # 
                                             max_pulse = 1700) for i in range(nServos)]

    def listener_callback(self, msg):
        # Unpack msg data
        _timestamp = msg.header # Note header contains timestamp when joint angle was calculated
        _names = msg.name # TODO: Agree on servo naming convention (I think simple 1, 2,3 or servo1, servo2, servo3 would be good)
        _positions = msg.position
        _vels = msg.velocity # TODO: Agree on units
        _efforts = msg.effort

        self.pos_old = self.pos # Store previous timestep position
        self.pos = msg.pos # Store new timestep position
        self.name = msg.name # Store name

    def throttleServo(self, servoID: int = 0):
        # TODO: Agree on topic puiblishing frequency and make sure publishers stick to it
        _speed_ratio = 100 / self.max_speed # TODO: 100 here is placeholder
        self.servos[servoID].throttle(_speed_ratio)
        pass

    def throttleServos(self, servos):
        pass

    def positionServo(self,nServos):
        pass
        '''
        for i in range(nServos):
            p = self.pos[i]
            s = self.servos[i]
            if p > pi: #Maximum value of 180 deg - Need to tune this
                s.angle = 1
            elif p < 0: #Minimum of 0 deg - Need to tune this
                s.angle = -1
            else:
                s.angle = p*(180/pi) #Set servo angle for all values inbetween
        '''
    
    def closeBus(self): #Close the bus when finished
        self.pca.deinit()

def main(args=None):
    n = 3 #Number of servos, Don't overload the board just yet!
    rclpy.init(args=args)
    minimal_subscriber = JointSub(n)
    while rclpy.ok():
        #Get values over and over
        rclpy.spin_once(minimal_subscriber)
        minimal_subscriber.positionServo(n) #Take revised values and send to servos
    rclpy.spin_once(minimal_subscriber) #Close the bus
    minimal_subscriber.closeBus()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()