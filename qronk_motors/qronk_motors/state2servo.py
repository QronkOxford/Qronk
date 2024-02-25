#2024
#Subscribe to the JointState message in ROS2 and pass it to a PCA9685 driver board, currently takes an
#angle and passes it on. Need to make an equivalent for angular velocity for continuous rotation servos.
#Only runs on raspberry pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
#from qronk_interfaces.msg import JointVelocity
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
        self.max_speed = 50 # Angular vel
        
        # Initialize continuous servos
        self.servos = [servo.Servo(self.pca.channels[i],
                                             min_pulse = 500, # 
                                             max_pulse = 2500, 
                                             actuation_range = 180) for i in range(nServos)]

    def listener_callback(self, msg):
        # Unpack msg data
        _vels = msg.velocity # Angular [rpm]
        _speed_ratios = [vel / self.max_speed for vel in _vels]
        _pos = msg.position # Angles [rad]

        self.positionServo(_pos)

    def throttleServo(self, speed_ratio, servoID: int = 0):
        #_speed_ratio = self.vel / self.max_speed
        self.servos[servoID].throttle = speed_ratio
        pass

    def throttleServos(self, speed_ratios):
        for i in range(len(self.vels)):  
            # Calculate speed ratio
            _speed_ratio = speed_ratios

            # Keep within limits
            if _speed_ratio > 1:
                _speed_ratio = 1
            elif _speed_ratio < -1:
                _speed_ratio = -1

            # Send to servo
            self.throttleServo(_speed_ratio, i)

    def positionServo(self,pos):
        for i in range(len(self.servos)):
            p = pos[i]
            s = self.servos[i]
            if p > pi: #Maximum value of 180 deg - Need to tune this
                s.angle = 1
            elif p < 0: #Minimum of 0 deg - Need to tune this
                s.angle = -1
            else:
                s.angle = p*(180/pi) #Set servo angle for all values inbetween
   
    def closeBus(self,servos): #Close the bus when finished
        speeds = servos*[0] #Stop servos
        for i in range(len(speeds)):
            self.throttleServo(speeds[i],i)
        self.pca.deinit()

def main(args=None):
    n = 3 #Number of servos, Don't overload the board just yet!
    rclpy.init(args=args)
    minimal_subscriber = JointSub(n)
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.closeBus(n)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()