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
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10) #Sub to the joint state message
        self.subscription  # prevent unused variable warning
        i2c = busio.I2C(SCL, SDA) #connect to driver board
        pca = PCA9685(i2c) 
        pca.frequency = 50
        self.servos = [servo.Servo(i) for i in range(1,nServos)] #Initialize servos

    def listener_callback(self, msg):
        self.pos = msg.position #Store names and positions
        self.name = msg.name

    def positionServo(self,nServos):
        for i in range(nServos):
            p = self.pos[i]
            s = self.servos[i]
            if p > pi: #Maximum value of 180 deg - Need to tune this
                s.angle = 1
            elif p < 0: #Minimum of 0 deg - Need to tune this
                s.angle = -1
            else:
                s.angle = p*(180/pi) #Set servo angle for all values inbetween

def main(args=None):
    n = 3 #Number of servos, Don't overload the board just yet!
    rclpy.init(args=args)
    minimal_subscriber = JointSub(n)
    while rclpy.ok():
        #Get values over and over
        rclpy.spin_once(minimal_subscriber)
        minimal_subscriber.positionServo(n) #Take revised values and send to servos
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()