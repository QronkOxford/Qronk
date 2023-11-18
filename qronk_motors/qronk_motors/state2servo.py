#Make sure to <pip3 install tkgpio> for virtual servos
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from qronk_motors.setupVirtualServos import run
from gpiozero import Servo

class JointSub(Node):

    def __init__(self):
        super().__init__('JointSub')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10) #Sub to the joint state message
        self.subscription  # prevent unused variable warning
        self.servos = [Servo(i) for i in range(1,4)] #Initialize servos

    def listener_callback(self, msg):
        self.pos = msg.position #Store names and positions
        self.name = msg.name

    def positionServo(self):
        for i in range(3): #Just one leg at present
            p = self.pos[i]
            s = self.servos[i]
            if p > 1.047: #Maximum value of 60deg (from datasheet)
                s.value = 1
            elif p < -1.047: #Minimum of -60deg
                s.value = -1
            else:
                s.value = p*(1/1.047) #Set servo value for all values inbetween

@run #Start the virtual servo simulation
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = JointSub()
    while rclpy.ok():
        #Get values over and over
        rclpy.spin_once(minimal_subscriber)
        minimal_subscriber.positionServo() #Take revieved values and send to servos
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()