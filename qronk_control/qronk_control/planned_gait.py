"""
planned_gait.py: This node will publish the joint states for the robot. The gait will be planned, with no feedback control. 
"""

import rclpy
from rclpy.node import Node

from qronk_interfaces.msg import JointVelocity

from qronk_control.gait_sequences import test_leg, angle_derivative


class plannedGait(Node):
    
    def __init__(self):
        
        super().__init__('plannedGait')
        
        self.declare_parameter('gait_type', 'testleg')

        self.publisher_ = self.create_publisher(JointVelocity, 'joint_velocity', 10)

        self.timer_frequency = 5  # Hz
        self.timer_period = 1.0 / self.timer_frequency
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.counter = 0
        
    def timer_callback(self):
        
        msg = JointVelocity()

        # Set the message to test_leg which moves the leg in a straight line
        try:
            msg.velocity = angle_derivative(test_leg(), self.timer_period)[self.counter]
        # Repeat the gait if the sequence reaches the end
        except IndexError:  
            self.counter = 0
            msg.velocity = angle_derivative(test_leg(), self.timer_period)[self.counter]
        self.publisher_.publish(msg)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    node = plannedGait()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
