"""
planned_gait.py: This node will publish the joint states for the robot. The gait will be planned, with no feedback control. 
"""

import rclpy
from rclpy.node import Node

from qronk_interfaces.msg import JointVelocity
from sensor_msgs.msg import JointState

from qronk_control.gait_sequences import test_leg, angle_derivative


class plannedGait(Node):
    
    def __init__(self):
        
        super().__init__('plannedGait')
        
        self.declare_parameter('gait_type', 'testleg')

        self.publisher_ = self.create_publisher(JointVelocity, 'joint_velocity', 10)
        self.publisher2_ = self.create_publisher(JointState, 'joint_states', 10)  # ! for testing purposes

        self.timer_frequency = 5  # Hz
        self.timer_period = 1.0 / self.timer_frequency
        self.counter = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback2)

        
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

    # ! for testing purposes before rollback to position based servo
    def timer_callback2(self):

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'chassis_shoulder_FR_joint',
            'shoulder_upper_leg_FR_joint',
            'upper_leg_lower_leg_FR_joint',
            'chassis_shoulder_FL_joint',
            'shoulder_upper_leg_FL_joint',
            'upper_leg_lower_leg_FL_joint',
            'chassis_shoulder_BR_joint',
            'shoulder_upper_leg_BR_joint',
            'upper_leg_lower_leg_BR_joint',
            'chassis_shoulder_BL_joint',
            'shoulder_upper_leg_BL_joint',
            'upper_leg_lower_leg_BL_joint'
        ]

        try:
            msg.position = test_leg()[self.counter]
        except IndexError:
            self.counter = 0
            msg.position = test_leg()[self.counter]
        self.publisher2_.publish(msg)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    node = plannedGait()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
