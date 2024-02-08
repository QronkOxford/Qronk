"""
walk_planned.py: This node will publish the joint states for the robot. The gait will be planned, with no feedback control. 
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from random import random  # ! for testing, delete once done. 


class plannedGait(Node):
    
    def __init__(self):
        
        super().__init__('plannedGait')
        
        self.declare_parameter('gait_type', 'crawl')

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        timer_frequency = 5  # Hz
        timer_period = 1.0 / timer_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        
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

        # * We will have to publish the position or velocity of each joints in the gait sequence. 
        msg.position = [
            0.0,
            3*random(),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]
        msg.velocity = []
        msg.effort = []

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = plannedGait()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# ? Notes for reference during development. 
    
# ? The JointState message type is defined as follows: 
""" 
> ros2 interface show sensor_msgs/msg/JointState 

>
# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

string[] name
float64[] position
float64[] velocity
float64[] effort
"""
