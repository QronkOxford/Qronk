import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class InjectTest(Node):

    def __init__(self):
        super().__init__('joint_state_test_publisher')
        self.publisher_ = self.create_publisher(JointState, 'topic')
        timer_period = 0.02  # Publish every 20ms
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.name = [0, 1, 2]
        msg.velocity = [50, 50, 50] # [rpm]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = InjectTest()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
