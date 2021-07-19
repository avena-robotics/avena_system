import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SecurityTriggerPublisher(Node):

    def __init__(self):
        super().__init__('security_trigger')
        self.publisher_ = self.create_publisher(Bool, 'security_trigger', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Bool()
        if(self.i % 2):
            msg.data = True
        else:
            msg.data = False
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    security_trigger_publisher = SecurityTriggerPublisher()

    rclpy.spin(security_trigger_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
