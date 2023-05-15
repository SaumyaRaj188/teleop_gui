import rclpy
from rclpy.node import Node
import geometry_msgs.msg


class TwistPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 1.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.publisher_.publish(twist)
        

def main(args=None):
    rclpy.init(args=args)

    twist_publisher = TwistPublisher()

    rclpy.spin(twist_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()