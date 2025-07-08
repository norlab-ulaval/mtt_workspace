import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class JoystickToGroupController(Node):

    def __init__(self):
        super().__init__('linear_velocity_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/wheel_group_controller/commands', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.publish_message)  # Publish every second
        self.twist_message = Twist()

    def listener_callback(self, msg: Twist):

        self.twist_message = msg

    def publish_message(self):
        msg = Float64MultiArray()
       
        wheel_speeds = [-self.twist_message.linear.x] * 20 #le negatif est pour inversé les axes pour avoir la bonne vitesse
        msg.data = wheel_speeds
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: %s' % str(self.twist_message.linear.x))


def main(args=None):
    rclpy.init(args=args)
    node = JoystickToGroupController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()