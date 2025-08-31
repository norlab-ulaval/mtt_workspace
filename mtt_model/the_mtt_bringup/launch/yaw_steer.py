import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class JoystickToVelocityController(Node):

    def __init__(self):
        super().__init__('angular_velocity_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, '/yaw_controller/commands', 10)
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
       
        yaw = [-self.twist_message.angular.z] #Le négatif est pour inversé les axes 
        msg.data = yaw
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: %s' % str(self.twist_message.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = JoystickToVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()