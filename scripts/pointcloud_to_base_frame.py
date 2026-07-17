import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs import do_transform_cloud
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


DEFAULT_NODE_NAME = 'pc_to_base'
DEFAULT_INPUT_TOPIC = '/hesai_lidar/points'
DEFAULT_OUTPUT_TOPIC = '/kiss_icp/input_points'


class PointCloudToBaseFrame(Node):
    def __init__(self, node_name=DEFAULT_NODE_NAME,
                 input_topic=DEFAULT_INPUT_TOPIC,
                 output_topic=DEFAULT_OUTPUT_TOPIC):
        super().__init__(node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.cb, 10)
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(
            PointCloud2, output_topic, qos)
        self.get_logger().info(
            f'Node started: {input_topic} → {output_topic}')

    def cb(self, msg):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_footprint', msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.5))

            p = do_transform_cloud(msg, t)
            p.header.frame_id = 'base_footprint'
            self.pub.publish(p)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node_name = DEFAULT_NODE_NAME
    input_topic = DEFAULT_INPUT_TOPIC
    output_topic = DEFAULT_OUTPUT_TOPIC
    if args is None:
        args = sys.argv[1:]
    for i, a in enumerate(args):
        if a == '--node-name' and i + 1 < len(args):
            node_name = args[i + 1]
        elif a == '--input-topic' and i + 1 < len(args):
            input_topic = args[i + 1]
        elif a == '--output-topic' and i + 1 < len(args):
            output_topic = args[i + 1]
    node = PointCloudToBaseFrame(node_name, input_topic, output_topic)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
