import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Location


class CustomListener(Node):

    def __init__(self):
        super().__init__('custom_listener')
        self.subscription = self.create_subscription(
            Location,
            'custom_topic',
            self.topic_callback,
            10)
        self.subscription  # prevent unused variable warning

    def topic_callback(self, msg):
        self.get_logger().info(
            f'I heard: ({msg.location.x:.2f}, {msg.location.y:.2f}, {msg.location.z:.2f}) / {msg.name}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CustomListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()