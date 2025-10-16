import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Location


class CustomTalker(Node):

    def __init__(self):
        super().__init__('custom_talker')
        self.publisher_ = self.create_publisher(Location, 'custom_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = Location()
        msg.location.x = 1.0
        msg.location.y = 1.0
        msg.location.z = 0.0
        msg.name = "honolulu"

        self.get_logger().info(
            f'Publishing: ({msg.location.x:.2f}, {msg.location.y:.2f}, {msg.location.z:.2f}) / {msg.name}'
        )
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CustomTalker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()