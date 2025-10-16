#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Location
from custom_interfaces.srv import AuthenticateListener

class AdvancedListener(Node):
    def __init__(self):
        super().__init__('adv_listener')

        # Subscriber
        self.subscription_ = self.create_subscription(
            Location,
            'custom_topic_adv',
            self.topic_callback,
            10
        )
        self.subscription_  # prevent unused variable warning

        # Publisher
        self.publisher_ = self.create_publisher(Location, 'share_secret', 10)

        # Client
        self.client_ = self.create_client(AuthenticateListener, '/adv_talker/share_secret_location')

        # Parameter
        self.declare_parameter('share_parameter', True)

        # Timer (2 seconds)
        self.timer_ = self.create_timer(2.0, self.call_location_service)

        self.get_logger().info("AdvancedListener node started.")

    def topic_callback(self, msg):
        self.get_logger().info(
            f"I heard: ({msg.location.x:.2f}, {msg.location.y:.2f}, {msg.location.z:.2f})/{msg.name}"
        )

    def call_location_service(self):
        if not self.client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Service not available...")
            return

        request = AuthenticateListener.Request()
        request.password = 12345

        future = self.client_.call_async(request)
        future.add_done_callback(self.handle_location_response)

    def handle_location_response(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        loc = response.secret_location
        self.get_logger().info(
            f"Authenticated location: ({loc.location.x:.2f}, {loc.location.y:.2f}, {loc.location.z:.2f})/{loc.name}"
        )

        # Publish secret if parameter is True
        if self.get_parameter('share_parameter').get_parameter_value().bool_value:
            self.publisher_.publish(loc)
            self.get_logger().info("Published secret location to /share_secret")

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AdvancedListener node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()