#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Location
from custom_interfaces.srv import AuthenticateListener

class AdvancedTalker(Node):

    def __init__(self):
        super().__init__('adv_talker')
        self.password = 12345

        # Publisher
        self.publisher_ = self.create_publisher(Location, 'custom_topic_adv', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)

        # Service
        self.location_service_ = self.create_service(
            AuthenticateListener,
            '/adv_talker/share_secret_location',
            self.share_location_request
        )

        self.get_logger().info("AdvancedTalker node started.")

    def timer_callback(self):
        msg = Location()
        msg.location.x = 1.0
        msg.location.y = 1.0
        msg.location.z = 0.0
        msg.name = "honolulu"
        self.publisher_.publish(msg)
        self.get_logger().info("Published location: (1.0, 1.0, 0.0) / honolulu")

    def share_location_request(self, request, response):
        if request.password == self.password:
            response.secret_location.location.x = 20.0
            response.secret_location.location.y = 15.0
            response.secret_location.location.z = 5.0
            response.secret_location.name = "Haiti"
            self.get_logger().info("Authentication successful. Returning location.")
        else:
            response.secret_location.location.x = 0.0
            response.secret_location.location.y = 0.0
            response.secret_location.location.z = 0.0
            response.secret_location.name = "unauthorized"
            self.get_logger().warn(f"Authentication failed. Wrong password: {request.password}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AdvancedTalker node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()