#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import webbrowser


class URLLauncher(Node):
    def __init__(self):
        super().__init__("url_launcher_node")

        self.subscription = self.create_subscription(
            String,
            '/qr_code/data',
            self.callback,
            10
        )

        self.get_logger().info("URL Launcher Node Started! Waiting for QR data...")

    def callback(self, msg):
        url = msg.data.strip()

        if url.startswith("http://") or url.startswith("https://"):
            self.get_logger().info(f"Opening URL: {url}")
            webbrowser.open(url)
        else:
            self.get_logger().warn(f"Received data but it's not a valid URL: {url}")


def main(args=None):
    rclpy.init(args=args)
    node = URLLauncher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
