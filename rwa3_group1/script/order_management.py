#!/usr/bin/env python3

import rclpy
from rwa3_group1.order_management_interface import OrderManagement


def main(args=None):
    rclpy.init(args=args)
    node = OrderManagement("order")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()  # Execute the main function when the script is run