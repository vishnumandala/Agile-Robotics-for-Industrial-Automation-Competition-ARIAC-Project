#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rwa4_group1.order_management_interface_update import OrderManagement


def main(args=None):
    rclpy.init(args=args)
    node = OrderManagement("order_management_upate")
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()