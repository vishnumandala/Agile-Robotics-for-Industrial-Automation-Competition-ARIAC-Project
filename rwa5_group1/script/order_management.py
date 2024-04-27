#!/usr/bin/env python3.8
"""
File: order_management.py
Author: Ankur Mahesh Chavan (achavan1@umd.edu),Datta Lohith Gannavarapu (gdatta@umd.edu),
Shail Kiritkumar Shah (sshah115@umd.edu) Vinay Krishna Bukka (vinay06@umd.edu),
Vishnu Mandala (vishnum@umd.edu)
Date: 03/28/2024
Description: main function to spin the node.
"""

import rclpy
from rclpy.executors import MultiThreadedExecutor
# from rwa5_group1.order_management_interface import OrderManagement
from rwa5_group1.order_management_interface_alc import OrderManagement
def main(args=None):
    rclpy.init(args=args)
    node = OrderManagement("order_management")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
