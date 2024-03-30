from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """This function is used to launch the order management node and 
    subsequently start the ariac competition after 2 sec delay.

    Returns:
        LaunchDescription: The launch description of the nodes.
    """
    return LaunchDescription(
        [
            Node(
                package="rwa3_group1",
                executable="order_management.py",
                name="OrderManagement",
                output="screen",
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="rwa3_group1",
                        executable="start_ariac_competition",
                        name="start_ariac_competition",
                        output="screen",
                    ),
                ],
            ),
        ]
    )
