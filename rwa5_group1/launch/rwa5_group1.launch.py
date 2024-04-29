import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction,DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from ariac_moveit_config.parameters import generate_parameters


def launch_setup(context, *args, **kwargs):
    moveit_cpp = Node(
        package="rwa5_group1",
        executable="moveit_cpp_python",
        output="screen",
        parameters=generate_parameters(),
    )
    ariac_start_cpp = TimerAction(
            period=5.0,
            actions=[
                Node(
                package='rwa5_group1',
                executable='start_ariac_competition',
                name='start_ariac_competition',
                output='screen',
            ),
            ]
        )
    ariac_python_py =  Node(
            package='rwa5_group1',
            executable='ariac_python_node.py',
            name='OrderManagement',
            output='screen'
            )
    start_rviz = LaunchConfiguration("rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rwa5_group1"), "rviz", "moveit_demo.rviz"]
    )
    
    moveit_python = Node(
        package="rwa5_group1",
        executable="floor_robot_demo.py",
        output="screen"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=generate_parameters(),
        condition=IfCondition(start_rviz),
    )
    

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ariac_moveit_config"),
                "/launch",
                "/ariac_robots_moveit.launch.py",
            ]
        )
    )

    nodes_to_start = [
        moveit_cpp,
        ariac_start_cpp,
        ariac_python_py
        # moveit_python
        # rviz_node,
        # moveit
    ]

    return nodes_to_start
def generate_launch_description():    
    """This function is used to launch the thermostat_house node and set the parameters for the node.

    Returns:
        LaunchDescription: The launch description of the nodes.
    """
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="false", description="start rviz node?"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )