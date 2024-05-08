from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ariac_moveit_config.parameters import generate_parameters


def launch_setup(context, *args, **kwargs):
    moveit_cpp = Node(
        package="final_group1",
        executable="moveit_cpp_python",
        output="screen",
        parameters=generate_parameters(),
    )

    ariac_python_py =  Node(
            package='final_group1',
            executable='ariac_bonus_node.py',
            name='OrderManagement',
            output='screen'
            )

    nodes_to_start = [
        moveit_cpp,
        ariac_python_py,
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