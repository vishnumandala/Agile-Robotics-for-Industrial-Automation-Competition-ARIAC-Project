from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction,DeclareLaunchArgument, OpaqueFunction
from ariac_moveit_config.parameters import generate_parameters


def launch_setup(context, *args, **kwargs):
    """Launch ariac node, moveit_cpp node and start ariac competition node after 10 seconds

    Args:
        context (any): context
        *args: args
        **kwargs: kwargs

    Returns:
        List: list of nodes to start
    """
    moveit_cpp = Node(
        package="final_group1",
        executable="moveit_cpp_python",
        output="screen",
        parameters=generate_parameters(),
    )
    ariac_start_cpp = TimerAction(
            period=10.0,
            actions=[
                Node(
                package='final_group1',
                executable='start_ariac_competition',
                name='start_ariac_competition',
                output='screen',
            ),
            ]
        )
    ariac_python_py =  Node(
            package='final_group1',
            executable='ariac_python_node.py',
            name='OrderManagement',
            output='screen'
            )

    nodes_to_start = [
        moveit_cpp,
        ariac_python_py,
        ariac_start_cpp,
    ]

    return nodes_to_start
def generate_launch_description():    
    """Launch ariac node, moveit_cpp node and start ariac competition node after 10 seconds

    Returns:
        LaunchDescription: return launch description
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