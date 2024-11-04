from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    aebNode = Node(
        package='safety_node',
        executable='aeb',
        parameters=[
            {'TTCThreshold': 1.8}
        ]
    )

    ld.add_action(aebNode)

    return ld
    