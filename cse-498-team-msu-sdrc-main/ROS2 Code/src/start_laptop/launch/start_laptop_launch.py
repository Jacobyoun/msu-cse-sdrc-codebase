from launch import LaunchDescription
from launch_ros.actions import Node

# This can be easily exteneded in the future. Just follow the pattern below to add another command.

def generate_launch_description():
    """Function to start all of the rosnode on the laptop"""
    
    return LaunchDescription([
        # Start command for the loopback node
        Node(
            package="loopback_test",
            executable="loopback_test_laptop",
            name="loopback_test",
            output="screen"
        ),
        # Start command for the controller publisher
        Node(
            package="controller_publisher",
            executable="control_publish",
            name="control_receive",
            output="screen"
        ),
    ])