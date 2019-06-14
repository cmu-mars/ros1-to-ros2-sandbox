"""Launch the ros2 talker and listener"""

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="test1_ros2",
            node_executable="talker_node",
            output="screen"),
        launch_ros.actions.Node(
            package="test1_ros2",
            node_executable="listener_node",
            output="screen"),
    ])
