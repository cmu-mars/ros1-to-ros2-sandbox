"""Launch the ros2 talker and listener"""
import launch
#from launch import LaunchDescription
import launch_ros.actions
import launch.event_handlers

def generate_launch_description():
    talker = launch_ros.actions.Node(
        package="test1_ros2",
        node_executable="talker_node",
        output="screen")
    listener = launch_ros.actions.Node(
        package="test1_ros2",
        node_executable="listener_node",
        output="screen")
    return launch.LaunchDescription([
        talker,
        listener,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=listener,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])
