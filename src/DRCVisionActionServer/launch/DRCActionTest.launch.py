from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import execute_process

def generate_launch_description():


    return LaunchDescription([
        Node(
            package= 'dasl_playground',
            node_executable='drc_vision_action_server',
            output='both',
            arguments=[('--log-level:=info')],

        )
    ])