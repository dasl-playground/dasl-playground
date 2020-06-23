from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackage

def generate_launch_description():

    rosbridge_websokect_param = LaunchConfiguration('rosbridge_websokect_pram', default=os.path.join(
        get_package_share_directory('file_server'),'param','rosbridge_param.yaml'))

    rosapi_param = LaunchConfiguration('rosapi_param',default=os.path.join(get_package_share_directory(
        'file_server'),'param', 'rosapi.yaml'))

    urdf_name = 'testbot.urdf'

    urdf = os.path.join(get_package_share_directory('file_server'),'urdf',urdf_name)
    # urdf_param = {'robot/name': test, 'robot_description':}

    #pkg_share = FindPackage('file_server')
   # urdf_file = os.path.join(pkg_share, 'urdf', urdf_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription([
        Node(
            package= 'rosbridge_server',
            node_executable='rosbridge_websocket',
            parameters=[rosbridge_websokect_param],
            output='screen'
        ),
        Node(
            package= 'rosapi',
            node_executable='rosapi_node',
            parameters=[rosapi_param],
            output='screen'
        ),
        Node(
            package= 'file_server',
            node_executable= 'file_server',
            node_name= 'file_server',
            output= 'screen',
            parameters= [rsp_params]
        )
    ])