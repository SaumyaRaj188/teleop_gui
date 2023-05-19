from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

package_name = 'teleop_gui'

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "test", default_value=TextSubstitution(text="0")
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket.py',
            name='rosbridge'
        ),
        Node(
            package='teleop_gui',
            executable='controller',
            name='controller'
        ),
        Node(
            package='teleop_gui',
            executable='test_listener',
            condition=IfCondition(LaunchConfiguration('test')),
            name='teleop_gui_listener'
        )
    ])