from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_node',
            executable='smart_meter_node',
            name='smart_meter_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),
         
        Node(
            package='smart_node',
            executable='smart_meter_waring',
            name='smart_meter_waring',
            output='screen',
            parameters=[],
            remappings=[]
        ),
    ])