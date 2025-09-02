from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    launch_description = LaunchDescription()
    package_name = 'lab3'
    
    turtlesim = Node(
        package = 'turtlesim_plus',
        executable = 'turtlesim_plus_node.py',
        name = 'turtlesimplus'
    )
    
    eater = Node(
        package = package_name,
        executable = 'eater.py',
        namespace = '/eater6606', 
        parameters=[{"sampling_frequency":100.0}]
    )

    killer = Node(
        package = package_name,
        executable = 'killer.py',
        namespace = '/killer6606',
        parameters=[{"sampling_frequency":20.0}, {"name1":"/eater6606"}]
    )
    
    launch_description.add_action(turtlesim)
    launch_description.add_action(killer)
    launch_description.add_action(eater)

    return launch_description