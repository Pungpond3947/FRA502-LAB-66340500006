from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    launch_description = LaunchDescription()
    namespace_teleop = '/teleop'
    namespace_copy = '/copy'
    turtle1 = 'Foxy'
    turtle2 = 'Noetic'
    turtle3 = 'Humble'
    turtle4 = 'Iron'
    name_eraser = 'eraser'
    
    turtlesim1 = Node(
        package = 'turtlesim_plus',
        executable = 'turtlesim_plus_node.py',
        namespace = namespace_teleop,
        name = 'turtlesimplus'
    )

    turtlesim2 = Node(
        package = 'turtlesim_plus',
        executable = 'turtlesim_plus_node.py',
        namespace = namespace_copy,
        name = 'turtlesimplus'
    )

    kill = Node(
        package = 'Exam1',
        executable = 'kill_turtle.py',
        parameters = [{"name_teleop":namespace_teleop}, {"name_copy":namespace_copy}]
    )

    spawn = Node(
        package = 'Exam1',
        executable = 'spawn_turtle.py',
        parameters = [{"name_teleop":namespace_teleop}, 
                      {"name_copy":namespace_copy}, 
                      {"turtle1":turtle1},
                      {"turtle2":turtle2},
                      {"turtle3":turtle3},
                      {"turtle4":turtle4}]
    )

    copy_turtle = Node(
        package = 'Exam1',
        executable = 'copy_turtle.py',
        parameters = [{"sampling_frequency":100.0},
                      {"name_teleop":namespace_teleop},
                      {"name_copy":namespace_copy}, 
                      {"turtle1":turtle1},
                      {"turtle2":turtle2},
                      {"turtle3":turtle3},
                      {"turtle4":turtle4}]
    )

    eraser = Node(
        package = 'Exam1',
        executable = 'eraser_turtle.py',
        parameters = [{"sampling_frequency":100.0},
                      {"name_teleop":namespace_teleop},
                      {"name_copy":namespace_copy}, 
                      {"turtle1":turtle1},
                      {"turtle2":turtle2},
                      {"turtle3":turtle3},
                      {"turtle4":turtle4},
                      {"name_eraser":name_eraser}]
    )

    teleop = Node(
        package = 'Exam1',
        executable = 'teleop_turtle.py',
        parameters = [{"sampling_frequency":100.0},
                      {"name_teleop":namespace_teleop}]
    )

    kill_node = Node(
        package = 'Exam1',
        executable = 'kill_node.py',
    )
    
    delayed_teleop = TimerAction(
        period=3.0, 
        actions=[teleop]
    )

    launch_description.add_action(turtlesim1)
    launch_description.add_action(turtlesim2)
    launch_description.add_action(kill)
    launch_description.add_action(spawn)
    launch_description.add_action(copy_turtle)
    launch_description.add_action(eraser)
    launch_description.add_action(kill_node)
    launch_description.add_action(delayed_teleop) 

    return launch_description