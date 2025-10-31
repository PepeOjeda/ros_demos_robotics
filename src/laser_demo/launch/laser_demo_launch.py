import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
from ros2launch.api import get_share_file_path_from_package
from launch_ros.parameter_descriptions import ParameterFile

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("", default_value=""),
   ]
#==========================

def launch_setup(context, *args, **kwargs):
    hokuyo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file_path_from_package(package_name="urg_node", file_name="urg_node_launch.py")
        )
    )
    tf_pub = Node(
        package="tf_publisher_gui",
        executable="gui_pub",
        parameters=[
            {'parent_frame' :  'map'},
            {'child_frame' :   'laser'}
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2", 
        arguments=[
            "-d" + get_share_file_path_from_package(package_name="laser_demo", file_name="demo.rviz")
        ],
    )


    return [
        hokuyo,
        tf_pub,
        rviz,
    ]


def generate_launch_description():

    launch_description = [
       # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
   ]
   
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
   
    return  LaunchDescription(launch_description)