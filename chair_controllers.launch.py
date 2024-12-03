import os
import json
import sys
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    nchairs = 5  # Default number of chairs
    for arg in sys.argv:  # Check for command-line arguments
        if arg.startswith('nchairs:='):
            nchairs = int(arg.split('nchairs:=', 1)[1])
        elif ':=' in arg:
            print(f"Unknown argument in {arg}")
            sys.exit(0)

    print(f"Controlling {nchairs} chairs")

    # List to store the nodes that will be launched
    nodelist = []

    # Launch the leader chair (chair 0), which will not follow anyone
    nodelist.append(
        Node(
            namespace="chair_0",
            package='cpmr_ch11',
            executable='leader_chair',
            name='leader_chair',
            output='screen',
            parameters=[{'chair_name': "chair_0"}]  # leader chair has no target
        )
    )
    print(f"Leader chair launched")

    # Launch the following chairs (chair 1 to nchairs-1)
    for chair in range(1, nchairs):
        name = f'chair_{chair}'  # Current chair name
        print(f"Processing chair {chair}")
        
        # Set the target chair as the one before the current chair
        target_name = f"chair_{chair - 1}"
        
        # Launch a following chair that follows the previous chair
        nodelist.append(
            Node(
                namespace=name,
                package='cpmr_ch11',
                executable='follow_chair',
                name='follow_chair',
                output='screen',
                parameters=[{'chair_name': name, 'target_name': target_name}]
            )
        )

    # Return the LaunchDescription with the list of all nodes
    return LaunchDescription(nodelist)

