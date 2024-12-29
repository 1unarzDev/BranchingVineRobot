import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

# Define the nodes to be launched concurrently, use the entry points defined in setup.py
def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='branching_vine_robot').find('branching_vine_robot')

    camera_node = launch_ros.actions.Node(
        package='realsense2_camera ',
        executable='realsense2_camera_node',
        parameters=[{
            depth_module.depth_profile:=640x480x30
            depth_module.infra_profile:=640x480xl0 
            rgb_camera.color_profile:=1280x720x30
        }]
    )

    state_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='state',
        name='state',
    )

    return launch.LaunchDescription([
        state_node,
        camera_node
    ])