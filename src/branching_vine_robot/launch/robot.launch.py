import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

# Define the nodes to be launched concurrently, use the entry points defined in setup.py
def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='branching_vine_robots').find('branching_vine_robots')

    # camera_node = launch_ros.actions.Node(
    #     package='camera_ros',
    #     executable='camera_node',
    #     output='screen',
    #     parameters=[{
    #         'width': 640,   
    #         'height': 480,
    #         'log_level': 'debug',   
    #     }]
    # )

    boat_state_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='state',
        name='state',
        arguments=[os.path.join(pkg_share, 'branching_vine_robot', 'state.py')],
    )

    return launch.LaunchDescription([
        boat_state_node
    ])