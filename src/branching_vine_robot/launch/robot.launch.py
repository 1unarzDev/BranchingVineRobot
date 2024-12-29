import launch
import launch_ros
import os

# Define the nodes to be launched concurrently, use the entry points defined in setup.py
def generate_launch_description():
    # Node for launching Intel RealSense d435 camera and start publishing rgbd info
    camera_node = launch_ros.actions.Node(
        package='realsense2_camera ',
        executable='realsense2_camera_node',
        parameters=[{
            'depth_module.depth_profile': '640x480x30',
            'depth_module.infra_profile': '640x480x10',
            'rgb_camera.color_profile': '1280x720x30'
        }]
    )

    # Start state machine
    state_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='state',
        name='state',
    )

    # Standard launch file convention that creates a launch description using all nodes
    return launch.LaunchDescription([
        state_node,
        camera_node
    ])