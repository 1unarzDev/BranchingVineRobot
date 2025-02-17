import launch
import launch_ros
import launch_ros.actions
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from branching_vine_robot.config import *

# Define the nodes to be launched concurrently, use the entry points defined in setup.py
def generate_launch_description():
    # Node for launching Intel RealSense d435 camera and start publishing rgbd info
    camera_node = launch_ros.actions.Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'depth_module.depth_profile': f'{DEPTH_WIDTH}x{DEPTH_HEIGHT}x{CAM_FPS}',
            'depth_module.infra_profile': f'{DEPTH_WIDTH}x{DEPTH_HEIGHT}x{CAM_FPS}',
            'rgb_camera.color_profile': f'{RGB_WIDTH}x{RGB_HEIGHT}x{CAM_FPS}'
        }]
    )

    # Start state machine
    control_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='control',
        name='control',
    )

    cluster_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='cluster',
        name='cluster',
    )

    server_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='server',
        name='server'
    )
    
    gui_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='gui',
        name='gui'
    )

    shutdown_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
        )
    )
    
    # Standard launch file convention that creates a launch description using all nodes
    return launch.LaunchDescription([
        camera_node,
        cluster_node,
        # server_node,
        gui_node,
        # control_node,
        shutdown_on_exit
    ])