import launch
import launch_ros
import launch_ros.actions
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from branching_vine_robot.config import DEPTH_HEIGHT, DEPTH_WIDTH, CAM_FPS, RGB_HEIGHT, RGB_WIDTH, CAMERA_CONFIG

DEFAULT_CAM_SERIAL_NO = CAMERA_CONFIG[0]['serial_no']

# Define the nodes to be launched concurrently, use the entry points defined in setup.py
def generate_launch_description():
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot', description='Namespace for camera')
    serial_no_arg = DeclareLaunchArgument('serial_no', default_value=DEFAULT_CAM_SERIAL_NO, description='Camera serial number')

    namespace = LaunchConfiguration('namespace')
    serial_no = LaunchConfiguration('serial_no')

    # Node for launching Intel RealSense d435 camera and start publishing rgbd info
    camera_node = launch_ros.actions.Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=namespace,
        parameters=[{
            'serial_no': serial_no,
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
        namespace=namespace
    )

    cluster_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='cluster',
        name='cluster',
        namespace=namespace
    )

    server_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='server',
        name='server',
        namespace=namespace
    )
    
    gui_node = launch_ros.actions.Node(
        package='branching_vine_robot',
        executable='gui',
        name='gui',
        namespace=namespace
    )

    shutdown_on_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
        )
    )
    
    # Standard launch file convention that creates a launch description using all nodes
    return launch.LaunchDescription([
        namespace_arg,
        serial_no_arg,
        camera_node,
        cluster_node,
        gui_node,
        # server_node,
        # control_node,
        shutdown_on_exit
    ])