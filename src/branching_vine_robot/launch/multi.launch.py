import launch
import launch.actions
import launch_ros
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from branching_vine_robot.config import CAMERA_SERIALS

def generate_launch_description():
    launch_desc = []
    
    for cam in CAMERA_SERIALS:
        namespace = cam['namespace']
        serial_no = cam['serial_no']

        single_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('branching_vine_robot'), '/launch/single.launch.py'
            ]),
            launch_arguments={
                'namespace': namespace,
                'serial_no': serial_no
            }.items()
        )
        launch_desc.append(single_launch)

    return launch.LaunchDescription(launch_desc)