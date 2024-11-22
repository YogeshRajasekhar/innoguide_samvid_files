from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Dynamically find package directories
    diff_drive_pkg_dir = get_package_share_directory('diff_drive_controller')
    robot_localization_pkg_dir = get_package_share_directory('robot_localization')

    return LaunchDescription([
        # Include diff_drive_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{diff_drive_pkg_dir}/launch/diff_drive_launch.py"
            )
        ),
        # Include robot_localization_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{robot_localization_pkg_dir}/launch/robot_localization_launch.py"
            )
        )
    ])
