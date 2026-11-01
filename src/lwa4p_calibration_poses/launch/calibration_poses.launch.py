import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('lwa4p_calibration_poses'),
        'config',
        'calibration_poses.yaml'
    )
    pose_file_parameter = {'pose_file_path': LaunchConfiguration('pose_file_path')}

    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_file_path',
            default_value='~/calibration_poses.txt',
            description='Pose file shared by pose_teacher and calibration_sequence',
        ),
        Node(
            package='lwa4p_calibration_poses',
            executable='pose_teacher_node',
            name='pose_teacher',
            output='screen',
            parameters=[config_file, pose_file_parameter],
        ),
        Node(
            package='lwa4p_calibration_poses',
            executable='calibration_sequence_node',
            name='calibration_sequence',
            output='screen',
            parameters=[config_file, pose_file_parameter],
        ),
    ])
