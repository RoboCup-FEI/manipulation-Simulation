import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('lwa4p_calibration_poses'),
        'config',
        'calibration_poses.yaml'
    )
    with open(config_file) as config:
        shared_config = yaml.safe_load(config)['/**']['ros__parameters']
    pose_file_parameter = {'pose_file_path': LaunchConfiguration('pose_file_path')}
    settle_time_parameter = {
        'settle_time_ms': ParameterValue(LaunchConfiguration('settle_time_ms'), value_type=int)
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_file_path',
            default_value='~/calibration_poses.txt',
            description='Pose file shared by pose_teacher and calibration_sequence',
        ),
        DeclareLaunchArgument(
            'settle_time_ms',
            default_value=str(shared_config['settle_time_ms']),
            description='Time to hold still at every pose of the calibration sequence [ms]',
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
            parameters=[config_file, pose_file_parameter, settle_time_parameter],
        ),
    ])
