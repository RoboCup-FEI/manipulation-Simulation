# Copyright 2026 Dmitrii Ognev
# Licensed under the MIT License. See LICENSE in the repository root.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('lwa4p_calibration_poses'),
        'config',
        'calibration_poses.yaml'
    )

    return LaunchDescription([
        Node(
            package='lwa4p_calibration_poses',
            executable='pose_teacher_node',
            name='pose_teacher',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='lwa4p_calibration_poses',
            executable='calibration_sequence_node',
            name='calibration_sequence',
            output='screen',
            parameters=[config_file],
        ),
    ])
