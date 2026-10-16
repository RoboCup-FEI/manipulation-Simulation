import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drv = get_package_share_directory("lwa4p_driver")
    return LaunchDescription([
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        # autorepeat keeps /joy alive while the sticks are still; teleop stops when it goes quiet
        Node(package="joy_linux", executable="joy_linux_node", name="joy",
             parameters=[{"dev": LaunchConfiguration("joy_dev"), "autorepeat_rate": 20.0}]),
        Node(package="lwa4p_driver", executable="teleop", name="teleop", output="screen",
             parameters=[os.path.join(drv, "config", "teleop.yaml"), {"keyboard": False}]),
    ])
