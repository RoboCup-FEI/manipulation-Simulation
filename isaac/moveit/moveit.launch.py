"""MoveIt/RViz for the real ros2_control controllers hosted by Isaac Sim."""
from pathlib import Path
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    here = Path(__file__).resolve().parent
    cache = Path(os.environ["LWA4P_ISAAC_CACHE"])
    config = (MoveItConfigsBuilder("lwa4p", package_name="lwa4p_moveit_config")
              .robot_description(file_path=str(cache / "robot.urdf"))
              .robot_description_semantic(file_path=str(cache / "robot.srdf"))
              .trajectory_execution(file_path=str(here / "moveit_controllers.yaml"))
              .planning_pipelines(pipelines=["ompl"]).to_moveit_configs())
    limits = config.joint_limits["robot_description_planning"]["joint_limits"]
    limits.pop("gripper_joint", None)
    for name in ("gripper_finger_left_joint", "gripper_finger_right_joint"):
        limits[name] = {"has_velocity_limits": True, "max_velocity": .05,
                        "has_acceleration_limits": True, "max_acceleration": .1}
    sim_time = {"use_sim_time": True}
    params = config.to_dict()
    params.update({"publish_robot_description_semantic": True,
                   "publish_planning_scene": True,
                   "publish_geometry_updates": True,
                   "publish_state_updates": True,
                   "publish_transforms_updates": True})
    controllers = [Node(package="controller_manager", executable="spawner", output="screen",
                        arguments=[name, "--controller-manager", "/controller_manager",
                                   "--controller-manager-timeout", "120"])
                   for name in ("joint_state_broadcaster", "arm_controller", "gripper_controller")]
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        *controllers,
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description, sim_time], output="screen"),
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[params, sim_time], output="screen"),
        ExecuteProcess(cmd=[sys.executable, str(here / "planning_scene.py")], output="screen"),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", str(here.parents[1] / "src/lwa4p_moveit_config/config/moveit.rviz")],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines,
                         config.joint_limits, sim_time], output="log"),
    ])
