"""Share the robot model and planner configuration between MoveIt and MTC."""
import os
from pathlib import Path

from moveit_configs_utils import MoveItConfigsBuilder


def moveit_configuration():
    here = Path(__file__).resolve().parent
    cache = Path(os.environ['LWA4P_ISAAC_CACHE'])
    config = (
        MoveItConfigsBuilder('lwa4p', package_name='lwa4p_moveit_config')
        .robot_description(file_path=str(cache / 'robot.urdf'))
        .robot_description_semantic(file_path=str(cache / 'robot.srdf'))
        .trajectory_execution(file_path=str(here / 'moveit_controllers.yaml'))
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    limits = config.joint_limits['robot_description_planning']['joint_limits']
    limits.pop('gripper_joint', None)
    for name in ('gripper_finger_left_joint', 'gripper_finger_right_joint'):
        limits[name] = {
            'has_velocity_limits': True,
            'max_velocity': 0.05,
            'has_acceleration_limits': True,
            'max_acceleration': 0.1,
        }
    return config
