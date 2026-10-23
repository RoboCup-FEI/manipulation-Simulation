"""Verify real controller motion, MoveIt plan/execute, and two physical fingers."""
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"arm_{i}_joint" for i in range(1, 7)]
FINGERS = ["gripper_finger_left_joint", "gripper_finger_right_joint"]
CACHE = Path(os.environ["LWA4P_ISAAC_CACHE"])
report = {"success": False, "checked_at": datetime.now(timezone.utc).isoformat()}
rclpy.init()
node = rclpy.create_node("lwa4p_isaac_execution_check")
state = {}
subscription = node.create_subscription(JointState, "/joint_states",
    lambda msg: state.update(zip(msg.name, msg.position)), qos_profile_sensor_data)


def wait(predicate, timeout=120):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    if not predicate():
        raise TimeoutError(f"Timed out waiting for {predicate}")


def future_result(future, timeout=120):
    wait(future.done, timeout)
    return future.result()


def settle(target, tolerance=.035):
    wait(lambda: all(name in state and abs(state[name] - value) < tolerance
                     for name, value in target.items()), 30)
    return {name: state[name] for name in target}


def trajectory(controller, target, seconds=3):
    client = ActionClient(node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=60):
        raise RuntimeError(f"{controller} action server unavailable")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(target)
    point = JointTrajectoryPoint(positions=list(target.values()))
    point.time_from_start.sec = seconds
    goal.trajectory.points = [point]
    handle = future_result(client.send_goal_async(goal))
    if not handle.accepted:
        raise RuntimeError(f"{controller} rejected trajectory")
    result = future_result(handle.get_result_async(), 120)
    if result.status != 4 or result.result.error_code != 0:
        raise RuntimeError(f"{controller} failed: {result.status} {result.result}")
    measured = settle(target, .002 if controller == "gripper_controller" else .035)
    client.destroy()
    return measured


try:
    wait(lambda: all(name in state for name in ARM + FINGERS), 180)
    cm = node.create_client(ListControllers, "/controller_manager/list_controllers")
    if not cm.wait_for_service(timeout_sec=60):
        raise RuntimeError("ControllerManager unavailable")
    controllers = future_result(cm.call_async(ListControllers.Request()))
    report["controllers"] = {item.name: item.state for item in controllers.controller}
    assert all(report["controllers"].get(name) == "active" for name in
               ("joint_state_broadcaster", "arm_controller", "gripper_controller"))
    initial = {name: state[name] for name in ARM}
    target = dict(initial)
    target[ARM[0]] += .12
    measured = trajectory("arm_controller", target)
    report["direct_controller"] = {"target": target, "measured": measured,
                                    "movement": measured[ARM[0]] - initial[ARM[0]]}
    assert report["direct_controller"]["movement"] > .08
    print("[VERIFY] Physical arm controller motion: PASS", flush=True)

    # Verify that the regular launcher (also used by the GUI) added the bench.
    scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
    if not scene_client.wait_for_service(timeout_sec=60):
        raise RuntimeError("MoveIt planning scene service unavailable")
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    scene = future_result(scene_client.call_async(request)).scene
    report["workbench_in_planning_scene"] = any(
        item.id == "isaac_workbench" for item in scene.world.collision_objects)
    assert report["workbench_in_planning_scene"], "Workbench missing from MoveIt collision scene"

    target = {name: state[name] for name in ARM}
    target[ARM[0]] += .2
    target[ARM[3]] += .1
    client = ActionClient(node, MoveGroup, "/move_action")
    if not client.wait_for_server(timeout_sec=60):
        raise RuntimeError("MoveGroup action server unavailable")
    goal = MoveGroup.Goal()
    goal.request.group_name = "arm"
    goal.request.pipeline_id = "ompl"
    goal.request.planner_id = "RRTConnectkConfigDefault"
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 10.
    goal.request.max_velocity_scaling_factor = .4
    goal.request.max_acceleration_scaling_factor = .4
    goal.request.start_state.is_diff = True
    constraints = Constraints()
    constraints.joint_constraints = [JointConstraint(joint_name=name, position=value,
        tolerance_above=.005, tolerance_below=.005, weight=1.) for name, value in target.items()]
    goal.request.goal_constraints = [constraints]
    goal.planning_options.plan_only = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    handle = future_result(client.send_goal_async(goal))
    assert handle.accepted, "MoveGroup rejected plan/execute goal"
    response = future_result(handle.get_result_async(), 180)
    result = response.result
    report["moveit"] = {"action_status": response.status, "error_code": result.error_code.val,
                        "planned_points": len(result.planned_trajectory.joint_trajectory.points),
                        "target": target}
    assert response.status == 4 and result.error_code.val == 1, str(report["moveit"])
    assert report["moveit"]["planned_points"] > 0
    report["moveit"]["measured"] = settle(target)
    print("[VERIFY] MoveIt planning and physical execution: PASS", flush=True)
    report["gripper_closed"] = trajectory("gripper_controller", dict.fromkeys(FINGERS, .008))
    target = dict.fromkeys(FINGERS, .0425)
    goal.request.group_name = "gripper"
    constraints = Constraints()
    constraints.joint_constraints = [JointConstraint(joint_name=name, position=value,
        tolerance_above=.0005, tolerance_below=.0005, weight=1.) for name, value in target.items()]
    goal.request.goal_constraints = [constraints]
    handle = future_result(client.send_goal_async(goal))
    assert handle.accepted, "MoveGroup rejected gripper goal"
    response = future_result(handle.get_result_async(), 120)
    report["gripper_moveit"] = {"action_status": response.status,
        "error_code": response.result.error_code.val,
        "planned_points": len(response.result.planned_trajectory.joint_trajectory.points)}
    assert response.status == 4 and response.result.error_code.val == 1, str(report["gripper_moveit"])
    report["gripper_open"] = settle(target, .002)
    print("[VERIFY] Two physical fingers, including MoveIt execution: PASS", flush=True)
    report["success"] = True
except BaseException as exc:
    import traceback
    report["error"] = str(exc)
    traceback.print_exc()
finally:
    (CACHE / "execution.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if report["success"] else 1)
