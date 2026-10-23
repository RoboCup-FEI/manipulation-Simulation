"""Register Isaac's workbench in MoveIt's collision scene."""
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

rclpy.init()
node = rclpy.create_node("lwa4p_isaac_workbench")
client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
success = False
try:
    if not client.wait_for_service(timeout_sec=120):
        raise TimeoutError("MoveIt /apply_planning_scene is unavailable")
    table = CollisionObject(id="isaac_workbench")
    table.header.frame_id = "world"
    table.operation = CollisionObject.ADD
    table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[1.4, 1.2, .05])]
    pose = Pose()
    pose.position.x, pose.position.z, pose.orientation.w = .25, -.13, 1.
    table.primitive_poses = [pose]
    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects = [table]
    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future, timeout_sec=30)
    success = future.done() and future.result() is not None and future.result().success
    if not success:
        raise RuntimeError("MoveIt did not accept the workbench collision object")
    print("[SCENE] Isaac workbench registered in MoveIt", flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if success else 1)
