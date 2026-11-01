"""LWA4P physics and in-process Humble ros2_control, for external MoveIt."""
import argparse
import hashlib
import json
import math
import os
import time
import traceback
import xml.etree.ElementTree as ET
from pathlib import Path

from prepare import ARM, CACHE, FINGERS, HERE, READY, prepare

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--headless", action="store_true")
parser.add_argument("--run-seconds", type=float, default=0)
args, _ = parser.parse_known_args()
prepare()
report = {"success": False, "ready": False}
app = None
manager = None
articulation_path = None
try:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless, "width": 960, "height": 540,
                         "multi_gpu": False, "limit_cpu_threads": 4})
    import isaacsim.core.experimental.utils.app as app_utils
    import omni.timeline
    import omni.usd
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.rendering_manager import ViewportManager
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics

    for extension in ("isaacsim.asset.importer.urdf", "isaacsim.ros2.bridge", "isaacsim.ros2.control"):
        app_utils.enable_extension(extension)
    app.update()
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    from isaacsim.ros2.control import Ros2ControlManager
    from isaacsim.ros2.control.bindings import _isaacsim_ros2_control as backend
    from isaacsim.ros2.nodes import Ros2ClockGraphConfig, create_ros2_clock_graph

    source = HERE.parent / "lwa4p_wsg50.urdf"
    digest = hashlib.sha256(source.read_bytes() + b"moveit-6.2-v1").hexdigest()[:12]
    asset_dir = CACHE / "usd" / digest
    manifest = asset_dir / "asset.json"
    if manifest.exists():
        usd_path = json.loads(manifest.read_text())["usd_path"]
    else:
        asset_dir.mkdir(parents=True, exist_ok=True)
        cfg = URDFImporterConfig(
            urdf_path=str(source), usd_path=str(asset_dir), fix_base=True,
            merge_fixed_joints=False, merge_mesh=False, allow_self_collision=False,
            collision_type="Convex Decomposition", joint_drive_type="force",
            joint_target_type="position",
            override_joint_stiffness={"arm_[1-4]_joint": 4000., "arm_[5-6]_joint": 1000.,
                                      "gripper_finger_.*_joint": 200.},
            override_joint_damping={"arm_[1-4]_joint": 120., "arm_[5-6]_joint": 30.,
                                    "gripper_finger_.*_joint": 200.})
        importer = URDFImporter()
        importer.config = cfg
        usd_path = str(Path(importer.import_urdf()).resolve())
        manifest.write_text(json.dumps({"usd_path": usd_path}) + "\n")

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    root = stage.DefinePrim("/World/LWA4P")
    root.GetReferences().AddReference(usd_path)
    root.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr(700.)
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(1200.)
    UsdGeom.XformCommonAPI(sun).SetRotate((25., 25., 0.))
    # The source URDF's mounting surface is at z=-0.105 m.
    table = UsdGeom.Cube.Define(stage, "/World/Table")
    UsdGeom.XformCommonAPI(table).SetTranslate((.25, 0., -.13))
    UsdGeom.XformCommonAPI(table).SetScale((.7, .6, .025))
    table.CreateDisplayColorAttr([(0.32, 0.35, 0.4)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    for prim in Usd.PrimRange(root):
        # The fixed, overhead camera is a separate fixed-world URDF branch.
        # It must not appear as a second controllable articulation to ros2_control.
        if prim.GetName() == "camera_link" and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            api.CreateSolverPositionIterationCountAttr(64)
            api.CreateSolverVelocityIterationCountAttr(16)
            api.CreateSleepThresholdAttr(0.)
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            name = prim.GetName()
            if name in READY:
                angular = name in ARM
                drive = UsdPhysics.DriveAPI.Apply(prim, "angular" if angular else "linear")
                drive.CreateTargetPositionAttr(math.degrees(READY[name]) if angular else READY[name])
                drive.CreateMaxForceAttr(64. if name in ARM[:4] else 19. if angular else 80.)
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.6, 1.5, 1.25],
                                    target=[.1, 0., .45])
    create_ros2_clock_graph(Ros2ClockGraphConfig(graph_path="/Graph/Clock"))
    SimulationManager.setup_simulation(dt=1 / 120., device="cpu")
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(8):
        app.update()
    robot = Articulation("/World/LWA4P")
    positions = [[READY[name] for name in robot.dof_names]]
    robot.set_dof_positions(positions)
    robot.set_dof_position_targets(positions)
    for _ in range(30):
        app.update()
    if not backend.is_ready():
        raise RuntimeError("Humble ros2_control backend did not become ready")
    articulation_path = next(str(prim.GetPath()) for prim in Usd.PrimRange(root)
        if prim.GetName() == "arm_podest_link" and prim.HasAPI(UsdPhysics.ArticulationRootAPI))
    timeline.pause()
    for _ in range(2):
        app.update()
    manager = Ros2ControlManager
    # Keep the source URDF: the stock live USD exporter treats the fixed camera
    # as a second articulation and can invalidate compound-collider tensor views.
    # Use the same native ControllerManager/IsaacSimSystem backend with an
    # explicit standard ros2_control block for the eight real driven joints.
    description = ET.parse(CACHE / "robot.urdf").getroot()
    control = ET.SubElement(description, "ros2_control", name="LWA4P", type="system")
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "isaacsim_ros2_control/IsaacSimSystem"
    ET.SubElement(hardware, "param", name="prim_path").text = articulation_path
    for name in ARM + FINGERS:
        joint = ET.SubElement(control, "joint", name=name)
        ET.SubElement(joint, "command_interface", name="position")
        for interface in ("position", "velocity", "effort"):
            ET.SubElement(joint, "state_interface", name=interface)
    rc = backend.setup_cm(articulation_path=articulation_path,
        urdf_xml=ET.tostring(description, encoding="unicode"),
        controller_yaml_path=str(HERE / "controllers.yaml"), ns_name="",
        publish_robot_description=False, use_sim_time=True)
    if rc != 0:
        raise RuntimeError(f"Native Humble ControllerManager setup failed: {rc}")
    timeline.play()
    report.update(ready=True, articulation=articulation_path, joint_names=robot.dof_names,
                  controller_manager="/controller_manager", ros_distro=os.environ.get("ROS_DISTRO"))
    (CACHE / "simulation.json").write_text(json.dumps(report, indent=2) + "\n")
    print("[LWA4P] READY: Humble controller_manager and physical articulation", flush=True)
    started = time.monotonic()
    while app.is_running() and (not args.run_seconds or time.monotonic() - started < args.run_seconds):
        tick = time.monotonic()
        app.update()
        time.sleep(max(0., 1 / 120. - (time.monotonic() - tick)))
    report["success"] = True
except KeyboardInterrupt:
    report["success"] = report["ready"]
except BaseException as exc:
    report["error"] = str(exc)
    traceback.print_exc()
finally:
    if manager is not None and articulation_path is not None:
        manager.teardown(articulation_path)
    (CACHE / "simulation.json").write_text(json.dumps(report, indent=2) + "\n")
    if app is not None:
        app.close(exit_code=0 if report["success"] else 1)
