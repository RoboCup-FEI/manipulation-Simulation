"""Prepare MoveIt's description from the exact URDF used by Isaac Sim."""
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
CACHE = Path(os.environ.get("LWA4P_ISAAC_CACHE", HERE / ".runtime"))
ARM = [f"arm_{i}_joint" for i in range(1, 7)]
FINGERS = ["gripper_finger_left_joint", "gripper_finger_right_joint"]
READY = dict(zip(ARM + FINGERS, [0., -.35, .8, 0., 2., 0., .0425, .0425]))


def prepare():
    CACHE.mkdir(parents=True, exist_ok=True)
    source = HERE.parent / "lwa4p_wsg50.urdf"
    urdf = ET.parse(source)
    for mesh in urdf.iter("mesh"):
        mesh_path = (source.parent / mesh.get("filename")).resolve()
        if not mesh_path.is_file():
            raise FileNotFoundError(mesh_path)
        mesh.set("filename", mesh_path.as_uri())
    urdf.write(CACHE / "robot.urdf", encoding="unicode", xml_declaration=True)
    srdf = ET.parse(HERE.parents[1] / "src/lwa4p_moveit_config/config/lwa4p.srdf")
    gripper = srdf.find("group[@name='gripper']")
    for child in list(gripper):
        gripper.remove(child)
    for name in FINGERS:
        ET.SubElement(gripper, "joint", name=name)
    for state in srdf.findall("group_state[@group='gripper']"):
        gap = float(state.find("joint").get("value"))
        for child in list(state):
            state.remove(child)
        for name in FINGERS:
            ET.SubElement(state, "joint", name=name, value=str(gap / 2))
    state = ET.SubElement(srdf.getroot(), "group_state", name="ready", group="arm")
    for name in ARM:
        ET.SubElement(state, "joint", name=name, value=str(READY[name]))
    srdf.write(CACHE / "robot.srdf", encoding="unicode", xml_declaration=True)
    (CACHE / "initial.json").write_text(json.dumps(READY, indent=2) + "\n")
    return CACHE


if __name__ == "__main__":
    print(prepare())
