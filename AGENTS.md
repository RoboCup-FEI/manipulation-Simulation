# AGENTS.md — working notes for the LWA 4P manipulation stack

Notes for agents and developers working on this repository. The READMEs explain how to
*use* the stack. This file covers how it is wired, where the traps are, and what to keep
in sync when you change something. Read it before editing.

- User docs: [README.md](README.md), [docker/README.md](docker/README.md),
  [src/lwa4p_gazebo/README.md](src/lwa4p_gazebo/README.md),
  [isaac/README.md](isaac/README.md), [isaac/moveit/README.md](isaac/moveit/README.md),
  [src/lwa4p_mtc_verification/README.md](src/lwa4p_mtc_verification/README.md).

---

## 1. What this is

- The manipulation subsystem of a RoboCup (FEI) medical-assistance robot: a **Schunk LWA 4P**
  6-DoF arm, a **Weiss/Schunk WSG 50-110** parallel gripper, and a **RealSense D435** looking
  down at the work area.
- **ROS 2 Humble only** (Ubuntu 22.04). MoveIt 2 (2.5.x) and Gazebo **Classic** 11.
- The repo is forked from `Tomas-Jurov/lwa4p_driver`. Git origin is
  `github.com/RoboCup-FEI/manipulation-Simulation`, and the working branch is `isaac-setup`.
- The repo root **is** the colcon workspace (`src/` lives here). Build from this directory.
  `/heavy_stuff/robocup_ws` around it is not a git repo.
- There is **no mobile base, navigation or task-level code** here yet. The `world` frame is
  the arm base, everything is a fixed-base table setup, and the "medical" content so far is a
  15 mm test tube on a stand plus a bin-picking scene. The MTC package says it is "not the
  manipulation skill server", so a skill server is expected but does not exist yet.

### Team decisions (2026-10-26)

- **The real robot keeps the Python CAN driver (`lwa4p_driver`) as it is.** Simulation keeps
  `ros2_control`. The team accepts the sim-to-real gap and will not write a `ros2_control`
  hardware interface or move to `ros2_canopen`. Do not propose that rewrite.
- The interface shared by sim and real is the **action level**: `arm_controller/follow_joint_trajectory`,
  `gripper_controller/*` and `/joint_states`. Code above it (MoveIt, MTC, a skill server, the
  behaviour tree) must not depend on which backend is running.
- Navigation and VLM perception belong to other teams. This repo consumes their outputs
  (poses, TF) and does not implement them.
- The planned stack is described in [manipulation_architecture.txt](manipulation_architecture.txt).
  Its lower layers predate the driver decision. See **§13** for how to read it.

### The host machine (checked 2026-10-26)

- Ubuntu **24.04** with ROS 2 **Jazzy** in `/opt/ros/jazzy`. The `.vscode` settings point at
  Jazzy too.
- Gazebo Classic plugins do not exist for Jazzy, so **do not try to build or run natively**.
  Use the Docker image `lwa4p:humble` ([docker/Dockerfile](docker/Dockerfile)).
- The NVIDIA driver was not working (`nvidia-smi` failed) and no Isaac Sim install was found
  at `../isaac-setup/runtime/isaac-sim-6.2.0` or `~/isaacsim`. The Isaac flow cannot run
  here unless that changes.
- The `lwa4p:humble` image was not built yet.

---

## 2. Packages

| Package | Type | Role |
|---|---|---|
| `lwa4p_description` | ament_cmake (data only) | Xacro for the arm, the WSG 50 (`wsg50.urdf.xacro`) and the D435 (`realsense_d435.urdf.xacro`), plus meshes and `display.launch.py`. The base xacro has **no camera**; the camera is added by `lwa4p_gazebo`. |
| `lwa4p_driver` | ament_python | Real hardware. `driver` = CANopen CiA 402 arm driver (`driver_node.py` + `ds402.py`). `gripper` = WSG 50 node (`gripper_node.py` + `wsg50.py`, a raw binary protocol over CAN). CLIs `test_move` and `gripper_cmd`. Launch file: `bringup.launch.py`. |
| `lwa4p_moveit_config` | ament_cmake | Setup Assistant output: SRDF, kinematics, joint limits, controllers. `demo.launch.py` (mock hardware), `real.launch.py` (driver + move_group + RViz). |
| `lwa4p_gazebo` | ament_cmake + scripts | Gazebo world, `gazebo.launch.py`, sim controllers, and the Python user API `scripts/simple_moveit.py` plus the demos `go_to.py`, `pick_tube.py`, `bin_pick.py` and the sim helper `finger_mimic.py`. |
| `lwa4p_mtc_verification` | ament_cmake C++17 | `verify_mtc` node: the `/lwa4p/test_ptp` Trigger service runs a 4-stage MTC task and checks the *measured* joint motion. Used for Isaac only. Follow its `.clang-format` (Google-based, 100 columns, Allman braces for functions, classes and namespaces). |
| `isaac/` (not a ROS package) | scripts | Plain URDF `lwa4p_wsg50.urdf` with datasheet masses, its generator `tools/make_isaac_urdf.py`, the Isaac Lab template `lwa4p_cfg.py`, and the Isaac Sim 6.2 + MoveIt launcher `isaac/moveit/`. |

The dependency chain is `description ← moveit_config ← gazebo`. `moveit_config` also depends
on `lwa4p_driver` at exec time, because `real.launch.py` includes `bringup.launch.py`.

---

## 3. Three backends, one MoveIt interface, and where they differ

All backends expose `arm_controller` and `gripper_controller` and publish `/joint_states`, so
MoveIt client code is meant to be backend-agnostic. **The gripper is modelled differently in
each backend.** Getting this wrong is the most likely bug.

| | Real robot | Gazebo Classic | Isaac Sim 6.2 | Mock (`demo.launch.py`) |
|---|---|---|---|---|
| Arm action | `/arm_controller/follow_joint_trajectory`, served by the Python driver | JTC via gazebo_ros2_control, PID force-driven | JTC in Isaac's in-process `controller_manager` | JTC, mock_components |
| Gripper joint(s) MoveIt commands | `gripper_joint` = **fingertip gap** (m) | `gripper_joint` (gap) | **`gripper_finger_left_joint` + `gripper_finger_right_joint`**, each = gap/2. There is **no `gripper_joint`**. | `gripper_joint` |
| Gripper controller type | **GripperCommand** action `gripper_controller/gripper_cmd` (served by `gripper_node`) | **FollowJointTrajectory** (JTC on `gripper_joint`), then `finger_mimic.py` drives `finger_controller` (ForwardCommand) | FollowJointTrajectory on the two finger joints | `GripperActionController` |
| MoveIt controllers file | `lwa4p_moveit_config/config/moveit_controllers.yaml` | `lwa4p_gazebo/config/moveit_controllers.yaml` | `isaac/moveit/moveit_controllers.yaml` | moveit_config |
| URDF used | `lwa4p_description/urdf/lwa4p.urdf.xacro` (mimic fingers) | `lwa4p_gazebo/urdf/lwa4p_gazebo.urdf.xacro` (+ camera, ros2_control, PID gains) | `isaac/lwa4p_wsg50.urdf` → `.runtime/robot.urdf` (mesh paths rewritten as `file://`) | moveit_config xacro |
| SRDF | `config/lwa4p.srdf` | the same | **rewritten** by `isaac/moveit/prepare.py`: the gripper group becomes the 2 finger joints, gripper states are split into gap/2, and a `ready` arm state is added | the same |
| `use_sim_time` | no | yes | yes | no |
| Gravity on the arm | real | **off** (`turnGravityOff` on every robot link) | on, with stiff drives | n/a |
| Camera | not integrated yet (no realsense2 launch) | simulated D435 depth sensor | D435 frame only, no RGB-D published | none |

Consequences:
- `SimpleMoveIt.gripper(gap)` sets a joint constraint on `gripper_joint`. It works on the real
  robot, in Gazebo and in mock mode, but **not in Isaac**. Code meant for Isaac must command the
  finger joints, or go through the SRDF named states (`open`, `tube_15mm`, `closed`). Those
  names exist in every backend because `prepare.py` keeps them.
- On the real robot, MoveIt sends `max_effort: 35.0` (from `moveit_controllers.yaml`).
  `gripper_node.execute()` runs a **force-controlled grasp** when `max_effort > 0` and the target
  gap is more than 1 mm below the current gap. Otherwise it runs a position `move`.
  `max_effort == 0` means use `default_force_n`, and a negative value always means move
  (`gripper_cmd move` sends -1).
- In Gazebo, `robot_state_publisher` and `move_group` are remapped to
  **`/joint_states_measured`**. `finger_mimic.py` republishes `/joint_states` there with
  `gripper_joint` replaced by the *measured* distance between the fingers. If you add a node
  that needs the gripper's true opening in sim, subscribe to that topic, or read the finger
  joints the way `bin_pick.py` does.
- Real `/joint_states` has two publishers: the driver (6 arm joints, 20 Hz) and the gripper node
  (`gripper_joint`, 10 Hz). The gripper also publishes `gripper/state` (gap in `position`, force
  in `effort`).

---

## 4. Frames and geometry to keep in mind

- `world` = URDF root = the robot base frame, 0.10 m below the shoulder axes. The real
  mounting surface is at **z = −0.105**. The Gazebo floor is at **z = −0.06**, an old pedestal
  mesh. The Isaac table top is at about −0.105.
- `arm_tool0` = flange. `gripper_tcp` = between the fingertips, **15 mm above the tips**. This
  is the IK tip of the `arm` group (chain `arm_base_link → gripper_tcp`) and the frame all
  grasp poses use. The flange-to-TCP distance is about 0.251 m.
- The gripper is mounted with `gripper_yaw` = π/2 about the flange axis (a xacro arg on
  `lwa4p.urdf.xacro`). The fingers close along the TCP x-axis.
- "Pointing down" in `SimpleMoveIt.pose()` is the quaternion `(x=cos(yaw/2), y=sin(yaw/2),
  z=0, w=0)`, i.e. a 180° flip about a horizontal axis, where `yaw` is the finger closing
  direction. With `yaw=None` the tool's z rotation is free (tolerance π).
- Reach with the gripper pointing down is about **0.55 m** at table height. Beyond that,
  expect `NO_IK_SOLUTION` or `INVALID_MOTION_PLAN`.
- Camera: `world → camera_link` at xyz `0.45 -0.15 0.95`, rpy `0 π/2 0`. This is a
  **placeholder** until hand-eye calibration. Data comes in `camera_color_optical_frame`
  (424×240 at 5 Hz, fx = fy = 306.2).
- Joint *i* is CANopen node *i+2* (nodes 3–8 on `can0`). The WSG uses `can1`, CAN IDs
  TX 0x01 / RX 0x02.
- The WSG reports **jaw width**. The gap used everywhere in ROS is
  `gap = jaw − finger_offset_mm (19.5)`. The usable gap is 0 to about 0.0895 m, and `open` is
  0.085.

## 5. Limits: several files must agree

The real arm's power supply is weak, so it is held to **about 0.5 rad/s and 0.4 rad/s²**.
Faster moves trip DC-link undervoltage `0x3222`. These numbers live in several places:

| File | Value |
|---|---|
| `lwa4p_moveit_config/config/joint_limits.yaml` | 0.5 rad/s, 0.4 rad/s², default scaling 0.8. Gripper 0.05 m/s. |
| `lwa4p_driver/config/driver.yaml` | `max_velocity_deg_s: 30` (≈0.524 rad/s), `acceleration_deg_s2: 60`, `limit_margin_deg: 3` |
| `lwa4p_description/urdf/lwa4p.urdf.xacro` | datasheet velocity of 72°/s, range ±170° (±155° on axis 3) minus 0.02 rad |
| `isaac/lwa4p_cfg.py` | `REAL_SPEED = 0.5`, datasheet accelerations |
| `isaac/moveit/configuration.py` | adds finger limits of 0.05 m/s and 0.1 m/s² (Isaac only) |

The driver **rejects** a trajectory if any segment exceeds 1.05 × `max_velocity_deg_s`, if a
point is within `limit_margin_deg` of the drive's own software limits (read over SDO 0x607D),
if the first point is more than 2° from the current position, or if the drives are not enabled.
If you raise MoveIt limits or scaling, raise the driver limit too, or MoveIt plans will be
rejected on the real arm.

## 6. Real-driver internals (`lwa4p_driver/driver_node.py`)

- It uses **Profile Position mode only**, with absolute targets in milli-degrees. A 20 Hz loop
  thread samples the trajectory (cubic Hermite when velocities are given, otherwise linear)
  at `t + lookahead_s` and sends that as a new PP setpoint, with a velocity sized to arrive in
  time. The drives do the smoothing. This is **not** a streaming CSP or servo interface. Keep
  that in mind before adding MoveIt Servo or a high-rate policy (the README recommends 5–10 Hz).
- `~/enable` brings up NMT, resets faults, sets PP mode, enables the drives in node order, and
  runs a **±0.5° commutation move on every joint**. **The real arm physically moves when you
  call it.**
- Path tolerance violations (8°) hold the arm and abort. Drive faults and CAN errors call
  `trip()`, which disables all drives so the brakes close.
- Every trajectory writes a CSV trace to `~/.ros/lwa4p_trace_*.csv`. `~/diagnose` is read-only:
  fault history, temperature (undocumented object 0x200D.1), and torque/current.
- Threading: one `can_lock` covers all SDO traffic, and `state_lock` covers the trajectory
  state. The node uses a MultiThreadedExecutor with a ReentrantCallbackGroup. Any new SDO
  access **must** take `can_lock`.
- Startup fails fast with `SystemExit` if `can0` is missing or down. You cannot start the
  driver in sim or CI without CAN hardware (a `vcan` would also need responding nodes).
- Parameter defaults in the code differ from the YAML: `lookahead_s` 0.15 vs 0.3,
  `enable_settle_s` 1.0 vs 0.3, and gripper `open_gap_mm` 30 vs 85. The YAML wins when you use
  `bringup.launch.py`. If you run a node bare, you get the code defaults.

## 7. Gazebo specifics (`lwa4p_gazebo`)

- `gazebo.launch.py` builds MoveIt config with `MoveItConfigsBuilder`, but **replaces
  `robot_description`** with `lwa4p_gazebo.urdf.xacro`, with XML comments stripped because
  gazebo_ros2_control chokes on them. It swaps in the sim `moveit_controllers.yaml`.
- Controllers (`joint_state_broadcaster`, `arm_controller`, `gripper_controller`,
  `finger_controller`) are spawned after `spawn_entity` exits. Wait for
  `You can start planning now!` and 4× `activated`.
- Joints are PID force-driven (gains in `config/ros2_controllers.yaml`, read by the plugin
  through `--params-file`). Do not switch to position teleporting: friction grasps rely on
  non-zero joint velocity. gazebo_ros2_control's own mimic joints do not work with PID, which
  is why `finger_mimic.py` exists.
- **MoveIt does not see Gazebo objects.** Scripts add collision boxes through
  `apply_planning_scene`, and those boxes **persist in move_group** after the script exits.
- To reset the world, restart the launch. There is no reset service in use.
- Every script in `scripts/` must be listed in `install(PROGRAMS …)` in `CMakeLists.txt`, have
  a `#!/usr/bin/env python3` shebang, and be executable. Scripts import `simple_moveit` as a
  sibling module, which works because they are installed side by side in `lib/lwa4p_gazebo`.
- `pick_tube.py` and `bin_pick.py` use `/gazebo/get_entity_state` only for sim-side checks and
  obstacle placement, and they degrade gracefully without it. Keep any new sim-only calls
  optional in the same way, so scripts still run on the real robot.
- `bin_pick.py` perception: one cloud → TF to `world` → crop to the bin → highest top surface →
  PCA for yaw and size → candidate grasps scored by finger-to-wall clearance. Bin constants
  are hard-coded and must match `worlds/lwa4p_table.world`.

## 8. Isaac Sim + MoveIt (`isaac/moveit/`)

- `run.sh` validates Isaac's Python, **rebuilds `lwa4p:humble`** (so compiled MTC stays in sync
  with `src/`), runs `prepare.py`, starts `simulation.py` on the **host** under Isaac's Python
  3.12, waits for `[LWA4P] READY`, then runs `moveit.launch.py` in the container with
  `--network host --ipc host`.
- `simulation.py` imports the URDF to USD (cached by SHA-256 under `.runtime/usd/<hash>/`),
  removes the ArticulationRoot from the separate fixed `camera_link` branch, and hosts a Humble
  `controller_manager` through `isaacsim.ros2.control` with `IsaacSimSystem`. Controller
  config: `isaac/moveit/controllers.yaml`. This depends on the **Sim 6.2 private API**
  (`_isaacsim_ros2_control.setup_cm`), so rerun `--test` after any Sim upgrade.
- `moveit.launch.py` also starts `verify_mtc` and `planning_scene.py` (the `isaac_workbench`
  box). It enables `ExecuteTaskSolutionCapability`.
- `--test` runs `verify_execution.py` (direct JTC, MoveIt arm, both fingers) and then
  `check_ptp.py` (MTC plus a check that a second call returns `BUSY`). Reports go to
  `.runtime/{execution,mtc}.json` and `.runtime/reports/`. The files in
  `isaac/moveit/verification/*.json` are **historical** results that are checked in, not
  outputs.
- Known issue: `move_group` (MoveIt 2.5.10) crashes with SIGSEGV on SIGINT shutdown. It
  happens after the work is done and is not fixed.
- Override with the env vars `ISAACSIM_PATH`, `ROS_DOMAIN_ID` (default 68),
  `LWA4P_ISAAC_CACHE` and `LWA4P_ISAAC_REPORT_DIR`.
- `isaac/lwa4p_cfg.py` is an **unvalidated** Isaac Lab 2.x template. `READY` puts the TCP at
  about (0.40, 0, 0.30), pointing down.

## 9. Build, run and verify

```bash
# On this host (Jazzy): always go through Docker. From the repo root:
docker build --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
  -f docker/Dockerfile -t lwa4p:humble .
```
- The image **COPYs `src/`** at build time and has no bind mount for the Gazebo, real or demo
  flows. **Rebuild after every source change**, or bind-mount `src/` yourself and rebuild with
  colcon inside the container.
- The build also acts as the smoke test. It imports `can`, `canopen`, the driver and the
  gripper modules, and runs `--show-args` on `gazebo.launch.py` and `real.launch.py`.
- Inside the image (or on a native Humble machine):
  `colcon build --symlink-install`. With symlink install, edits to Python, YAML and launch files
  apply immediately. **New** files need a rebuild, and C++ always needs one.
- `rosdep` must skip `warehouse_ros_mongo` (it is in `lwa4p_moveit_config/package.xml`, has no
  Humble binary, and is unused).
- `canopen==2.3.0` and `python-can==4.6.1` come from pip and are **not** declared in
  `package.xml` or `setup.py`.
- The MTC packages are only in the Dockerfile's apt list. Because `lwa4p_mtc_verification`
  needs `moveit_task_constructor_core`, a plain `colcon build` on a machine without MTC fails
  on that package. Use `--packages-skip lwa4p_mtc_verification` there.

**DDS and environment.** Use `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` everywhere (`move_group`
hangs on Zenoh in Humble). Domain IDs and settings differ by flow:
- Dockerfile default: `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`.
- Real robot docs: `ROS_DOMAIN_ID=43`, `--network host`.
- Isaac: `ROS_DOMAIN_ID=68`, `ROS_LOCALHOST_ONLY=0`.

Match these when you attach a second terminal.

**Tests.** There are **no unit or ament tests** in any package (no `test/` directories and no
`ament_lint`). Verification today consists of:
1. the Dockerfile's build-time import and `--show-args` checks;
2. `./isaac/moveit/run.sh --test` (needs a GPU and Isaac);
3. manual runs: `gazebo.launch.py gui:=false` + `go_to.py home`, then `pick_tube.py` and
   `bin_pick.py`.

The pure-logic code (`joint_motion_verification.cpp`, `Trajectory.sample` in the driver,
`find_top_object` and `grasp_options` in `bin_pick.py`, `crc16` in `wsg50.py`) is the easiest
place to add tests.

## 10. Safety rules for agents

- **Never** run anything that talks to `can0` or `can1` (driver, gripper node, `test_move`,
  `gripper_cmd`, `real.launch.py`, `~/enable`, `gripper_cmd home`) unless the user explicitly
  says the real robot is ready and someone is at the E-stop. Each of these moves hardware.
- Run only one process per CAN bus.
- Do not raise velocity or acceleration limits for the real arm without being asked (see §5).
- Treat `isaac/lwa4p_wsg50.urdf` as **generated**. Change the xacro in `lwa4p_description` (or
  the numbers in `make_isaac_urdf.py`), then regenerate with
  `python3 isaac/tools/make_isaac_urdf.py` in a sourced Humble environment. The generator
  reads the *Gazebo* xacro (arm + gripper + camera).

## 11. Change checklist: what must stay in sync

| If you change… | …also update |
|---|---|
| Joint names, groups or named states (SRDF) | `simple_moveit.py` (reads the SRDF at runtime for `named()`), `go_to.py` docs, `prepare.py` (assumes the gripper group/state layout), the MTC task (uses `arm`, `wsg50`, `gripper_tcp`, `tube_15mm`, `open`), `driver.yaml` `joint_names` |
| Gripper geometry or offset (`wsg50.urdf.xacro`) | `gripper.yaml` `finger_offset_mm`, `bin_pick.py` `FINGER_T`/`FINGER_W`/`TCP_ABOVE_FINGERTIP`, regenerate the Isaac URDF, the `0.0425` checks in `joint_motion_verification.cpp` and `prepare.py` `READY` |
| Camera pose | the `realsense_d435` call in `lwa4p_gazebo.urdf.xacro`, `isaac/README.md` §3, regenerate the Isaac URDF |
| World objects or bin positions | `lwa4p_table.world`, the constants and collision boxes in `bin_pick.py`/`pick_tube.py`, `isaac/README.md` §4 |
| Controller names | all three `moveit_controllers.yaml` files, both `ros2_controllers.yaml`, `isaac/moveit/controllers.yaml`, the driver `controller_name`, the gripper action name in `gripper_node.py` |
| Adding a Gazebo script | `install(PROGRAMS)` in `lwa4p_gazebo/CMakeLists.txt`, shebang, `chmod +x` |
| Adding Python deps to the driver | `package.xml` / `setup.py` **and** the Dockerfile pip line |
| Anything in `src/` used by Isaac | nothing manual: `run.sh` rebuilds the image |

## 12. Known rough edges (not bugs to fix unasked, but don't be surprised)

- Error strings in `gripper_cmd.py` and `simple_moveit.py` mention `arm_start.sh`, which does not
  exist. The real entry point is `real.launch.py`.
- `isaac/moveit/README.md` mentions `./isaac-setup/lwa4p-moveit.sh` in a "shared RoboCup
  workspace", which is not present here.
- The base `lwa4p.urdf.xacro` uses placeholder inertias (1 kg). Only the Isaac URDF has real
  masses.
- `kinematics.yaml` uses KDL with a 0.05 s timeout. There is no `ompl_planning.yaml` in
  `lwa4p_moveit_config/config`, so OMPL settings come from `moveit_configs_utils` defaults.
  Pilz Cartesian limits are present.
- The real camera has no driver or launch integration, and hand-eye calibration has not been
  done.
- `.gitignore` excludes `.vscode/`, `*.csv` and `*.log`. Driver traces and Isaac logs are never
  committed.

---

## 13. Planned architecture (RoboCup 2027)

The team's proposal is the diagram in [manipulation_architecture.txt](manipulation_architecture.txt)
(UTF-8, LF line endings). Nothing in the diagram is implemented yet, except what the "Today"
column below says.

### How to read each layer, given the team decisions in §1

| Layer in the diagram | Owner | Today | Notes for the plan |
|---|---|---|---|
| Compute split (GPU / CPU cores 4–5 / cores 6–7, RT priority) | — | nothing | The GPU part concerns the perception team. Core isolation and RT priority **do not apply** to the real arm: its driver is a 20 Hz Python loop sending Profile Position targets over SDO. |
| Perception and world model (VLM, then a TF2 3D pose) | other team | `bin_pick.py` has a simple point-cloud detector (sim only) | Contract: `PoseStamped` in a TF frame. VLM latency is in seconds, so use it for task-level detection only, never closed-loop. Grasping thin objects (tubes) needs **orientation and size**, not just a depth-lookup point. Ask for them, or fit them from the cloud locally as `bin_pick.py` does. |
| Task supervisor (BehaviorTree.CPP) | this team | nothing | See the split with MTC below. |
| MTC | this team | only `lwa4p_mtc_verification`, a sim test utility and not a skill | MTC tasks need `move_group/ExecuteTaskSolutionCapability`. Only `isaac/moveit/moveit.launch.py` enables it. **`real.launch.py` and `gazebo.launch.py` do not.** Add it before running MTC skills there. |
| OMPL (RRTConnect) + Pilz LIN/PTP | this team | OMPL via `moveit_configs_utils` defaults. Isaac forces `ompl` only. | `pilz_cartesian_limits.yaml` allows 1.0 m/s, far beyond the real arm (0.5 rad/s joints). Set realistic values before using LIN on hardware. Check which pipelines `move_group` actually loaded before relying on Pilz. |
| Kinematics (pick_ik / TRAC-IK) | this team | KDL (`kinematics.yaml`) | That one file applies to every backend. pick_ik has Humble binaries; TRAC-IK may need a source build. |
| Ruckig smoothing | this team | not configured | Compatible with the real driver, which resamples any dense trajectory. Keep normal time parameterization in front of it. |
| MoveIt Servo / "velocity bypass" | — | — | **Not possible on the real arm.** The driver serves only `FollowJointTrajectory` goals in Profile Position mode and has no velocity or streaming input. At most, treat it as sim-only research. |
| `ros2_control` (controller_manager, JTC, forward_command, joint_state_broadcaster) | — | Gazebo, Isaac, mock | **Simulation only.** On the real robot, `lwa4p_driver` serves `arm_controller/follow_joint_trajectory` and `gripper_node` serves `gripper_controller/gripper_cmd`. There is no controller_manager, no 100–250 Hz loop, and no velocity controller. |
| `ros2_canopen` / Lely, 1 Mbit/s, sync PDO | — | — | **Superseded by the driver decision.** The real bus is 500 kbit/s, SDO only, PP mode, 20 Hz, through `lwa4p_driver` (see §6). |
| Hardware: LWA 4P, E-stop | — | as §6 | The diagram **omits the WSG 50 gripper**, which sits on `can1`, speaks its own protocol (not CiA 402), and is served by `gripper_node`. Every skill involving grasping depends on it. |

### Design rules for new manipulation code

1. **Keep sim and real apart at the action interface.** Skills and the behaviour tree talk to
   MoveIt and MTC, plus `/joint_states`, TF and `gripper/state`. Never call `controller_manager`
   services (e.g. `switch_controller`), and never require Gazebo or Isaac services. Sim-only
   helpers must stay optional, as in `pick_tube.py`.
2. **Abstract the gripper.** Skills use SRDF named states (`open`, `tube_15mm`, `closed`) or one
   "grasp(width, force)" call. Never use raw joint names: the real robot and Gazebo use
   `gripper_joint`, Isaac uses two finger joints (§3). Grasp verification needs two
   implementations behind one interface:
   - **real:** `gripper_node` reports `HOLDING`, and `gripper/state` carries the force in
     `effort`;
   - **sim:** the measured gap between the fingers, as in `bin_pick.py`.
3. **Plan within the real driver's limits, even in simulation.** The simulation controllers
   accept trajectories the driver rejects:
   - segments faster than about 30°/s;
   - a start more than 2° from the current position;
   - targets within 3° of the drive limits;
   - tracking error above 8°.

   Keep velocity and acceleration scaling within `joint_limits.yaml`. Consider a shared
   pre-execution check, so failures show up in simulation first. Test each new skill once on the
   real arm at low speed.
4. **Split responsibilities between the behaviour tree and MTC.** MTC owns the motion
   sequence (approach → grasp → lift → transfer → place, via its Pick/Place containers) and
   backtracks over IK and grasp candidates. The behaviour tree works at skill level: requesting
   perception, starting an MTC task, verifying the grasp, retrying or re-detecting, recovering,
   and coordinating with the navigation team. Do not duplicate the motion stages as
   behaviour-tree nodes.
5. **Expose skills as ROS actions** (a "skill server"). The robot-level behaviour tree, owned by
   another team and possibly Nav2's BT.CPP, can then call manipulation as one subtree.
   Interfaces belong in a dedicated `*_interfaces` package.
6. **Planning scene.** Perception results must become MoveIt collision objects, because MoveIt
   sees nothing on its own. Objects persist in `move_group` until you remove them, so own their
   lifecycle (add, attach, detach, remove) in the skill server.
7. **Frames.** Manipulation plans in `world` (= the arm base, §4). When the arm is mounted on
   the mobile base, `world` must hang off the base TF tree, the base must stay still during
   execution, and the camera pose needs hand-eye calibration (still a placeholder).
8. **Platform.** ROS 2 Humble reaches end of life in May 2027, around the competition, and
   Gazebo Classic is already EOL. Do not add new Humble-only dependencies casually.

**Naming.** The `lwa4p_` prefix is only for hardware-specific packages (description, driver,
MoveIt config, sim models). New layers above the action interface are hardware-agnostic and
**must not** use `lwa4p_` or other robot-specific names: no `lwa4p`, `wsg`, `schunk` or
`erb` in package names, node names, action names or BT node IDs. Robot specifics (group names,
frames, named states, gripper widths and forces) come from parameters or YAML, so the skills
can move to another arm or gripper without renaming.

Suggested homes for new code, which **do not exist yet**:
- `manipulation_interfaces`: skill action definitions;
- `manipulation_skills`: skill server + MTC tasks, in C++ (you can reuse the
  `.clang-format` from `lwa4p_mtc_verification`);
- `manipulation_bt`: BT.CPP nodes and XML trees.

Agree on the names with the team before creating them.
