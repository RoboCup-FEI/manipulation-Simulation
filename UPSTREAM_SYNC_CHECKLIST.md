# Checklist: bringing `RoboCup-FEI/manipulation-Simulation` into `Tomas-Jurov/lwa4p_driver` main

Analysis date: 2026-11-09. Local `main` = `5852ce9`, upstream `main` = `c07db5d`.

## Situation

- **The histories are unrelated.** `git merge-base` finds nothing. Upstream rewrote its history in
  `ba3ac94` ("student-only repo"), which removed the example solutions and `isaac/` from every
  commit. Local `c8086cd` matches upstream `18283e7` (PR #1, docker setup). The only difference
  is `bin_pick.py` and `pick_tube.py`, which the rewrite removed. That commit is the effective
  fork point.
- Since the fork, **local has ~30 commits** (diagnostics, Isaac, MTC verification, calibration
  poses, FTS + mesh WSG 50) and **upstream has 10** (Jazzy/Harmonic, FTM 75 driver, wrist camera,
  workspace obstacles, lab world, tool length fix).
- **16 files changed on both sides** (conflict hotspots): `README.md`, `docker/Dockerfile`,
  `lwa4p.urdf.xacro`, `wsg50.urdf.xacro`, `driver_node.py`, `ds402.py`, `gripper_node.py`,
  `lwa4p_driver/package.xml`, `lwa4p_gazebo/README.md`, `lwa4p_gazebo.urdf.xacro`,
  `lwa4p.srdf`, `simple_moveit.py`, and 4 files under `isaac/`.
- Upstream refs were fetched into this clone as `refs/upstream/main` and
  `refs/upstream/workspace-obstacles`. No remote or branch was added.

---

## 0. Decisions to make first (blocking)

- [ ] **Agree the scope with Tomas Jurov.** Upstream is now a *student-only* repo. Its
      `.gitignore` keeps these out on purpose: example solutions / task scripts
      (`src/lwa4p_gazebo/scripts/*.py` except `finger_mimic.py`, `gz_cloud_frame.py`),
      calibration tools, OctoMap configs and `isaac/`. Decide which of the local additions belong upstream:

      | Local addition | Upstream policy | Suggestion |
      |---|---|---|
      | Driver/gripper diagnostics, effort, `/diagnostics`, `~/diagnose` | fits | **upstream** |
      | FTS sensor + mesh WSG 50 model | conflicts with FTM 75 model (§2) | **upstream after reconciliation** |
      | `lwa4p_calibration_poses` + `_interfaces` | "calibration tools kept local" | ask; probably fork only |
      | `lwa4p_mtc_verification` | not covered | ask; fork only if Isaac stays local |
      | `isaac/` (model, MoveIt bridge, verification) | ignored upstream | **fork only** |
      | `bin_pick.py`, `pick_tube.py`, `go_to.py`, `simple_moveit.py` | example solutions, ignored | **fork only** |
      | `AGENTS.md`, `manipulation_architecture.txt`, `manipulation_implementation_proposal.md` | team-internal | **fork only** |
      | `LICENSE` line "Copyright (c) 2026 RoboCup-FEI" | owner's call | ask |
      | `.dockerignore` | harmless | upstream |

- [ ] **Decide which gripper/sensor model is the reference** (see §2). Both sides model the
      same physical sensor (SCHUNK FTS-LWA = FTM 75) with different numbers.
- [ ] Decide what to do with the stale fork branches. Each is 27–30 commits behind `main`:
      `calibration-poses-humble` (3 ahead, includes a duplicate "workspace obstacles" commit),
      `calibration-poses-jazzy` (1 ahead, Jazzy port of calibration poses that `main` doesn't have),
      `joint-jog-teleop` (1 ahead, gamepad/keyboard jogging). Merge, port or delete each.

## 1. Git mechanics

- [ ] **Don't** merge the two `main`s with `--allow-unrelated-histories`. Every shared file
      would show as an add/add conflict, and the purged example solutions would come back
      into upstream history.
- [ ] Create a work branch from `upstream/main`. Port the chosen pieces as new commits, using
      `git checkout <local-commit> -- <paths>` or a patch from `git diff c8086cd main -- <paths>`
      applied with 3-way merge.
- [ ] Split the work into small PRs to `Tomas-Jurov/lwa4p_driver`, e.g. (a) driver diagnostics,
      (b) tool model reconciliation, (c) optional packages. Don't open one big PR.
- [ ] Afterwards, rebase the fork on the new upstream so both repos share history again. The
      fork's own files should live in fork-only commits on top, or in a separate repo/overlay
      that uses upstream as a dependency.
- [ ] Check that no file under the upstream "local only" `.gitignore` patterns is
      force-added (`git add -f`) by mistake.

## 2. Robot model: the critical conflict

The two tool chains are incompatible:

| | Upstream (`c07db5d`) | Local (`5852ce9`) |
|---|---|---|
| `arm_tool0` from `arm_6_link` | z 0.084, rpy 0 (top of spigot) | z 0.0751, **yaw π** (Bakyta diploma) |
| sensor | `ftm75` box model, +52.75 mm, links `ftm_link`, `ftm_sensor_frame`, `ftm_tool0` | `fts_lwa` mesh, +55.35 mm, links `fts_lwa_base_link`, `fts_lwa_tool_link` |
| gripper | box model; `gripper_adapter_link`, `gripper_body_link`, `gripper_finger_*_link` | meshes; `gripper_connector_link`, `gripper_base_link`, `gripper_jaw_*_link`, `gripper_finger_*_link` |
| `arm_6_link` → `gripper_tcp` | 293.75 mm | 289.45 mm |
| `arm_6_link` → fingertips | **308.75 mm** (FTM table-touch, 2026-11-07, ±3 mm) | **298.95 mm** (ruler, 2026-11-02) |
| TCP above fingertips | 15 mm | 9.5 mm |
| wrist camera | `camera_link` on `arm_tool0` from `config/camera_calibration.yaml` | none in description (Isaac only) |
| xacro args | `gripper`, `gripper_yaw`, `ftm`, `wrist_camera`, `workspace_obstacles` | `gripper`, `gripper_yaw` |

- [ ] **Measure the tool length once more and agree on one value.** The two sides differ by
      ~10 mm at the fingertips. That is enough to crash into the table or miss grasps.
- [ ] **Keep `arm_tool0` where upstream has it, or re-express the hand-eye calibration.**
      `camera_calibration.yaml` is `arm_tool0 → camera_link`. With local's yaw π and −8.9 mm
      shift, the camera would turn 180° about the tool axis and the point cloud would be wrong.
      Either keep `arm_tool0` at z 0.084, rpy 0 and put the Bakyta offsets below it, or
      transform the calibration and regenerate it with `hand_eye_to_urdf`.
- [ ] Use one sensor model. If the FTS mesh is kept, keep the upstream frame names
      (`ftm_link`, `ftm_sensor_frame`, `ftm_tool0`) or update `ftm.yaml` `frame_id`,
      `ftm_node.py` gravity compensation and the SRDF. `ftm_node` needs `ftm_sensor_frame`
      to exist in TF.
- [ ] Keep the upstream `ftm:=false` path working with the mesh gripper. The gripper must
      attach to `arm_tool0` when the sensor is off.
- [ ] Pick one TCP convention (15 mm vs 9.5 mm above the tips) and update scripts and docs.
- [ ] Rename links in only one place. Everything that references them must follow:
      SRDF ACM, `lwa4p_gazebo.urdf.xacro` (`colour`, `finger_surface`), gz bridge/camera,
      `finger_mimic.py`, scripts, `make_isaac_urdf.py`, `manipulation_implementation_proposal.md`
      (still names `gripper_body_link`).
- [ ] Take the upstream base column (cylinder + `workspace_obstacles` switch) and the upstream
      **binary STL arm meshes with normals**. They fix the black arm in Jazzy RViz. Regenerate
      `isaac/meshes/*` from them.
- [ ] `meshes/fts_lwa/fts_lwa.stl` is **3.5 MB**. Decimate the visual mesh; the collision mesh
      is already small.
- [ ] Collision meshes are scaled about the link origin (FTS ×1.1, WSG housing ×1.05). The
      FTS origin is on the gripper face, so the enlarged mesh grows toward the arm and away
      from the axis unevenly. Check it, or use a padded primitive.
- [ ] **Mesh licensing.** The WSG meshes come from `nalt/wsg50-ros-pkg`, which has no license
      file. The FTS mesh is of unknown origin; the diploma author's permission covers his
      xacro, not necessarily the CAD. Clear this before publishing in an MIT repo, or replace
      them with upstream's primitives.

## 3. MoveIt config

- [ ] Delete local `lwa4p_moveit_config/config/workspace_obstacles.urdf.xacro` and its include
      in `lwa4p_moveit_config/config/lwa4p.urdf.xacro`. Upstream replaced it with
      `lwa4p_description/urdf/workspace_obstacles.urdf.xacro`. That version is newer: re-measured,
      with mesh racks, holder and low layer, dispenser bracket and clamp, and without the
      "yellow robot". If both are included, the links are defined twice and xacro fails.
- [ ] Regenerate the SRDF disabled-collision matrix once, against the final link set:
      upstream obstacles + camera + chosen sensor/gripper links. Neither current SRDF is valid
      for the merged model.
- [ ] Keep upstream changes: joint-limit margin `radians(4)` (inside the driver's 3° margin),
      `allow_failure: true` for the gripper, `moveit_ros_perception` dependency, no
      `warehouse_ros_mongo`.

## 4. `lwa4p_driver`

- [ ] `ds402.py`: keep **both** local additions (error names, history, torque, temperature)
      and upstream's two-direction `commutate()`.
- [ ] Error-code text exists twice: local `ds402.ERROR_NAMES` and upstream
      `driver_node.ERROR_TEXT`. Merge them into one table in `ds402`. Upstream's `0x7122`
      (commutation error) is missing locally.
- [ ] `driver_node.py`: keep upstream's `commutation_check` parameter and `error_text` in
      `step()` together with local's `trip(why, bad)` / `joint_report`. Both sides rewrote the
      same `step()` and `srv_enable` blocks, so the merge needs hand work.
- [ ] `gripper_node.py`: combine upstream's `try/except → SystemExit` startup with local's
      diagnostics and status-log timers.
- [ ] `package.xml`: take the union of both sides (`diagnostic_msgs` from local; `std_msgs`,
      `geometry_msgs`, `tf2_ros`, `python3-scipy` from upstream).
- [ ] Keep upstream `setup.py` (`CONSOLE_SCRIPTS`, optional local tools, `ftm` entry point)
      and the `ftm` node in `bringup.launch.py`.
- [ ] On the real robot, check bus load: the gripper's temperature/diagnostic polling, the FTM
      polling at 50 Hz and the WSG all share **can1**.
- [ ] The control loop now does one extra SDO read (torque) per cycle at 20 Hz. Measure the
      real loop period after the merge.
- [ ] Small fix: `self.q =[` → `self.q = [` (`driver_node.py`).

## 5. Gazebo / simulation

- [ ] Upstream runs one codebase on **Humble (Gazebo Classic)** and **Jazzy (Gazebo Harmonic)**
      and picks the simulator from `ROS_DISTRO`. Every ported model change must work in both
      (`lwa4p_gazebo.urdf.xacro`, `sim:=classic|gz`, `ros2_controllers_gz.yaml`).
- [ ] Map the local link renames into upstream's `lwa4p_gazebo.urdf.xacro`: colours, finger
      friction surfaces, FTM colour, camera sensor.
- [ ] Check grasping with the mesh fingers in both simulators. Upstream already notes that rack
      tubes do not hold yet.
- [ ] Don't re-add `bin_pick.py`, `pick_tube.py`, `go_to.py`, `simple_moveit.py` upstream
      (§0). Upstream `CMakeLists.txt` already installs any local `scripts/*.py`, so they keep
      working in the fork.

## 6. New C++ packages (only if they go upstream)

- [ ] Make one source build on Humble **and** Jazzy. The Jazzy changes are currently only on
      `calibration-poses-jazzy`, not on `main`.
- [ ] MTC is released on both distros (Humble 0.1.3, Jazzy 0.1.8, checked in rosdistro), so
      `rosdep install` works. Still build `lwa4p_mtc_verification` against both.
- [ ] Add MTC and `nlohmann-json3-dev` to **`docker/Dockerfile.jazzy`** as well. Locally they
      were only added to the Humble Dockerfile. Merge upstream's `moveit-ros-perception` line
      into the Humble Dockerfile.
- [ ] Run `colcon test` (gtests in `lwa4p_calibration_poses`) in both containers.
- [ ] Check `calibration_poses.yaml` against the new joint limits (±(170−4)°, axis 3 ±(155−4)°).

## 7. Isaac (if it stays in the fork)

- [ ] Regenerate `isaac/lwa4p_wsg50.urdf` and meshes with `isaac/tools/make_isaac_urdf.py`
      after §2. It hard-codes link names: `arm_podest_link`, `fts_lwa_base_link`,
      `gripper_*`, `camera_link`.
- [ ] Re-run `isaac/moveit/verify_execution.py` and refresh `verification/*.json`. The tool
      length and TCP change affect those results.

## 8. Docs and meta

- [ ] `README.md`: start from upstream's (Humble + Jazzy, apt lists, troubleshooting). Add only
      the diagnostics/sensor sections that go upstream.
- [ ] `lwa4p_gazebo/README.md`, `docker/README.md`: same approach. Upstream removed the
      example-solution commands.
- [ ] `AGENTS.md` has machine-specific paths (`/heavy_stuff/robocup_ws`) and team decisions.
      Keep it in the fork only, and update its "Jazzy migration in progress" note once upstream
      Jazzy support is in.
- [ ] Maintainer emails: new packages use `dmitriiognev@users.noreply.github.com`. Agree with
      upstream (`tomas.jurov@stuba.sk`).
- [ ] Upstream `camera_calibration.yaml` has `source: /home/tomas/...`. That's a harmless
      absolute path, but worth mentioning to Tomas.

## 9. Verification before opening the PR(s)

- [ ] `rosdep install --from-paths src -iy` and `colcon build` in the Humble **and** Jazzy
      Docker images.
- [ ] `xacro` + `check_urdf` for all arg combinations: `gripper`, `ftm`, `wrist_camera`,
      `workspace_obstacles` set to true/false.
- [ ] RViz: model, obstacles and camera point cloud line up with the real table. This is the
      test for the `arm_tool0`/calibration question in §2.
- [ ] MoveIt plan + execute in Gazebo Classic (Humble) and Gazebo Harmonic (Jazzy).
- [ ] Real robot: driver enable (commutation check), `/diagnostics`, `~/diagnose`, gripper,
      `ftm` node; FTM table-touch test of the final tool length.
