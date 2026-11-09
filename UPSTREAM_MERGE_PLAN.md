# Plan: merge `Tomas-Jurov/lwa4p_driver` main into a new branch of `manipulation-Simulation`

Prepared 2026-11-09. Fork `main` = `5852ce9`, upstream `main` = `c07db5d`. All conflict lists
below come from a dry run: `git merge-tree --merge-base=18283e7 main upstream/main`. No branch or
working-tree file was changed.

---

## 1. Strategy

**Branch name:** `upstream-main-merge`. Avoid `upstream/main`: git would confuse it with the
remote-tracking branch `upstream/main` and with the `refs/upstream/main` fetched earlier.

**Approach: a real merge commit, made possible by a temporary graft.**

The two histories are unrelated because upstream rewrote its history in `ba3ac94`. The fork's
`c8086cd` and upstream's `18283e7` are the same point: upstream PR #1, docker setup. All 7
commits up to that point match pairwise in author, dates and message. Two things changed the
hashes: the rewrite dropped GitHub's PGP signature from the root commit (`71e126b` vs
`df69258`, same tree), and it removed `bin_pick.py` and `pick_tube.py` from every commit from
"lwa4p ros2 package" on. Using `18283e7` as the base, the fork's two scripts count as
"added by the fork", so the merge keeps them. A temporary
`git replace --graft` tells git that. `git merge` then does a normal 3-way merge with `18283e7` as
the base, so `--ours`/`--theirs`, conflict stages and mergetool all work. Remove the graft after
the merge commit. The merge commit itself has `upstream/main` as its second parent, so **every
later upstream sync is a plain `git merge upstream/main`**.

Rejected alternatives:
- `--allow-unrelated-histories` with no base: every shared file becomes an add/add conflict.
- Cherry-picking upstream's 7 non-merge commits: works, but upstream never becomes an ancestor,
  so the same problem comes back at the next sync.

This branch updates the **fork**. Sending fork work to Tomas is a separate step (§7). That
branch must start from `upstream/main`, because upstream purged `isaac/` and the example
solutions from its history on purpose.

**Two-step resolution** keeps the merge reviewable:
1. **Merge commit:** take **upstream's robot model as-is** (the frames, FTM 75, camera,
   obstacles, and tool length that were measured on 2026-11-07), keep all fork-only content, and
   fix the silent breakages. The merge commit is consistent and close to what upstream tested.
2. **Follow-up commits on the same branch:** re-apply the fork's mesh gripper and FTS sensor on
   top of upstream's frame tree, then update the fork-only consumers (Isaac, scripts, docs).

## 2. Decisions to make before Phase 2 (Phase 1 doesn't depend on them)

| # | Decision | Recommended default | Why |
|---|---|---|---|
| D1 | Reference tool chain | **Upstream frames + upstream tool length.** Use the fork's meshes only as visuals/collision inside them | The camera hand-eye calibration is `arm_tool0 → camera_link` with `arm_tool0` at z 0.084, rpy 0. `ftm_node` needs `ftm_sensor_frame`. Upstream's length comes from an FTM table-touch test (±3 mm); the fork's from a ruler |
| D2 | Tool length if D1 is disputed | Re-run the FTM table-touch test before Phase 2 | The two sides differ by ~10 mm at the fingertips (308.75 vs 298.95 mm from `arm_6_link`) |
| D3 | TCP convention | Upstream: 15 mm above the fingertips | Upstream README, MoveIt and the lab world already use it. The fork used 9.5 mm |
| D4 | Fork-only content | Keep all of it on this branch (`isaac/`, example scripts, calibration, MTC, team docs) | This branch becomes the fork's `main` |
| D5 | Mesh licensing | Keep the meshes in the fork. Don't send them upstream until the license is cleared | WSG meshes: no license in `nalt/wsg50-ros-pkg`. FTS mesh: origin unknown |

## 3. Conflict inventory (dry run)

### 3a. Textual conflicts (12 files)

| File | Type | Fork side | Upstream side | Phase 1 resolution | Follow-up |
|---|---|---|---|---|---|
| `docker/Dockerfile` | content, 1 hunk | `+ moveit-task-constructor-{core,capabilities,visualization}`, `nlohmann-json3-dev` | `+ moveit-ros-perception` | **union**, keep all 4 apt lines | Add the same MTC + `nlohmann-json3-dev` to `docker/Dockerfile.jazzy` (`ros-jazzy-…`) |
| `src/lwa4p_driver/lwa4p_driver/driver_node.py` | content, 2 hunks | `step()`: `bad` list → `trip(why, bad)` → `joint_report`; new `srv_diagnose` | `step()`: per-joint loop with `error_text(last_error)` | Hunk 1: **fork's** `bad`/`trip`. Hunk 2: **keep `srv_diagnose`** with one `return res` | Move upstream's `ERROR_TEXT` hints (`0x3222` "motor supply too weak / sagging", `0x7122` commutation error, "redo enable") into `ds402.ERROR_NAMES`. Delete `ERROR_TEXT`/`error_text` once unused. Fix `self.q =[` |
| `src/lwa4p_description/urdf/lwa4p.urdf.xacro` | content, 2 hunks + **silent hunk** | `arm_tool0` z 0.0751 yaw π, `fts_lwa`, `wsg_50_xacro` | camera, `ftm75`, `wsg50` on `ftm_tool0`, `workspace_obstacles`, base cylinder, 4° margin | **`git checkout --theirs`** for the whole file. Without that, the auto-merged `arm_tool0` (0.0751, π) stays and turns the camera by 180° (see 3b) | Phase 2: put FTS + mesh WSG back in upstream's frames |
| `src/lwa4p_description/urdf/wsg50.urdf.xacro` | content, both sides rewrote it | mesh model, macro `wsg_50_xacro`, links `gripper_connector/base/jaw_*` | box model, macro `wsg50`, links `gripper_adapter/body/finger_*`, finger_l −10 mm | **`--theirs`** | Phase 2 |
| `src/lwa4p_moveit_config/config/lwa4p.srdf` | content, 2 hunks | ACM for FTS + mesh links + old obstacles | ACM for `ftm_link`, camera, box gripper + new obstacles | **`--theirs`** | Phase 2: regenerate the ACM with the Setup Assistant for the final link set |
| `src/lwa4p_gazebo/urdf/lwa4p_gazebo.urdf.xacro` | content, 1 hunk | colours for `fts_lwa_base_link`, `gripper_connector/base/jaw_*` | colours for `ftm_link`, `gripper_adapter/body_link` (+ gz/classic, camera) | **`--theirs`** | Phase 2: rename to the final link names |
| `src/lwa4p_gazebo/README.md` | content, 1 hunk | `bin_pick.py` hint, reach 0.55 m, TCP 9.5 mm | reach 0.5 m (FTM), TCP 15 mm, gripper gap | **`--theirs`** | Phase 3: re-add the `bin_pick.py` note (fork only) |
| `isaac/README.md` | modify/delete | updated | deleted (ignored upstream) | **keep fork's** (`git add`) | Phase 3: link names, tool length |
| `isaac/lwa4p_cfg.py` | modify/delete | updated | deleted | **keep fork's** | Phase 3 |
| `isaac/lwa4p_wsg50.urdf` | modify/delete | regenerated for FTS + mesh WSG | deleted | **keep fork's** | Phase 3: regenerate with `make_isaac_urdf.py` |
| `isaac/tools/make_isaac_urdf.py` | modify/delete | link-name tables for FTS + mesh WSG | deleted | **keep fork's** | Phase 3: link names in `FTS_MASS`/mass tables, camera now from description |
| `src/lwa4p_gazebo/scripts/simple_moveit.py` | modify/delete | updated finger link names/sizes | deleted (example solution) | **keep fork's** | Phase 3: link names after Phase 2 |

### 3b. Silent problems (auto-merged, but wrong)

These won't show as conflicts. Each needs an explicit fix in the merge commit.

| # | What git does | Effect | Fix in the merge commit |
|---|---|---|---|
| S1 | Takes the fork's `arm_tool0_joint` (z 0.0751, rpy 0 0 π) into upstream's xacro | Wrist camera (`camera_calibration.yaml`, parent `arm_tool0`) rotated 180° and shifted 8.9 mm. Wrong point cloud, wrong camera collision box | `git checkout --theirs` for the whole `lwa4p.urdf.xacro` (above) |
| S2 | Keeps the fork's include of `lwa4p_moveit_config/config/workspace_obstacles.urdf.xacro` **and** adds upstream's `lwa4p_description/urdf/workspace_obstacles.urdf.xacro` | Same link names (`obstacle_table`, `obstacle_tprofile_*`, …) defined twice. URDF parse error, so MoveIt doesn't start | `git rm src/lwa4p_moveit_config/config/workspace_obstacles.urdf.xacro` and delete its `<xacro:include>` in `lwa4p_moveit_config/config/lwa4p.urdf.xacro` |
| S3 | Deletes `isaac/meshes/arm_{1..6,base}_link.stl` (unchanged in the fork, deleted upstream) | Isaac URDF points to missing meshes | `git checkout main -- isaac/meshes`. Phase 3: regenerate from upstream's new binary STLs with normals |
| S4 | Deletes `src/lwa4p_gazebo/scripts/go_to.py` | Example script lost | `git checkout main -- src/lwa4p_gazebo/scripts/go_to.py` |
| S5 | Adds upstream's "local only" block to `.gitignore` (`isaac/`, `src/lwa4p_gazebo/scripts/*.py`) | Tracked files stay tracked, but **new** Isaac files or scripts are silently ignored in the fork | Delete the `isaac/` line and the `scripts/*.py` + `!…` lines in the fork. Keep the calibration-board / probe patterns |
| S6 | Merges `README.md` without conflict | Both Isaac (fork) and Jazzy (upstream) sections exist, but reach/TCP/sensor wording disagrees | Read it once; fix in Phase 3 |
| S7 | `AGENTS.md` unchanged | Says "Jazzy migration in progress" and lists the fork's link names | Phase 3 |

### 3c. Merged cleanly, checked by hand

- `ds402.py`: fork diagnostics + upstream two-direction `commutate()`. OK.
- `gripper_node.py`: upstream's `try/except → SystemExit` wraps the init, and the fork's
  diagnostics and status timers follow it. OK.
- `driver_node.py` `srv_enable`: upstream's `commutation_check` and the fork's `joint_report`
  messages are both present. OK.
- `lwa4p_driver/package.xml` (union of deps), `setup.py`, `bringup.launch.py` (`ftm` node),
  `lwa4p_gazebo` launch/CMake/package (Humble + Jazzy), MoveIt `joint_limits`,
  `moveit_controllers` (`allow_failure: true`), `moveit_config/package.xml`. All taken from
  upstream, no fork changes.
- Arm meshes `meshes/lwa4p/*.stl`: upstream's binary STLs with normals (they fix the black arm in
  Jazzy RViz).
- `bin_pick.py`, `pick_tube.py`: not in the merge base, so the fork's copies are kept.

## 4. Execution

### Phase 0: preparation
- [ ] Pause OneDrive sync for this folder during the merge. Syncing `.git` during big
      operations can corrupt the repo.
- [ ] Clean working tree. Move `UPSTREAM_SYNC_CHECKLIST.md` and this file out of the repo or
      commit them separately.
- [ ] Replace the earlier ad-hoc fetch with a proper remote:

```bash
git update-ref -d refs/upstream/main
```
```bash
git update-ref -d refs/upstream/workspace-obstacles
```
```bash
git remote add upstream https://github.com/Tomas-Jurov/lwa4p_driver.git
```
```bash
git fetch upstream
```

### Phase 1: merge commit
```bash
git switch -c upstream-main-merge main
```
```bash
git replace --graft c8086cd $(git rev-parse c8086cd^1 c8086cd^2) 18283e7
```
```bash
git merge-base HEAD upstream/main
```
The last command must print `18283e7…`. Then:
```bash
git merge --no-ff --no-commit upstream/main
```
Resolve the conflicts (§3a column "Phase 1 resolution"):
```bash
git checkout --theirs -- src/lwa4p_description/urdf/lwa4p.urdf.xacro src/lwa4p_description/urdf/wsg50.urdf.xacro src/lwa4p_moveit_config/config/lwa4p.srdf src/lwa4p_gazebo/urdf/lwa4p_gazebo.urdf.xacro src/lwa4p_gazebo/README.md
```
```bash
git add isaac/README.md isaac/lwa4p_cfg.py isaac/lwa4p_wsg50.urdf isaac/tools/make_isaac_urdf.py src/lwa4p_gazebo/scripts/simple_moveit.py
```
- [ ] Edit `docker/Dockerfile` (union) and `driver_node.py` (2 hunks) by hand.
- [ ] S2–S5 from §3b:
```bash
git rm src/lwa4p_moveit_config/config/workspace_obstacles.urdf.xacro
```
```bash
git checkout main -- isaac/meshes src/lwa4p_gazebo/scripts/go_to.py
```
  Then edit `lwa4p_moveit_config/config/lwa4p.urdf.xacro` (remove the include) and `.gitignore`.
- [ ] Check that no conflict markers are left, then commit and drop the graft:
```bash
git grep -nE "^(<<<<<<<|=======|>>>>>>>)( |$)"
```
```bash
git add -A && git commit
```
```bash
git replace -d c8086cd
```
```bash
git merge-base upstream-main-merge upstream/main
```
The last command must print upstream `c07db5d`.

Commit message: `Merge Tomas-Jurov/lwa4p_driver main (Jazzy, FTM 75, wrist camera, workspace obstacles)`.
List S1–S5 in the body.

Fork features that are temporarily inactive after Phase 1: the mesh WSG 50 and FTS visuals.
The fork-only scripts and Isaac files still use the old link names until Phase 3.

### Phase 2: model reconciliation (after D1–D3)
One commit per item:
- [ ] **FTS mesh as the visual of `ftm_link`.** Keep upstream's frame names (`ftm_link`,
      `ftm_sensor_frame`, `ftm_tool0`) and upstream's 52.75 mm. Place the FTS mesh so its faces
      match those frames; check the 55.35 mm vs 52.75 mm difference against the real sensor.
      Decimate `fts_lwa.stl` (3.5 MB). Keep the `ftm:=false` path working.
- [ ] **Mesh WSG 50 with upstream's dimensions.** Keep upstream's joint names (`gripper_joint`,
      `gripper_finger_{left,right}_joint` mimic ±0.5) and `gripper_tcp` 15 mm above the tips.
      Use the fork's meshes. Pick one set of link names and change it in one commit only. Keep
      the upstream macro signature `wsg50 parent yaw`, or update its single caller.
- [ ] Check that upstream's Jazzy `gz_ros2_control` handles the fork's finger axes. It reads the
      mimic tags, and the fork uses `axis -sign` + `yaw π`. Test grasping in Gazebo Classic and
      Harmonic.
- [ ] `lwa4p_gazebo.urdf.xacro`: colours / `finger_surface` for the final link names.
- [ ] Regenerate the SRDF disabled-collision matrix: arm + `camera_link` + sensor + gripper +
      upstream obstacles + `arm_podest_link` cylinder.
- [ ] `ds402.ERROR_NAMES` consolidation and `driver_node.py` clean-up (from §3a).
- [ ] `docker/Dockerfile.jazzy`: MTC + `nlohmann-json3-dev`.

### Phase 3: fork-only consumers and docs
- [ ] `isaac/tools/make_isaac_urdf.py`: link-name/mass tables. The camera now comes from the
      description, so don't add it twice. Regenerate `isaac/lwa4p_wsg50.urdf` and
      `isaac/meshes/*` from upstream's binary STLs.
- [ ] Re-run `isaac/moveit/verify_execution.py` and refresh `isaac/moveit/verification/*.json`.
      The tool length and TCP changed.
- [ ] `bin_pick.py`, `pick_tube.py`, `go_to.py`, `simple_moveit.py`: link names, finger sizes,
      TCP offset, reach (the tool is now longer).
- [ ] `lwa4p_calibration_poses`: check `config/calibration_poses.yaml` against the new joint
      limits (±166°, axis 3 ±151°). Bring in the Jazzy changes from `calibration-poses-jazzy` so
      one source builds on both distros.
- [ ] `lwa4p_mtc_verification`: `ik_frame gripper_tcp` stays valid. Build on Humble (MTC 0.1.3)
      and Jazzy (0.1.8).
- [ ] Docs: `README.md` (S6), `lwa4p_gazebo/README.md` (`bin_pick` note), `isaac/README.md`,
      `AGENTS.md` (S7), `manipulation_implementation_proposal.md` (still names
      `gripper_body_link`).

### Phase 4: verification (before merging into the fork's `main`)
- [ ] `rosdep install --from-paths src -iy` and `colcon build` + `colcon test` in **both**
      Docker images (`docker/Dockerfile`, `docker/Dockerfile.jazzy`).
- [ ] `xacro` + `check_urdf` for every combination of `gripper`, `ftm`, `wrist_camera`,
      `workspace_obstacles` (16 cases), plus `lwa4p_gazebo.urdf.xacro sim:=classic|gz`.
- [ ] RViz on the real robot: the camera point cloud lines up with the table and obstacles.
      This is the S1 test.
- [ ] MoveIt plan + execute: Gazebo Classic (Humble), Gazebo Harmonic (Jazzy), Isaac.
- [ ] Real robot: driver enable with the commutation check, `/diagnostics`, `~/diagnose`,
      gripper, `ftm` node on the shared can1; loop period at 20 Hz with the extra torque read.

### Phase 5: land in the fork
- [ ] Push `upstream-main-merge` and open a PR into `RoboCup-FEI/manipulation-Simulation` `main`.
      **Use a merge commit, not squash or rebase.** A squash would cut the link to upstream
      history again.
- [ ] Rebase or recreate `calibration-poses-humble`, `calibration-poses-jazzy` and
      `joint-jog-teleop` on the new `main`, or close them.

## 5. Order and size

| Step | Commits | Depends on |
|---|---|---|
| Phase 1 merge | 1 | nothing |
| Phase 2 model | ~5 | D1–D3, Phase 1 |
| Phase 3 consumers/docs | ~4 | Phase 2 link names |
| Phase 4 verification | none | Phases 1–3, robot time |

## 6. Rollback

Until Phase 5, `main` is untouched. To start over:
```bash
git merge --abort
```
or delete the branch, and drop the graft if it's still there:
```bash
git replace -d c8086cd
```

## 7. Later: contributing back to Tomas Jurov

Not part of this branch. Once it's merged into the fork:
1. `git switch -c contrib/<topic> upstream/main`.
2. Bring over only the agreed paths, e.g. `git checkout upstream-main-merge -- src/lwa4p_driver`
   for the diagnostics.
3. Open one small PR per topic (diagnostics, FTS/mesh model once licensed, …). Don't include
   `isaac/`, example scripts, team docs or the LICENSE line unless Tomas agrees.
