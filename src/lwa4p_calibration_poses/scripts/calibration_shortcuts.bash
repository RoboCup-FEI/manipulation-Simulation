# Copyright 2026 Dmitrii Ognev
# Licensed under the MIT License. See LICENSE in the repository root.
#
# Host-side shortcuts for teaching and replaying calibration poses in Docker.
# Usage: source this file (e.g. from ~/.bashrc), then run calib-help.

LWA4P_IMAGE="${LWA4P_IMAGE:-lwa4p:humble-calib}"
LWA4P_POSE_DIR="${LWA4P_POSE_DIR:-$HOME/lwa4p_calibration}"
LWA4P_POSE_FILE_NAME="${LWA4P_POSE_FILE_NAME:-calibration_poses.txt}"

_CALIB_CONTAINER_POSE_DIR=/home/ros/calibration
_CALIB_REAL_CONTAINER=lwa4p-real
_CALIB_SIM_CONTAINER=lwa4p-light

_calib_container() {
  if [ -n "${LWA4P_CONTAINER:-}" ]; then
    echo "$LWA4P_CONTAINER"
    return 0
  fi
  local container
  for container in "$_CALIB_REAL_CONTAINER" "$_CALIB_SIM_CONTAINER"; do
    if [ -n "$(docker ps -q -f "name=^${container}$")" ]; then
      echo "$container"
      return 0
    fi
  done
  echo "No running $_CALIB_REAL_CONTAINER or $_CALIB_SIM_CONTAINER container," \
    "start one with calib-real or calib-sim" >&2
  return 1
}

_calib_tty_flag() {
  if [ -t 0 ]; then
    echo "-t"
  fi
}

_calib_exec() {
  local container
  container="$(_calib_container)" || return 1
  docker exec -i $(_calib_tty_flag) "$container" \
    bash -c "source /usr/local/bin/lwa4p-env.bash && $*"
}

_calib_docker_run() {
  local container="$1"
  shift
  mkdir -p "$LWA4P_POSE_DIR"
  xhost +si:localuser:"$(id -un)" > /dev/null
  docker run --rm -i $(_calib_tty_flag) --init --name "$container" --shm-size=512m \
    -e DISPLAY="$DISPLAY" -e LIBGL_ALWAYS_SOFTWARE=1 \
    --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
    --mount type=bind,src="$LWA4P_POSE_DIR",dst="$_CALIB_CONTAINER_POSE_DIR" \
    "$@"
}

calib-real() {
  local can_channel="${1:-can0}"
  _calib_docker_run "$_CALIB_REAL_CONTAINER" --network host -e ROS_DOMAIN_ID=43 \
    "$LWA4P_IMAGE" ros2 launch lwa4p_moveit_config real.launch.py can_channel:="$can_channel"
}

calib-sim() {
  _calib_docker_run "$_CALIB_SIM_CONTAINER" \
    "$LWA4P_IMAGE" ros2 launch lwa4p_moveit_config demo.launch.py
}

calib-nodes() {
  _calib_exec ros2 launch lwa4p_calibration_poses calibration_poses.launch.py \
    pose_file_path:="$_CALIB_CONTAINER_POSE_DIR/$LWA4P_POSE_FILE_NAME"
}

calib-shell() {
  _calib_exec bash
}

calib-enable() {
  _calib_exec "ros2 service call /lwa4p_driver/enable std_srvs/srv/Trigger '{}'"
}

calib-disable() {
  _calib_exec "ros2 service call /lwa4p_driver/disable std_srvs/srv/Trigger '{}'"
}

calib-reset() {
  _calib_exec "ros2 service call /lwa4p_driver/reset std_srvs/srv/Trigger '{}'"
}

calib-diagnose() {
  _calib_exec "ros2 service call /lwa4p_driver/diagnose std_srvs/srv/Trigger '{}'"
}

calib-joints() {
  _calib_exec python3 - << 'EOF'
import math

import rclpy
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = [f'arm_{joint}_joint' for joint in range(1, 7)]

rclpy.init()
node = rclpy.create_node('calib_joints_printer')
arm_states = []
node.create_subscription(
    JointState, 'joint_states',
    lambda message: arm_states.append(message) if ARM_JOINT_NAMES[0] in message.name else None,
    10)
while rclpy.ok() and not arm_states:
    rclpy.spin_once(node, timeout_sec=0.5)

positions = dict(zip(arm_states[0].name, arm_states[0].position))
degrees = []
for joint_name in ARM_JOINT_NAMES:
    degrees.append(math.degrees(positions[joint_name]))
    print(f'{joint_name}  {positions[joint_name]: .6f} rad  {degrees[-1]: 8.2f} deg')
print('calib-goto-deg ' + ' '.join(f'{value:.2f}' for value in degrees))
EOF
}

calib-goto() {
  if [ "$#" -ne 6 ]; then
    echo "Usage: calib-goto J1 J2 J3 J4 J5 J6   (radians)" >&2
    return 1
  fi
  local positions
  positions="$(IFS=,; echo "$*")"
  _calib_exec "ros2 service call /send_to_pose lwa4p_calibration_interfaces/srv/SendToPose" \
    "'{positions: [$positions]}'"
}

calib-goto-deg() {
  if [ "$#" -ne 6 ]; then
    echo "Usage: calib-goto-deg J1 J2 J3 J4 J5 J6   (degrees)" >&2
    return 1
  fi
  local radians=()
  local degrees
  for degrees in "$@"; do
    radians+=("$(awk -v value="$degrees" 'BEGIN { printf "%.6f", value * atan2(0, -1) / 180 }')")
  done
  calib-goto "${radians[@]}"
}

calib-save() {
  _calib_exec "ros2 service call /save_pose std_srvs/srv/Trigger '{}'"
}

calib-run() {
  _calib_exec "ros2 service call /run_calibration_sequence std_srvs/srv/Trigger '{}'"
}

calib-poses() {
  local pose_file="$LWA4P_POSE_DIR/$LWA4P_POSE_FILE_NAME"
  if [ ! -f "$pose_file" ]; then
    echo "No pose file yet: $pose_file" >&2
    return 1
  fi
  cat -n "$pose_file"
}

calib-new-file() {
  local pose_file="$LWA4P_POSE_DIR/$LWA4P_POSE_FILE_NAME"
  if [ -f "$pose_file" ]; then
    local backup_file
    backup_file="$pose_file.$(date +%Y%m%d_%H%M%S).bak"
    mv "$pose_file" "$backup_file"
    echo "Old poses moved to $backup_file"
  fi
}

calib-help() {
  cat << EOF
Calibration shortcuts (image $LWA4P_IMAGE, poses in $LWA4P_POSE_DIR/$LWA4P_POSE_FILE_NAME)

Start (each in its own terminal):
  calib-real [can0]        driver + MoveIt + RViz on the real robot
  calib-sim                MoveIt + RViz with mock hardware (no robot)
  calib-nodes              pose_teacher + calibration_sequence in the running container

Real robot:
  calib-enable             enable drives (each joint moves about 0.5 deg)
  calib-disable            disable drives, brakes close, aborts any motion
  calib-reset              reset drive faults
  calib-diagnose           fault codes, temperatures, currents

Teach and run:
  calib-joints             actual joint positions [rad]
  calib-goto-deg J1..J6    move to joint positions in degrees
  calib-goto J1..J6        move to joint positions in radians
  calib-save               append the actual pose to the pose file
  calib-poses              show the pose file
  calib-new-file           back up the pose file and start a new one
  calib-run                run the whole calibration sequence
  calib-shell              bash inside the running container
EOF
}
