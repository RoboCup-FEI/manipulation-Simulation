#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEGRATION="$HERE"
REPOSITORY="$(cd "$HERE/../.." && pwd)"
export LWA4P_ISAAC_CACHE="${LWA4P_ISAAC_CACHE:-$HERE/.runtime}"
REPORT_DIR="${LWA4P_ISAAC_REPORT_DIR:-$LWA4P_ISAAC_CACHE/reports}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-68}"
SIM_ARGS=()
RVIZ=true
TEST=0
for arg in "$@"; do
    case "$arg" in
        --headless) SIM_ARGS+=(--headless); RVIZ=false ;;
        --no-rviz) RVIZ=false ;;
        --test) TEST=1; SIM_ARGS+=(--headless); RVIZ=false ;;
        -h|--help)
            echo 'Usage: isaac/moveit/run.sh [--headless|--no-rviz|--test]'
            echo 'Default: Isaac window + MoveIt/RViz. Default ROS_DOMAIN_ID: 68.'
            exit 0 ;;
        *) echo "Unknown argument: $arg" >&2; exit 2 ;;
    esac
done
# Validate the existing Python environment without starting a simulator.
"$HERE/isaac.sh" python -c 'import sys; print("Isaac Python:", sys.version.split()[0])'
if ! docker image inspect lwa4p:humble >/dev/null 2>&1; then
    docker build --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
        -f "$REPOSITORY/docker/Dockerfile" -t lwa4p:humble "$REPOSITORY"
fi
python3 "$INTEGRATION/prepare.py"
LWA4P_ISAAC_CACHE="$(cd "$LWA4P_ISAAC_CACHE" && pwd)"
export LWA4P_ISAAC_CACHE
MOUNTS=(--mount "type=bind,src=$REPOSITORY,dst=$REPOSITORY")
case "$LWA4P_ISAAC_CACHE/" in
    "$REPOSITORY/"*) ;;
    *) MOUNTS+=(--mount "type=bind,src=$LWA4P_ISAAC_CACHE,dst=$LWA4P_ISAAC_CACHE") ;;
esac
rm -f "$LWA4P_ISAAC_CACHE/simulation.json"
if (( TEST )); then rm -f "$LWA4P_ISAAC_CACHE/execution.json"; fi
CONTAINER="lwa4p-isaac-moveit-$$"
SIM_PID=""
ROS_PID=""
ADDED_XHOST=0
cleanup() {
    trap - EXIT INT TERM
    docker stop --time 5 "$CONTAINER" >/dev/null 2>&1 || true
    if [[ -n "$SIM_PID" ]]; then
        kill -INT -- "-$SIM_PID" 2>/dev/null || true
        for _ in {1..30}; do
            kill -0 "$SIM_PID" 2>/dev/null || break
            sleep .2
        done
        kill -TERM -- "-$SIM_PID" 2>/dev/null || true
    fi
    [[ -z "$ROS_PID" ]] || wait "$ROS_PID" 2>/dev/null || true
    [[ -z "$SIM_PID" ]] || wait "$SIM_PID" 2>/dev/null || true
    if [[ "$ADDED_XHOST" == 1 ]]; then
        xhost "-si:localuser:$(id -un)" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
DISPLAY_ARGS=()
if [[ "$RVIZ" == true ]]; then
    : "${DISPLAY:?Start this command in a graphical desktop terminal or use --headless.}"
    DISPLAY_ARGS=(-e "DISPLAY=$DISPLAY" -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1
                  -e QT_QPA_PLATFORM=xcb --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly)
    if [[ -n "${XAUTHORITY:-}" && -f "$XAUTHORITY" ]]; then
        DISPLAY_ARGS+=(-e XAUTHORITY=/tmp/isaac-rviz-xauthority
                      --mount "type=bind,src=$XAUTHORITY,dst=/tmp/isaac-rviz-xauthority,readonly")
    elif ! xhost | rg -Fq "SI:localuser:$(id -un)"; then
        xhost "+si:localuser:$(id -un)" >/dev/null
        ADDED_XHOST=1
    fi
fi
echo "[LWA4P] Starting Isaac Sim, ROS domain $ROS_DOMAIN_ID; log: $LWA4P_ISAAC_CACHE/simulation.log"
setsid env ISAAC_ROS_DISTRO=humble ROS_LOCALHOST_ONLY=0 \
    "$HERE/isaac.sh" python "$INTEGRATION/simulation.py" "${SIM_ARGS[@]}" \
    > "$LWA4P_ISAAC_CACHE/simulation.log" 2>&1 &
SIM_PID=$!
for ((second=0; second<900; second++)); do
    if rg -q '\[LWA4P\] READY' "$LWA4P_ISAAC_CACHE/simulation.log"; then break; fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        tail -60 "$LWA4P_ISAAC_CACHE/simulation.log" >&2
        exit 1
    fi
    sleep 1
done
if (( second == 900 )); then echo 'Isaac startup timed out.' >&2; exit 1; fi
echo '[LWA4P] Physical robot ready; starting MoveIt 2 Humble.'
docker run --rm --init --name "$CONTAINER" --network host --ipc host \
    --user "$(id -u):$(id -g)" \
    -e "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e "LWA4P_ISAAC_CACHE=$LWA4P_ISAAC_CACHE" "${DISPLAY_ARGS[@]}" \
    "${MOUNTS[@]}" --workdir "$REPOSITORY" \
    lwa4p:humble ros2 launch "$INTEGRATION/moveit.launch.py" "rviz:=$RVIZ" \
    > "$LWA4P_ISAAC_CACHE/moveit.log" 2>&1 &
ROS_PID=$!
for ((second=0; second<30; second++)); do
    if [[ "$(docker inspect --format '{{.State.Running}}' "$CONTAINER" 2>/dev/null || true)" == true ]]; then break; fi
    if ! kill -0 "$ROS_PID" 2>/dev/null; then
        tail -60 "$LWA4P_ISAAC_CACHE/moveit.log" >&2; exit 1
    fi
    sleep 1
done
if (( second == 30 )); then echo 'MoveIt container startup timed out.' >&2; exit 1; fi
if (( TEST )); then
    docker exec "$CONTAINER" /usr/local/bin/lwa4p-entrypoint \
        python3 "$INTEGRATION/verify_execution.py" | tee "$LWA4P_ISAAC_CACHE/execution.log"
    mkdir -p "$REPORT_DIR"
    cp "$LWA4P_ISAAC_CACHE/execution.json" "$REPORT_DIR/lwa4p-moveit-execution.json"
    echo "[LWA4P] Verification passed: $REPORT_DIR/lwa4p-moveit-execution.json"
else
    echo '[LWA4P] RViz: group arm → goal ready/home/work1/work2 → Plan & Execute.'
    echo '[LWA4P] Gripper: group gripper → open/closed. Ctrl+C stops both processes.'
    wait -n "$SIM_PID" "$ROS_PID"
fi
