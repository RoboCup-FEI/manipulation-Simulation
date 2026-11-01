#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="${ISAACSIM_PATH:-$HERE/../../../isaac-setup/runtime/isaac-sim-6.2.0}"
if [[ -z "${ISAACSIM_PATH:-}" && ! -x "$SIM/python.sh" && -x "$HOME/isaacsim/python.sh" ]]; then
    SIM="$HOME/isaacsim"
fi
if [[ ! -x "$SIM/python.sh" ]]; then
    echo 'Set ISAACSIM_PATH to your standalone Isaac Sim 6.2 installation.' >&2
    exit 1
fi
SIM="$(cd "$SIM" && pwd)"
MODE="${1:-gui}"
if (( $# )); then shift; fi
# Native ROS Python paths must not leak into Isaac's bundled Python 3.12.
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
export OMNI_KIT_ACCEPT_EULA=YES
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
# The host has CUDA 12.0. Prefer the CUDA libraries shipped with this Sim build;
# its bundled PyTorch requires newer symbols from the same libcudart.so.12 ABI.
CUDA_LIBRARY_DIRS=("$SIM"/exts/isaacsim.pip.nv/pip_prebundle/nvidia/*/lib)
if [[ -d "${CUDA_LIBRARY_DIRS[0]}" ]]; then
    export LD_LIBRARY_PATH="$(IFS=:; printf '%s' "${CUDA_LIBRARY_DIRS[*]}")"
fi
if [[ -n "${ISAAC_ROS_DISTRO:-}" ]]; then
    case "$ISAAC_ROS_DISTRO" in
        humble|jazzy) ;;
        *) echo 'ISAAC_ROS_DISTRO must be humble or jazzy.' >&2; exit 2 ;;
    esac
    export ROS_DISTRO="$ISAAC_ROS_DISTRO"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH="$SIM/exts/isaacsim.ros2.core/$ROS_DISTRO/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
case "$MODE" in
    gui)
        exec "$SIM/isaac-sim.sh" --/renderer/multiGpu/enabled=false \
            --/plugins/carb.tasking.plugin/threadCount=4 \
            --/plugins/omni.tbb.globalcontrol/maxThreadCount=4 \
            --/app/renderer/resolution/width=960 --/app/renderer/resolution/height=540 "$@"
        ;;
    python) exec "$SIM/python.sh" "$@" ;;
    check)
        exec "$SIM/isaac-sim.compatibility_check.sh" --no-window --/app/quitAfter=60 "$@"
        ;;
    *) echo 'Usage: run.sh [gui|python SCRIPT [ARGS...]|check]' >&2; exit 2 ;;
esac
