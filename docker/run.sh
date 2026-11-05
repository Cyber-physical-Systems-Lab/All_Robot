#!/usr/bin/env bash
# Run a simulator container with the repo mounted, X11 display and (for Isaac) the GPU.
#
#   docker/run.sh build [ros_gz|mujoco|isaaclab]       build image(s)
#   docker/run.sh <service> [command...]               run (default: interactive bash)
#
# Examples
#   docker/run.sh ros_gz   ros2 launch cps_robot_description display.launch.py robot:=go2
#   docker/run.sh ros_gz   ros2 launch cps_robot_description gazebo.launch.py robot:=nova5_2f85
#   docker/run.sh mujoco   python scripts/build_assets.py --mujoco
#   docker/run.sh mujoco   python -m mujoco.viewer --mjcf=robots/nova5_2f85/mjcf/scene.xml
#   docker/run.sh isaaclab bash isaaclab/convert_all.sh
#   docker/run.sh isaaclab /workspace/isaaclab/isaaclab.sh -p isaaclab/spawn_robot.py --robot nova5_2f85
#   docker/run.sh isaaclab /workspace/isaaclab/isaaclab.sh -p isaaclab/spawn_robot.py --robot all --check --headless
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
COMPOSE=(docker compose -f compose.yaml)

[ $# -ge 1 ] || { sed -n '2,15p' "$0"; exit 1; }
svc="$1"; shift

if [ "$svc" = build ]; then
  exec "${COMPOSE[@]}" build "$@"
fi

extra=()
if [ -n "${DISPLAY:-}" ]; then
  command -v xhost >/dev/null && xhost +local: >/dev/null 2>&1 || true
  [ "$svc" = mujoco ] && extra+=(-e MUJOCO_GL=glfw)
fi
if [ "$svc" = isaaclab ] && ! docker info 2>/dev/null | grep -qi nvidia; then
  echo "WARNING: NVIDIA Container Toolkit not detected; Isaac Sim needs it." >&2
  echo "  https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html" >&2
fi

[ $# -eq 0 ] && set -- bash
exec "${COMPOSE[@]}" run --rm "${extra[@]}" "$svc" "$@"
