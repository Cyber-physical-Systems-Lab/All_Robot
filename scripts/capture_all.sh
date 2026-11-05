#!/usr/bin/env bash
# Capture every robot in every simulator via Docker, then refresh the README gallery.
#
#   bash scripts/capture_all.sh                  # rviz gazebo mujoco isaac
#   bash scripts/capture_all.sh mujoco isaac     # subset
#
# Output: docs/screenshots/<simulator>/<robot>.png  (commit them together with README.md)
# Without Docker, run the per-simulator commands from README "Screenshots" inside each conda env.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
SIMS=("$@"); [ ${#SIMS[@]} -eq 0 ] && SIMS=(rviz gazebo mujoco isaac)
status=()
for s in "${SIMS[@]}"; do
  echo "############ $s"
  case "$s" in
    rviz)   docker/run.sh ros_gz bash scripts/screenshot_ros.sh rviz ;;
    gazebo) docker/run.sh ros_gz bash scripts/screenshot_ros.sh gazebo ;;
    mujoco) docker/run.sh mujoco bash -c "python scripts/build_assets.py --mujoco && MUJOCO_GL=osmesa python scripts/screenshot_mujoco.py" ;;
    isaac)  docker/run.sh isaaclab bash -c "bash isaaclab/convert_all.sh && /workspace/isaaclab/isaaclab.sh -p isaaclab/spawn_robot.py --robot all --screenshot --headless" ;;
    *) echo "unknown simulator $s"; false ;;
  esac
  status+=("$s:$?")
done
# containers run as root: give the files back to the current user
docker run --rm -v "$REPO/docs:/d" alpine chown -R "$(id -u):$(id -g)" /d >/dev/null 2>&1 || true
python3 scripts/make_gallery.py
echo "exit codes: ${status[*]}"
