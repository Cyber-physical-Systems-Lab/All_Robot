#!/usr/bin/env bash
# Create the conda environments for the lab simulators.
#
#   bash envs/setup.sh isaaclab   # env_isaaclab : Isaac Sim 4.5 + Isaac Lab 2.1   (~25 GB, RTX GPU)
#   bash envs/setup.sh mujoco     # env_mujoco   : MuJoCo 3.x
#   bash envs/setup.sh ros_gz     # env_ros_gz   : ROS 2 Humble + Gazebo Fortress + RViz2 (RoboStack)
#   bash envs/setup.sh all
#
# Options (environment variables):
#   ISAACLAB_PATH=~/IsaacLab      where Isaac Lab is cloned   (isaaclab)
#   ISAACLAB_RL=none              RL frameworks for ./isaaclab.sh --install: none|rsl_rl|skrl|rl_games|sb3|all
#   CPS_WS=~/cps_ws               colcon workspace for this package (ros_gz)
# Re-running is safe: existing environments are updated, not recreated.
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAACLAB_VERSION=v2.1.0
ISAACLAB_PATH="${ISAACLAB_PATH:-$HOME/IsaacLab}"
ISAACLAB_RL="${ISAACLAB_RL:-none}"
CPS_WS="${CPS_WS:-$HOME/cps_ws}"

command -v conda >/dev/null || { echo "conda not found. Install Miniconda: https://docs.anaconda.com/miniconda/"; exit 1; }
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"
SOLVER=conda; command -v mamba >/dev/null && SOLVER=mamba

env_exists() { conda env list | awk '{print $1}' | grep -qx "$1"; }

create_env() {  # $1 = yml file, $2 = env name
  if env_exists "$2"; then
    echo ">>> updating $2"; $SOLVER env update -n "$2" -f "$1" --prune
  else
    echo ">>> creating $2"; $SOLVER env create -f "$1"
  fi
}

activate_hook() {  # $1 = env, $2 = script body run on `conda activate`
  local d; d="$(conda run -n "$1" printenv CONDA_PREFIX)/etc/conda/activate.d"
  mkdir -p "$d"; printf '%s\n' "$2" > "$d/cps_robots.sh"
}

setup_isaaclab() {
  for b in cmake g++ git; do
    command -v $b >/dev/null || { echo "missing '$b': sudo apt install cmake build-essential git"; exit 1; }
  done
  create_env "$REPO/envs/env_isaaclab.yml" env_isaaclab
  if [ ! -d "$ISAACLAB_PATH/.git" ]; then
    git clone --branch "$ISAACLAB_VERSION" https://github.com/isaac-sim/IsaacLab.git "$ISAACLAB_PATH"
  fi
  conda activate env_isaaclab
  export OMNI_KIT_ACCEPT_EULA=YES
  (cd "$ISAACLAB_PATH" && ./isaaclab.sh --install "$ISAACLAB_RL")
  conda env config vars set -n env_isaaclab \
      ISAACLAB_PATH="$ISAACLAB_PATH" CPS_ROBOTS_PATH="$REPO" OMNI_KIT_ACCEPT_EULA=YES >/dev/null
  conda deactivate
  cat <<EOF

env_isaaclab ready. Verify:
  conda activate env_isaaclab
  python \$ISAACLAB_PATH/scripts/tutorials/00_sim/create_empty.py      # first run pulls extensions (~10 min)
  python \$CPS_ROBOTS_PATH/isaaclab/spawn_robot.py --robot nova5_2f85
EOF
}

setup_mujoco() {
  create_env "$REPO/envs/env_mujoco.yml" env_mujoco
  conda env config vars set -n env_mujoco CPS_ROBOTS_PATH="$REPO" >/dev/null
  echo -e "\nenv_mujoco ready:\n  conda activate env_mujoco\n  python -m mujoco.viewer --mjcf=\$CPS_ROBOTS_PATH/robots/nova5_2f85/mjcf/scene.xml"
}

setup_ros_gz() {
  create_env "$REPO/envs/env_ros_gz.yml" env_ros_gz
  mkdir -p "$CPS_WS/src"
  ln -sfn "$REPO" "$CPS_WS/src/All_Robot"
  conda activate env_ros_gz
  (cd "$CPS_WS" && colcon build --symlink-install --packages-select cps_robot_description \
      --cmake-args -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python")
  conda deactivate
  # auto-source the workspace whenever the env is activated
  activate_hook env_ros_gz "[ -f \"$CPS_WS/install/setup.bash\" ] && source \"$CPS_WS/install/setup.bash\""
  echo -e "\nenv_ros_gz ready:\n  conda activate env_ros_gz\n  ros2 launch cps_robot_description gazebo.launch.py robot:=nova5_2f85"
}

case "${1:-}" in
  isaaclab) setup_isaaclab ;;
  mujoco)   setup_mujoco ;;
  ros_gz)   setup_ros_gz ;;
  all)      setup_mujoco; setup_ros_gz; setup_isaaclab ;;
  *) sed -n '2,13p' "$0"; exit 1 ;;
esac
