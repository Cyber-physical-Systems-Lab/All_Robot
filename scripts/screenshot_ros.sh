#!/usr/bin/env bash
# Capture docs/screenshots/{rviz,gazebo}/<robot>.png on a virtual display (no monitor needed).
#
#   bash scripts/screenshot_ros.sh rviz   [robot ...]
#   bash scripts/screenshot_ros.sh gazebo [robot ...]
#
# Run inside env_ros_gz or docker/run.sh ros_gz (the package must be built and sourced).
# Needs: Xvfb + ImageMagick  (apt install xvfb imagemagick; already in the ros_gz Docker image)
# Tunables: SHOT_WAIT (s before capture, default 12 rviz / 20 gazebo), SHOT_DISPLAY (default 99)
set -uo pipefail
MODE="${1:-}"; shift || true
[[ "$MODE" == rviz || "$MODE" == gazebo ]] || { sed -n '2,9p' "$0"; exit 1; }
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$REPO/docs/screenshots/$MODE"; mkdir -p "$OUT_DIR"
for b in Xvfb import convert ros2; do command -v $b >/dev/null || { echo "missing $b"; exit 1; }; done

ROBOTS=("$@")
[ ${#ROBOTS[@]} -eq 0 ] && mapfile -t ROBOTS < <(python3 -c \
  "import yaml;print('\n'.join(yaml.safe_load(open('$REPO/robots.yaml'))['robots']))")
if [ "$MODE" = rviz ]; then WAIT="${SHOT_WAIT:-12}"; else WAIT="${SHOT_WAIT:-20}"; fi

# virtual display with software OpenGL
export DISPLAY=":${SHOT_DISPLAY:-99}"
Xvfb "$DISPLAY" -screen 0 1600x1200x24 +extension GLX +render -noreset >/dev/null 2>&1 &
XVFB=$!
export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
trap 'kill $XVFB 2>/dev/null' EXIT
sleep 2

fail=0
for r in "${ROBOTS[@]}"; do
  out="$OUT_DIR/$r.png"
  echo "=== $MODE / $r"
  if [ "$MODE" = rviz ]; then
    cfg="$(mktemp --suffix=.rviz)"
    python3 "$REPO/scripts/shot_view.py" rviz "$r" "$cfg"
    setsid ros2 launch cps_robot_description display.launch.py robot:="$r" gui:=false rviz_config:="$cfg" \
      >"/tmp/shot_$r.log" 2>&1 &
  else
    setsid ros2 launch cps_robot_description gazebo.launch.py robot:="$r" >"/tmp/shot_$r.log" 2>&1 &
  fi
  pid=$!
  sleep "$WAIT"
  if [ "$MODE" = gazebo ]; then
    ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean \
      --timeout 3000 --req "$(python3 "$REPO/scripts/shot_view.py" gz-pose "$r")" >/dev/null 2>&1 \
      || echo "    (could not move GUI camera; default view used)"
    sleep 4
  fi
  # whole virtual screen, then trim the empty black border around the window
  if import -window root png:- 2>/dev/null | convert - -trim +repage "$out" && [ -s "$out" ]; then
    echo "    saved ${out#$REPO/}"
  else
    echo "    FAILED (see /tmp/shot_$r.log)"; fail=1
  fi
  kill -TERM -- -"$pid" 2>/dev/null; sleep 2; kill -KILL -- -"$pid" 2>/dev/null
  pkill -f "ign gazebo" 2>/dev/null; pkill -f "ruby.*ign" 2>/dev/null; true
  wait "$pid" 2>/dev/null
done
exit $fail
