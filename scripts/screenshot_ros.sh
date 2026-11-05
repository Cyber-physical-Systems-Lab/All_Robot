#!/usr/bin/env bash
# Capture docs/screenshots/{rviz,gazebo}/<robot>.png on a virtual display (no monitor needed).
#
#   bash scripts/screenshot_ros.sh rviz   [robot ...]
#   bash scripts/screenshot_ros.sh gazebo [robot ...]
#
# Run inside env_ros_gz or docker/run.sh ros_gz (the package must be built and sourced).
# Needs: Xvfb + ImageMagick + x11-utils (apt install xvfb imagemagick x11-utils)
# Tunables: SHOT_WAIT (s after the window appears, default 6 rviz / 15 gazebo),
#           SHOT_TIMEOUT (s to wait for the window, default 90),
#           SHOT_DISPLAY (first X display number tried, default 99; the next free one is used)
set -uo pipefail
MODE="${1:-}"; shift || true
[[ "$MODE" == rviz || "$MODE" == gazebo ]] || { sed -n '2,10p' "$0"; exit 1; }
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="$REPO/docs/screenshots/$MODE"; mkdir -p "$OUT_DIR"
LOG_DIR="${TMPDIR:-/tmp}/cps_shots"; mkdir -p "$LOG_DIR"
for b in Xvfb import convert xwininfo ros2; do command -v $b >/dev/null || { echo "missing $b"; exit 1; }; done

ROBOTS=("$@")
[ ${#ROBOTS[@]} -eq 0 ] && mapfile -t ROBOTS < <(python3 -c \
  "import yaml;print('\n'.join(yaml.safe_load(open('$REPO/robots.yaml'))['robots']))")
if [ "$MODE" = rviz ]; then WAIT="${SHOT_WAIT:-6}"; else WAIT="${SHOT_WAIT:-15}"; fi
TIMEOUT="${SHOT_TIMEOUT:-90}"

# virtual display with software OpenGL, on the first free display number from SHOT_DISPLAY (99)
n="${SHOT_DISPLAY:-99}"
while [ -e "/tmp/.X$n-lock" ] || [ -e "/tmp/.X11-unix/X$n" ]; do n=$((n + 1)); done
export DISPLAY=":$n"
Xvfb "$DISPLAY" -screen 0 1600x1200x24 +extension GLX +render -noreset >"$LOG_DIR/xvfb.log" 2>&1 &
XVFB=$!
for _ in $(seq 20); do xwininfo -root >/dev/null 2>&1 && break; sleep 0.5; done
xwininfo -root >/dev/null 2>&1 || { echo "Xvfb did not start on $DISPLAY (see $LOG_DIR/xvfb.log)"; exit 1; }
export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
LAUNCH_PID=""
descendants() { local c; for c in $(pgrep -P "$1"); do descendants "$c"; echo "$c"; done; }
cleanup_sim() {   # stop the current launch by PID (never by pattern: that could hit a parent shell)
  [ -z "$LAUNCH_PID" ] && return 0
  local procs; procs="$(descendants "$LAUNCH_PID") $LAUNCH_PID"
  kill -INT "$LAUNCH_PID" 2>/dev/null
  for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
  kill -KILL $procs 2>/dev/null
  pkill -KILL -f '^ign gazebo' 2>/dev/null; pkill -KILL -f '^ruby /usr/bin/ign gazebo' 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null; LAUNCH_PID=""
  true
}
trap 'cleanup_sim; kill $XVFB 2>/dev/null' EXIT

# id of the 3D view: RViz's render panel / Gazebo's main window (largest match)
find_window() {
  local pat="$1"
  xwininfo -root -tree 2>/dev/null | awk -v pat="$pat" '
    $0 ~ pat { for (i = 1; i <= NF; i++) if ($i ~ /^[0-9]+x[0-9]+\+/) { split($i, g, /[x+]/);
               a = g[1] * g[2]; if (a > best) { best = a; id = $1 } } }
    END { if (id) print id }'
}

fail=0
for r in "${ROBOTS[@]}"; do
  out="$OUT_DIR/$r.png"; log="$LOG_DIR/${MODE}_$r.log"
  echo "=== $MODE / $r"
  if [ "$MODE" = rviz ]; then
    cfg="$LOG_DIR/$r.rviz"
    python3 "$REPO/scripts/shot_view.py" rviz "$r" "$cfg"
    ros2 launch cps_robot_description display.launch.py robot:="$r" gui:=false rviz_config:="$cfg" >"$log" 2>&1 &
    LAUNCH_PID=$!
    pat='"rviz2": [(]"rviz2" "rviz2"[)]'          # the OGRE render widget inside the RViz window
  else
    cfg="$LOG_DIR/$r.gui.config"
    python3 "$REPO/scripts/shot_view.py" gz-gui "$r" "$cfg"
    ros2 launch cps_robot_description gazebo.launch.py robot:="$r" gui_config:="$cfg" >"$log" 2>&1 &
    LAUNCH_PID=$!
    pat='"Gazebo": [(]"ign-gazebo-gui"'
  fi
  win=""
  for _ in $(seq "$TIMEOUT"); do win="$(find_window "$pat")"; [ -n "$win" ] && break; sleep 1; done
  if [ -z "$win" ]; then
    echo "    FAILED: no window after ${TIMEOUT}s (see $log)"; fail=1; cleanup_sim; continue
  fi
  sleep "$WAIT"
  if grep -qE "process has died|Aborted|Segmentation fault" "$log"; then
    echo "    FAILED: simulator crashed (see $log)"; rm -f "$out"; fail=1; cleanup_sim; continue
  fi
  win="$(find_window "$pat")"   # again: the first match can be a splash screen that is gone now
  # timeout: import blocks forever if the window vanishes during the grab
  if timeout 30 import -window "$win" "$out" 2>>"$log" && [ -s "$out" ] \
     && [ "$(convert "$out" -format '%[fx:standard_deviation>0.03]' info:)" = 1 ]; then
    [ "$MODE" = gazebo ] && mogrify -gravity North -chop 0x48 "$out"   # drop the "Gazebo" app bar
    echo "    saved ${out#$REPO/}"
  else
    echo "    FAILED: blank or near-uniform capture (see $log)"; rm -f "$out"; fail=1
  fi
  cleanup_sim
done
exit $fail
