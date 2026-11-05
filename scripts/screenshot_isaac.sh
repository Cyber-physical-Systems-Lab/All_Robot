#!/usr/bin/env bash
# Capture docs/screenshots/isaac/<robot>.png, one Isaac Sim process per robot.
#
#   bash scripts/screenshot_isaac.sh            # all robots
#   bash scripts/screenshot_isaac.sh nova5 go2
#
# Needs ISAACLAB_PATH (conda activate env_isaaclab, or docker/run.sh isaaclab) and the USDs from
# isaaclab/convert_all.sh. Several robots in one process leak PhysX / renderer state into each
# other (NaNs, blank frames), so every robot gets a fresh app. Prints a PASS/FAIL table.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${ISAACLAB_PATH:?set ISAACLAB_PATH (conda activate env_isaaclab, or use docker/run.sh isaaclab)}"
export OMNI_KIT_ACCEPT_EULA=YES
LOG_DIR="${TMPDIR:-/tmp}/cps_shots"; mkdir -p "$LOG_DIR"

PY=python3   # any interpreter with PyYAML, just to list the robots
python3 -c "import yaml" 2>/dev/null || PY="$ISAACLAB_PATH/_isaac_sim/python.sh"
ROBOTS=("$@")
[ ${#ROBOTS[@]} -eq 0 ] && mapfile -t ROBOTS < <("$PY" -c \
  "import yaml;print('\n'.join(yaml.safe_load(open('$REPO/robots.yaml'))['robots']))")

rows=(); fail=0
for r in "${ROBOTS[@]}"; do
  log="$LOG_DIR/isaac_$r.log"
  echo "=== isaac / $r  (log: $log)"
  "$ISAACLAB_PATH/isaaclab.sh" -p "$REPO/isaaclab/spawn_robot.py" --robot "$r" --screenshot --headless >"$log" 2>&1
  row="$(grep -E "^\| $r \|" "$log" | tail -1)"
  [ -z "$row" ] && row="| $r | FAIL | crashed, see $log |"
  [[ "$row" == *"| PASS |"* ]] || fail=1
  echo "    $row"; rows+=("$row")
done
printf '\n| robot | Isaac Lab | note |\n|---|---|---|\n'; printf '%s\n' "${rows[@]}"
exit $fail
