#!/usr/bin/env bash
# Regenerate every robot USD from the standalone URDFs with Isaac Lab's URDF converter.
#
#   conda activate env_isaaclab                # sets ISAACLAB_PATH (see envs/setup.sh)
#   bash isaaclab/convert_all.sh               # all robots
#   bash isaaclab/convert_all.sh nova5 myagv   # selected robots
# Docker:  docker/run.sh isaaclab bash isaaclab/convert_all.sh
#
# Output: robots/<r>/usd/<r>.usd  - a file (older Isaac Lab) or a folder holding <r>/<r>.usda + payloads/
#         (newer Isaac Lab); isaaclab/cps_robots_cfg.py resolves both.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${ISAACLAB_PATH:?set ISAACLAB_PATH (conda activate env_isaaclab, or use docker/run.sh isaaclab)}"
# plain interpreter for helper calls (isaaclab.sh -p prints an [INFO] line on stdout)
if [ -n "${CONDA_PREFIX:-}" ] && [ -x "$CONDA_PREFIX/bin/python" ]; then
  PY=("$CONDA_PREFIX/bin/python")                 # env_isaaclab
elif [ -x "$ISAACLAB_PATH/_isaac_sim/python.sh" ]; then
  PY=("$ISAACLAB_PATH/_isaac_sim/python.sh")      # Docker / binary install
else
  PY=(python3)
fi

"${PY[@]}" "$REPO/scripts/build_assets.py" >/dev/null   # make sure standalone URDFs are fresh

ROBOTS=("$@")
if [ ${#ROBOTS[@]} -eq 0 ]; then
  mapfile -t ROBOTS < <("${PY[@]}" -c "import yaml;print('\n'.join(yaml.safe_load(open('$REPO/robots.yaml'))['robots']))")
fi

for r in "${ROBOTS[@]}"; do
  read -r URDF FIXED < <("${PY[@]}" - "$REPO" "$r" <<'PY'
import sys, yaml, pathlib
repo, r = pathlib.Path(sys.argv[1]), sys.argv[2]
c = yaml.safe_load(open(repo / "robots.yaml"))["robots"][r]
u = repo / c["urdf"]
isaac = u.parent / "isaac" / u.name          # flattened meshes (scripts/build_assets.py --isaac)
print(isaac if isaac.exists() else u.parent / "standalone" / u.name, "1" if c.get("fixed_base", True) else "0")
PY
)
  OUT="$REPO/robots/$r/usd/$r.usd"
  EXTRA=()
  [ "$FIXED" = "1" ] && EXTRA+=(--fix-base)
  echo "=== $r : $URDF -> $OUT ${EXTRA[*]:-}"
  rm -rf "$OUT"   # stale output from a previous run would otherwise be kept
  "$ISAACLAB_PATH/isaaclab.sh" -p "$ISAACLAB_PATH/scripts/tools/convert_urdf.py" \
      "$URDF" "$OUT" "${EXTRA[@]}" \
      --joint-stiffness 400.0 --joint-damping 40.0 --joint-target-type position --headless
  # the converter writes the absolute source path into doc comments; keep the committed files clean
  [ -d "$OUT" ] && find "$OUT" -name "*.usda" -exec sed -i "s|$REPO/|<repo>/|g" {} +
done
