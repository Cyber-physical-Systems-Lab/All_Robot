#!/usr/bin/env bash
# Build the mounted package on start (fast with --symlink-install), then run the command.
set -e
source /opt/ros/humble/setup.bash
cd /ws
if [ ! -f install/setup.bash ] || [ src/All_Robot/robots.yaml -nt install/setup.bash ] \
   || [ src/All_Robot/CMakeLists.txt -nt install/setup.bash ]; then
  colcon build --symlink-install --packages-select cps_robot_description >/tmp/colcon.log 2>&1 \
    || { cat /tmp/colcon.log; exit 1; }
fi
source install/setup.bash
grep -q "install/setup.bash" ~/.bashrc 2>/dev/null || \
  echo "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash" >> ~/.bashrc
exec "$@"
