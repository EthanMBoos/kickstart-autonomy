#!/usr/bin/env bash
set -e
cd /ws
source /opt/ros/jazzy/setup.bash

# No build happens here. The workspace is bind-mounted from the host (see
# compose.yaml), so `make ros2_container` only builds the image and starts
# the container; building the code (colcon build) is a separate, explicit
# step you run yourself in the shell this drops you into. install/ may not
# exist yet the first time.

# Nothing ROS-related launches here on purpose. The whole stack (bridge,
# localization, Nav2, behavior, see autonomy.launch.py) is meant to be run
# by hand, in a `make shell`, so a launch/param/bridge change is something
# you watch happen, not something a script hides.
/ws/scripts/ensure_zenoh_router.sh

# ROS_LOG_DIR is set once per container start, in a profile.d script so it
# reaches every later `docker exec ... bash -lc` shell (make shell, make
# ros2_container, the verify skill's automated bring-up). That's what lets
# a plain `ros2 launch spar_bringup autonomy.launch.py world:=blank`
# log correctly with no wrapper script: ros2 launch already respects
# ROS_LOG_DIR on its own (launch/logging.py), it just needs a value.
if [ -d /ws/logs ]; then
  ROS_LOG_DIR="$(/ws/scripts/claim_run_dir.sh)"
  printf 'world=%s\nstarted=%s\n' "${WORLD:-blank}" "$(date -Is)" \
    > "$ROS_LOG_DIR/run-info"
  echo "export ROS_LOG_DIR=$ROS_LOG_DIR" > /etc/profile.d/spar_ros_log_dir.sh
  echo "[entrypoint] logging to logs/$(basename "$ROS_LOG_DIR")"
fi

# The container just needs to stay up for `make ros2_container`/`make shell` to exec
# into (see compose.yaml's command); this exec's into that idle process.
exec "$@"
