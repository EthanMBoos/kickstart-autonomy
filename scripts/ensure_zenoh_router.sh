#!/usr/bin/env bash
# Starts the rmw_zenoh_cpp router if nothing is listening on its port yet.
# One router serves the whole container; the entrypoint starts it once, but
# any manually relaunched `ros2 launch` also needs it up, and a plain
# env-var guard doesn't cross a `docker exec` process boundary, so this
# checks the port itself instead.
set -euo pipefail

if [ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ] && \
   ! (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; then
  ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/zenoh_router.log 2>&1 &
  sleep 2
fi
