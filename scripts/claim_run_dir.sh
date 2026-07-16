#!/usr/bin/env bash
# Atomically claims the next logs/runNNN directory and prints its absolute
# path. Called once per container start by docker/entrypoint.sh, which
# exports the result as ROS_LOG_DIR for every later shell (see there).
set -euo pipefail

n=1
while ! mkdir "/ws/logs/$(printf 'run%03d' "$n")" 2>/dev/null; do
  n=$((n + 1))
done
echo "/ws/logs/$(printf 'run%03d' "$n")"
