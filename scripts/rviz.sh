#!/usr/bin/env bash
# Runs rviz2 behind noVNC so it's reachable from a browser on the host —
# there's no way to pop an X window out of this Linux container onto macOS.
# Xvnc is the X server and the VNC server in one process. `make rviz` runs
# this in the foreground; Ctrl-C tears everything down via the trap below.
#
# The -r /tf remaps follow $NS (default /husky, set in env.sh, matching
# autonomy.launch.py); TF is published on the namespaced topic even though
# frame_ids themselves aren't prefixed.
set -e

export DISPLAY=:1
VNC_PORT=5901
NOVNC_PORT=6080

if pgrep -f "^Xvnc ${DISPLAY} " >/dev/null; then
  echo "rviz already running in this container — Ctrl-C the other \`make rviz\` first" >&2
  exit 1
fi

cleanup() {
  kill "$rviz_pid" "$vnc_pid" "$websockify_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

Xvnc "$DISPLAY" -SecurityTypes None -geometry 1280x800 -depth 24 \
  >/ws/logs/xvnc.log 2>&1 &
vnc_pid=$!

# Wait for Xvnc's socket instead of guessing with a fixed sleep.
xvnc_up=false
for _ in $(seq 1 20); do
  (exec 3<>"/dev/tcp/localhost/${VNC_PORT}") 2>/dev/null && exec 3>&- && xvnc_up=true && break
  sleep 0.5
done
if ! $xvnc_up; then
  echo "Xvnc never came up, see /ws/logs/xvnc.log" >&2
  exit 1
fi

source /ws/scripts/env.sh
rviz2 -d /ws/src/spar_bringup/rviz/spar.rviz \
  --ros-args -p use_sim_time:=true \
  -r /tf:="$NS/tf" -r /tf_static:="$NS/tf_static" \
  >/ws/logs/rviz2.log 2>&1 &
rviz_pid=$!

websockify --web=/usr/share/novnc "$NOVNC_PORT" "localhost:${VNC_PORT}" \
  >/ws/logs/novnc.log 2>&1 &
websockify_pid=$!

echo "rviz2 ready: http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=true&resize=remote"
wait "$rviz_pid"
