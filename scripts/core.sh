#!/usr/bin/env bash
# The infra container: one zenoh router + one ROS-TCP endpoint, nothing
# else. The robot containers join this container's network namespace
# (compose `network_mode: "service:core"`), so all of them behave like
# processes on one machine and everything is localhost.
# No -u: colcon's generated setup.bash reads variables it never sets
# (COLCON_TRACE), which nounset turns into a fatal error.
set -eo pipefail

ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/zenoh_router.log 2>&1 &

# The endpoint comes from the image (/opt/spar_core, see the Dockerfile),
# not the student workspace, so core runs on a fresh clone before any
# colcon build exists.
source /opt/spar_core/install/setup.bash
exec ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
