# The one copy of the in-container environment: ROS, the workspace overlay,
# and the robot namespace default. Sourced by the Makefile's ROS_ENV and by
# the scripts here; source it before any `set -u` (ROS setup.bash trips over
# unset variables under nounset). install/ may not exist yet — nothing
# auto-builds (see docker/entrypoint.sh) — so the trailing `true` keeps
# callers chaining `&&` alive so they get a shell to build in, not an error.
source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
export NS="${NS:-/husky}"
true
