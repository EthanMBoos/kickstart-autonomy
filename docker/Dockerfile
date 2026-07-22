FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-rmw-zenoh-cpp \
      ros-jazzy-rviz2 \
      tigervnc-standalone-server \
      novnc \
      websockify \
 && rm -rf /var/lib/apt/lists/*

# rviz2 for a headless Linux container has nowhere to put its window on
# macOS, so it's tunneled through a browser instead: Xvnc (tigervnc) is the
# X server and the VNC server in one process, websockify/novnc bridge that
# to a plain web page. See scripts/rviz.sh and `make rviz`.

# Zenoh instead of DDS: discovery goes through a router over TCP (run by the
# core service, scripts/core.sh), so none of the DDS-in-docker failure modes
# exist — no multicast dependency, no participant-index ceiling, no wedged
# wait-sets.
# Both FastDDS and CycloneDDS were tried first and hit those in this setup.
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_ROUTER_CHECK_ATTEMPTS=10

# No simulator in here: the sim (MuJoCo physics + rendering) runs in Unity on
# the host and connects over the ROS-TCP endpoint. This container is pure ROS.

WORKDIR /ws

# The image holds the *environment* only; the workspace (src, scripts, build,
# install, logs) is bind-mounted by compose so build artifacts and logs land
# on the host. Only the package manifests are copied in — just enough for
# rosdep to install dependencies. The actual colcon build is run by hand,
# in the container, and lands in ./build on the host (see README), so code
# edits need an incremental rebuild, never an image rebuild.
COPY src/spar/package.xml src/spar/
COPY src/spar_bringup/package.xml src/spar_bringup/
COPY src/ros_tcp_endpoint/package.xml src/ros_tcp_endpoint/
RUN . /opt/ros/jazzy/setup.sh \
 && apt-get update && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
 && rm -rf /var/lib/apt/lists/*

# The core service (see compose.yaml) runs the ROS-TCP endpoint before any
# student build exists, so that one package is also baked into the image at
# /opt/spar_core. The bind-mounted src/ros_tcp_endpoint still builds into
# the student workspace like everything else; core just never uses that copy.
COPY src/ros_tcp_endpoint /opt/spar_core/src/ros_tcp_endpoint
RUN . /opt/ros/jazzy/setup.sh \
 && cd /opt/spar_core && colcon build

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
