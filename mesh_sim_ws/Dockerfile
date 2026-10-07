# ROS 2 Humble + Gazebo Sim Harmonic (the pairing this repo's README targets).
#
# NOTE: humble+harmonic has NO apt package. rosdistro releases ros_gz for humble at
# 0.244.x -- the Ignition/Fortress-era ros_ign* line -- and the OSRF apt repo ships
# only gz-* binaries, zero ros-* packages. So ros_gz must be COMPILED against Harmonic.
FROM ros:humble

ENV COLCON_WS=/root/colcon_ws
ENV COLCON_WS_SRC=/root/colcon_ws/src
ENV PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"
ENV DEBIAN_FRONTEND=noninteractive

ARG GZ_VERSION=harmonic
ENV GZ_VERSION=${GZ_VERSION}

RUN apt-get update -qq \
    && apt-get install -y wget git lsb-release gnupg \
    && rm -rf /var/lib/apt/lists/*

# OSRF repo -> provides the gz-harmonic binaries
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update -qq \
    && apt-get install -y \
        build-essential python3-pip python3-colcon-common-extensions \
        gz-harmonic \
        ros-humble-rcl-interfaces ros-humble-rclcpp ros-humble-builtin-interfaces \
        ros-humble-sdformat-urdf ros-humble-vision-msgs ros-humble-actuator-msgs \
        ros-humble-image-transport ros-humble-nav2-bringup ros-humble-navigation2 \
        ros-humble-slam-toolbox ros-humble-joint-state-publisher \
        ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard \
        ros-humble-rviz2 ros-humble-xacro ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# ros_gz compiled against Harmonic
RUN mkdir -p /root/ros_gz_ws/src \
    && git clone https://github.com/gazebosim/ros_gz.git -b humble /root/ros_gz_ws/src/ros_gz \
    && apt-get update -qq \
    && rosdep update --rosdistro humble \
    && (rosdep install -r --from-paths /root/ros_gz_ws/src -i -y --rosdistro humble || true) \
    && . /opt/ros/humble/setup.sh \
    && cd /root/ros_gz_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Everything below is NEW (rajant_mesh). It must stay BELOW the ros_gz build so
# that layer keeps its cache -- in particular iputils-ping must NOT be folded
# into the apt layer above, even though that is where it naturally belongs.
# ---------------------------------------------------------------------------

# NETWORK: archive.ubuntu.com is unreachable from this site -- DNS returns
# IPv6-only answers, there is no IPv6 egress, and the IPv4 path (91.189.92.23)
# times out too. kr.archive.ubuntu.com over IPv4 works. Kept BELOW the cached
# layers so it does not invalidate the expensive ros_gz build above.
RUN for f in /etc/apt/sources.list /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do \
        [ -f "$f" ] && sed -i \
            -e 's|http://archive.ubuntu.com|http://kr.archive.ubuntu.com|g' \
            -e 's|http://security.ubuntu.com|http://kr.archive.ubuntu.com|g' "$f" || true; \
    done; \
    printf 'Acquire::ForceIPv4 "true";\n' > /etc/apt/apt.conf.d/99force-ipv4

# iputils-ping is REQUIRED: rajant_api.is_host_reachable() shells out to `ping`.
# Its absence is what broke the first NUC2 deploy (FileNotFoundError: 'ping').
# packages.ros.org is crawling (~541 B/s vs 9 MB/s for the Ubuntu mirror) and
# apt-get update hard-fails on it (exit 100) after ~430s. iputils-ping and
# iproute2 are plain Ubuntu packages, so the ROS index is not needed here at
# all -- set it aside for this layer only, then restore it.
RUN mv /etc/apt/sources.list.d/ros2.sources /tmp/ros2.sources \
    && apt-get update -qq \
    && apt-get install -y --no-install-recommends iputils-ping iproute2 \
    && mv /tmp/ros2.sources /etc/apt/sources.list.d/ros2.sources \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
        protobuf==4.23.4 grpcio==1.56.2 grpcio-tools==1.56.2

# rajant_api is vendored -- it is not on PyPI.
COPY vendor/rajant_api /usr/local/lib/python3.10/dist-packages/rajant_api

# Both packages share ONE colcon workspace.
COPY src ${COLCON_WS_SRC}

RUN . /opt/ros/humble/setup.sh \
    && . /root/ros_gz_ws/install/setup.sh \
    && cd ${COLCON_WS} && colcon build

ENV GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2
ENV ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${COLCON_WS_SRC}/ackermann-vehicle-gzsim-ros2

# Namespace: build-time default, overridable at runtime with -e ROBOT_NS=robot2
ARG ROBOT_NS=robot1
ENV ROBOT_NS=${ROBOT_NS}

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /root/ros_gz_ws/install/setup.bash" >> /root/.bashrc \
    && echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

# Sources all three workspaces for NON-interactive use too (.bashrc returns
# early for non-interactive shells).  docker exec <c> /entrypoint.sh ros2 ...
RUN printf '%s\n' '#!/bin/bash' 'set -e' \
      'source /opt/ros/humble/setup.bash' \
      'source /root/ros_gz_ws/install/setup.bash' \
      'source /root/colcon_ws/install/setup.bash' \
      'exec "$@"' > /entrypoint.sh \
    && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
