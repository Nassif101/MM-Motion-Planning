# syntax=docker/dockerfile:1.7

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble AS development

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO
ARG ROS_MCP_VERSION=3.1.0
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
ARG COLCON_BUILD_MODE=parallel
ARG COLCON_PARALLEL_WORKERS=
ARG ROS2_WS_CMAKE_BUILD_TYPE=RelWithDebInfo

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV ROS_DISTRO=${ROS_DISTRO} \
    ROS_WS=/workspaces/mm-motion-planning/ros2_ws \
    WORKSPACE=/workspaces/mm-motion-planning \
    COLCON_BUILD_MODE=${COLCON_BUILD_MODE} \
    COLCON_PARALLEL_WORKERS=${COLCON_PARALLEL_WORKERS} \
    ROS2_WS_CMAKE_BUILD_TYPE=${ROS2_WS_CMAKE_BUILD_TYPE} \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    QT_X11_NO_MITSHM=1 \
    CCACHE_DIR=/home/${USERNAME}/.cache/ccache \
    UV_TOOL_DIR=/opt/uv-tools \
    UV_TOOL_BIN_DIR=/usr/local/bin \
    UV_CACHE_DIR=/var/cache/uv

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends \
    sudo git git-lfs curl wget ca-certificates gnupg lsb-release \
    build-essential cmake ninja-build ccache pkg-config gdb clang clang-format clang-tidy cppcheck \
    less htop tree unzip nano vim tmux lsof iputils-ping dnsutils usbutils pciutils gdbserver \
    python3-pip python3-dev python3-setuptools python3-venv python3-rosdep python3-vcstool \
    python3-colcon-common-extensions python3-colcon-mixin python3-argcomplete \
    python3-opencv python3-numpy python3-scipy python3-yaml \
    libboost-all-dev libeigen3-dev libfmt-dev libyaml-cpp-dev libtinyxml2-dev libpcl-dev \
    mesa-utils x11-apps x11-utils libgl1-mesa-dev libgl1-mesa-dri libglx-mesa0 libglu1-mesa \
    libglfw3 libglfw3-dev libglew-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    xvfb x11vnc novnc websockify openbox vulkan-tools \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-moveit-setup-assistant \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-warehouse-ros-sqlite \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-octomap-server \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-diagnostic-aggregator \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    ros-${ROS_DISTRO}-rosbridge-suite \
 && rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --break-system-packages transforms3d pyquaternion rosbags uv \
 && uv tool install "ros-mcp==${ROS_MCP_VERSION}"

# Bake the current workspace's non-source rosdeps into the image. The
# post-create rosdep pass remains as a safety net when package manifests change.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
 && apt-get install -y --no-install-recommends \
    bluez python3-apt \
    ros-${ROS_DISTRO}-axis-description \
    ros-${ROS_DISTRO}-clearpath-viz \
    ros-${ROS_DISTRO}-ewellix-description \
    ros-${ROS_DISTRO}-flir-ptu-description \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-hardware-interface-testing \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joy-linux \
    ros-${ROS_DISTRO}-microstrain-inertial-description \
    ros-${ROS_DISTRO}-ptz-action-server-msgs \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-robotiq-description \
    ros-${ROS_DISTRO}-ur-description \
    ros-${ROS_DISTRO}-velodyne-description \
    ros-${ROS_DISTRO}-zed-description \
 && rm -rf /var/lib/apt/lists/*

RUN existing_group="$(getent group "${USER_GID}" | cut -d: -f1 || true)" \
 && if [ -z "${existing_group}" ]; then \
      groupadd --gid "${USER_GID}" "${USERNAME}"; \
    elif [ "${existing_group}" != "${USERNAME}" ] && ! getent group "${USERNAME}" >/dev/null 2>&1; then \
      groupmod --new-name "${USERNAME}" "${existing_group}"; \
    fi \
 && existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)" \
 && if id -u "${USERNAME}" >/dev/null 2>&1; then \
      usermod --uid "${USER_UID}" --gid "${USER_GID}" "${USERNAME}"; \
    elif [ -n "${existing_user}" ]; then \
      usermod --login "${USERNAME}" --home "/home/${USERNAME}" --move-home "${existing_user}" \
      && usermod --gid "${USER_GID}" "${USERNAME}"; \
    else \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" --shell /bin/bash -m "${USERNAME}"; \
    fi \
 && usermod -aG sudo,video,dialout,plugdev "${USERNAME}" \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
 && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
 && mkdir -p "${CCACHE_DIR}" /var/cache/uv /tmp/.X11-unix \
 && chown -R "${USERNAME}:${USERNAME}" "${CCACHE_DIR}" /var/cache/uv \
 && chown root:root /tmp/.X11-unix \
 && chmod 1777 /tmp/.X11-unix

COPY .devcontainer/scripts/start-novnc.sh /usr/local/bin/start-novnc
RUN chmod 0755 /usr/local/bin/start-novnc

RUN cat <<'EOF_BASHRC' >> /etc/bash.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${ROS_WS}/install/setup.bash" ]; then
  source "${ROS_WS}/install/setup.bash"
fi

cb() {
  local build_type="${ROS2_WS_CMAKE_BUILD_TYPE:-RelWithDebInfo}"
  local args=(--symlink-install)

  if [ "${COLCON_BUILD_MODE:-parallel}" = "queued" ]; then
    args+=(--executor sequential)
  elif [ -n "${COLCON_PARALLEL_WORKERS:-}" ]; then
    args+=(--parallel-workers "${COLCON_PARALLEL_WORKERS}")
  fi

  args+=(--cmake-args "-DCMAKE_BUILD_TYPE=${build_type}" "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache")
  colcon build "${args[@]}" "$@"
}

cb-release() { ROS2_WS_CMAKE_BUILD_TYPE=Release cb "$@"; }
cb-debug() { ROS2_WS_CMAKE_BUILD_TYPE=RelWithDebInfo cb "$@"; }

alias srcws='source "${ROS_WS}/install/setup.bash"'
alias gz-headless='gz sim -s -r'
alias gz-gui='gz sim -g'
alias ros-tcp-server='ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000'
alias rosbridge='ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090'

start_novnc() {
  export DISPLAY="${NOVNC_DISPLAY:-:99}"
  /usr/local/bin/start-novnc "$@"
}
alias novnc='start_novnc'
EOF_BASHRC

RUN cat <<'EOF_PROFILE' > /etc/profile.d/mm-motion-planning-ros.sh
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${ROS_WS}/install/setup.bash" ]; then
  source "${ROS_WS}/install/setup.bash"
fi
EOF_PROFILE

USER ${USERNAME}
WORKDIR ${WORKSPACE}
