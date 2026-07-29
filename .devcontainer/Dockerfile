ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ARG COLCON_BUILD_MODE=parallel
ARG COLCON_PARALLEL_WORKERS=
ARG CMAKE_BUILD_PARALLEL_LEVEL=
ARG MAKEFLAGS=
ARG ROS2_WS_CMAKE_BUILD_TYPE=RelWithDebInfo

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV ROS_DISTRO=${ROS_DISTRO}
ENV COLCON_BUILD_MODE=${COLCON_BUILD_MODE}
ENV COLCON_PARALLEL_WORKERS=${COLCON_PARALLEL_WORKERS}
ENV CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL}
ENV MAKEFLAGS=${MAKEFLAGS}
ENV ROS2_WS_CMAKE_BUILD_TYPE=${ROS2_WS_CMAKE_BUILD_TYPE}
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV QT_X11_NO_MITSHM=1
ENV WORKSPACE=/mt-mohamad-nassif

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    curl \
    wget \
    ca-certificates \
    gnupg \
    lsb-release \
    software-properties-common \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    gdb \
    clang \
    clang-format \
    clang-tidy \
    cppcheck \
    valgrind \
    nano \
    vim \
    less \
    htop \
    tree \
    unzip \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-venv \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-argcomplete \
    python3-opencv \
    python3-numpy \
    python3-scipy \
    python3-yaml \
    libboost-all-dev \
    libeigen3-dev \
    libfmt-dev \
    libyaml-cpp-dev \
    libtinyxml2-dev \
    libpcl-dev \
    mesa-utils \
    x11-apps \
    x11-utils \
    libgl1-mesa-dev \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libglu1-mesa \
    libglfw3 \
    libglfw3-dev \
    libglew-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    openbox \
    tmux \
    iputils-ping \
    dnsutils \
    usbutils \
    pciutils \
    lsof \
    gdbserver \
    vulkan-tools \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-diagnostic-aggregator \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-imu-filter-madgwick \
    ros-${ROS_DISTRO}-octomap-server \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-moveit-setup-assistant \
    ros-${ROS_DISTRO}-warehouse-ros-sqlite \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-rqt-graph \
    ros-${ROS_DISTRO}-rqt-plot \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --break-system-packages \
    transforms3d \
    pyquaternion \
    rosbags

RUN mkdir -p /tmp/.X11-unix \
 && chown root:root /tmp/.X11-unix \
 && chmod 1777 /tmp/.X11-unix

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
 && chmod 0440 "/etc/sudoers.d/${USERNAME}"

RUN rosdep init 2>/dev/null || true \
 && rosdep update || true

COPY .devcontainer/scripts/start-novnc.sh /usr/local/bin/start-novnc
RUN chmod 0755 /usr/local/bin/start-novnc

RUN cat <<'EOF_BASHRC' >> /etc/bash.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
  source "${WORKSPACE}/install/setup.bash"
fi

cb() {
  local build_type="${ROS2_WS_CMAKE_BUILD_TYPE:-RelWithDebInfo}"
  local args=(--symlink-install)

  if [ "${COLCON_BUILD_MODE:-parallel}" = "queued" ]; then
    args+=(--executor sequential)
  elif [ -n "${COLCON_PARALLEL_WORKERS:-}" ]; then
    args+=(--parallel-workers "${COLCON_PARALLEL_WORKERS}")
  fi

  args+=(--cmake-args "-DCMAKE_BUILD_TYPE=${build_type}")
  colcon build "${args[@]}" "$@"
}

cb-release() {
  ROS2_WS_CMAKE_BUILD_TYPE=Release cb "$@"
}

cb-debug() {
  ROS2_WS_CMAKE_BUILD_TYPE=RelWithDebInfo cb "$@"
}

alias srcws='source "${WORKSPACE}/install/setup.bash"'
alias gz-headless='gz sim -s -r'
alias gz-gui='gz sim -g'
alias ros-tcp-server='ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000'

start_novnc() {
  export DISPLAY="${NOVNC_DISPLAY:-:99}"
  /usr/local/bin/start-novnc "$@"
}
alias novnc='start_novnc'
EOF_BASHRC

WORKDIR /mt-mohamad-nassif
