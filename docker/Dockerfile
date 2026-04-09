ARG FROM_IMAGE=docker.io/osrf/ros:jazzy-desktop
ARG ROS_DISTRO=jazzy

FROM ${FROM_IMAGE} AS base

ARG ROS_DISTRO
ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ARG USERNAME=mtt
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} \
    WORKSPACE=/workspaces/mtt_tools \
    SHELL=/bin/bash

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    can-utils \
    curl \
    git \
    iproute2 \
    iputils-ping \
    python3-can \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pyinotify \
    python3-rosdep \
    python3-vcstool \
    rsync \
    screen \
    ssh \
    sudo \
    zsh \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-forward-command-controller \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joy-linux \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-xacro && \
    rm -rf /var/lib/apt/lists/*

RUN if ! getent group "${USER_GID}" >/dev/null; then groupadd --gid "${USER_GID}" "${USERNAME}"; fi && \
    if ! id -u "${USERNAME}" >/dev/null 2>&1; then \
      useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}"; \
    fi && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN rosdep init 2>/dev/null || true

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

FROM base AS devel

ARG USERNAME=mtt
WORKDIR /workspaces/mtt_tools
USER ${USERNAME}

FROM base AS builder

WORKDIR /workspaces/mtt_tools
COPY . /workspaces/mtt_tools

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install -iyr --from-paths . --ignore-src && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

