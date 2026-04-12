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
    ccache \
    curl \
    git \
    iproute2 \
    iputils-ping \
    inotify-tools \
    python3-can \
    python3-colcon-common-extensions \
    python3-pip \
    python3-pyinotify \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
    rsync \
    screen \
    ssh \
    sudo \
    zsh \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-forward-command-controller \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joy-linux \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-xacro && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --no-cache-dir --ignore-installed cantools

RUN git clone https://github.com/norlab-ulaval/libnabo.git /tmp/libnabo && \
    cd /tmp/libnabo && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/libnabo

RUN git clone https://github.com/norlab-ulaval/libpointmatcher.git /tmp/libpointmatcher && \
    cd /tmp/libpointmatcher && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/libpointmatcher && \
    ldconfig

RUN git clone --branch 2.1.1 https://github.com/norlab-ulaval/norlab_icp_mapper.git /tmp/norlab_icp_mapper && \
    cd /tmp/norlab_icp_mapper && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/norlab_icp_mapper && \
    ldconfig


RUN existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)" && \
    existing_group="$(getent group "${USER_GID}" | cut -d: -f1 || true)" && \
    if [ -n "${existing_group}" ] && [ "${existing_group}" != "${USERNAME}" ]; then \
      groupmod -n "${USERNAME}" "${existing_group}"; \
    elif ! getent group "${USERNAME}" >/dev/null; then \
      groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi && \
    if [ -n "${existing_user}" ] && [ "${existing_user}" != "${USERNAME}" ]; then \
      usermod -l "${USERNAME}" "${existing_user}" && \
      usermod -d "/home/${USERNAME}" -m "${USERNAME}"; \
    elif ! id -u "${USERNAME}" >/dev/null 2>&1; then \
      useradd --uid "${USER_UID}" --gid "${USERNAME}" -m -s /bin/bash "${USERNAME}"; \
    fi && \
    usermod --gid "${USERNAME}" "${USERNAME}" && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN rosdep init 2>/dev/null || true

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
COPY docker/ros_setup.sh /ros_setup.sh
RUN chmod +x /ros_entrypoint.sh /ros_setup.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

FROM base AS devel

WORKDIR /workspaces/mtt_tools
COPY . /workspaces/mtt_tools

RUN bash -lc ' \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    mapfile -t workspace_paths < <(bash ./scripts/workspace_source_paths) && \
    apt-get update && \
    rosdep install -iyr --from-paths "${workspace_paths[@]}" --ignore-src \
'

ARG USERNAME=mtt
USER ${USERNAME}

FROM devel AS builder

RUN bash -lc ' \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    mapfile -t workspace_paths < <(bash ./scripts/workspace_source_paths) && \
    colcon build --base-paths "${workspace_paths[@]}" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
'
