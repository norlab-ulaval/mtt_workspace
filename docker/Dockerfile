# syntax=docker/dockerfile:1
# ══════════════════════════════════════════════════════════════════════════════
# mtt_workspace:devel — fast workspace image
#
# Requires mtt_workspace:base to be built first (once):
#   docker build -f docker/Dockerfile.base -t mtt_workspace:base .
#
# Shortcut reusing existing built image (zero cost):
#   docker tag mtt_workspace:devel mtt_workspace:base
#
# Daily workflow — adding/removing a ROS package:
#   Edit Layer 7 below, then: dc build   (~3-5 min)
#
# Layer costs from mtt_workspace:base:
#   Layer 7  ROS packages   ~2-3 min (Canadian mirror, lists cached)
#   Layer 8  pip            ~20s
#   rosdep   workspace deps ~30s (cached when package.xml unchanged)
#   colcon   build          ~5-15 min (cached when source unchanged)
# ══════════════════════════════════════════════════════════════════════════════

ARG BASE_IMAGE=mtt_workspace:base
ARG ROS_DISTRO=jazzy

FROM ${BASE_IMAGE} AS base

# Base image may have USER mtt set as last instruction — switch to root for installs.
USER root

ARG ROS_DISTRO

# --- Layer 7: ROS packages (edit here — only busts layers 8+ not CUDA/ZED) ---
#
# Network hardening: force IPv4 and use HTTPS Ubuntu archives.
# This avoids stalls on HTTP/mirror resolution in some environments.
#
# apt list cache: --mount=type=cache,target=/var/lib/apt/lists caches the package
# index across builds so repeated apt-get updates only download the diff (~1 MB)
# instead of the full 43 MB index every time.
#
# Full package list — install all packages needed by the workspace.
# apt-get install skips already-installed packages in seconds,
# so this list is correct whether building from a clean Dockerfile.base
# or from a tagged old image that has most packages already.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4 && \
    sed -i \
        's|http://archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g; \
         s|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g; \
         s|http://ca.archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g' \
        /etc/apt/sources.list.d/*.sources /etc/apt/sources.list 2>/dev/null || true && \
    apt-get update && apt-get install -y --no-install-recommends \
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
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-gtsam \
    ros-${ROS_DISTRO}-nmea-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-ros2-socketcan

# --- Layer 8: pip ---
RUN pip3 install --break-system-packages --no-cache-dir --ignore-installed cantools && \
    chmod -R a+rX /usr/local/lib/python3.12/dist-packages/

# ══════════════════════════════════════════════════════════════════════════════
# devel stage — workspace source + colcon build
#
# Cache isolation for rosdep:
#   1. pkg_manifests stage extracts ONLY package.xml files from src/
#   2. rosdep layer rebuilds only when ROS dependencies change, not on .cpp/.hpp edits
#   3. Full source copy + colcon come after, so source edits don't bust rosdep
# ══════════════════════════════════════════════════════════════════════════════

FROM base AS pkg_manifests
COPY src/ /src/
RUN find /src -name "package.xml" | \
    while IFS= read -r f; do install -D "$f" "/manifests/${f#/src/}"; done

FROM base AS devel

WORKDIR /workspaces/mtt_workspace

# Step 1: package manifests only — rosdep layer stable across source edits
COPY --from=pkg_manifests /manifests ./src/
COPY scripts/workspace_source_paths ./scripts/workspace_source_paths

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    bash -lc ' \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        rosdep update && \
        mapfile -t workspace_paths < <(bash ./scripts/workspace_source_paths) && \
        apt-get update && \
        rosdep install -iyr --from-paths "${workspace_paths[@]}" --ignore-src \
    '

# Step 2: full source — busts colcon build but not rosdep above
COPY . .

ARG USERNAME=mtt
USER ${USERNAME}

FROM devel AS builder

RUN bash -lc ' \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    mapfile -t workspace_paths < <(bash ./scripts/workspace_source_paths) && \
    colcon build --base-paths "${workspace_paths[@]}" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
'
