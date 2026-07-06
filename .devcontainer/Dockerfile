FROM osrf/ros:jazzy-desktop-full

# 仅替换 Ubuntu 24.04 (noble) 源为阿里云，加速 apt-get update；并安装本项目运行/调试依赖。
RUN set -eux; \
    sed -i 's@http://.*archive.ubuntu.com/ubuntu/@https://mirrors.aliyun.com/ubuntu/@g' /etc/apt/sources.list; \
    sed -i 's@http://.*security.ubuntu.com/ubuntu/@https://mirrors.aliyun.com/ubuntu/@g' /etc/apt/sources.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        gdb \
        x11-apps \
        curl \
        unzip \
        procps \
        libxext6 \
        libxrender1 \
        libxtst6 \
        libxi6 \
        libfreetype6 \
        ros-jazzy-slam-toolbox \
        ros-jazzy-octomap \
        ros-jazzy-octomap-msgs \
        ros-jazzy-octomap-server \
        ros-jazzy-octomap-rviz-plugins \
        ros-jazzy-launch-testing-ament-cmake; \
    rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8

WORKDIR /workspaces/alien-scanner
