# syntax=docker/dockerfile:1
# Holohover ROS 2 workspace image (amd64 / PC).
# Build context is the repository root, see docker/docker-compose.yml.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
        build-essential \
        cmake \
        git \
        libbox2d-dev \
        libeigen3-dev \
        libgpiod-dev \
        libmatio-dev \
        libyaml-cpp-dev \
        python3-pyqtgraph \
        ros-humble-foxglove-bridge \
        ros-humble-rosbag2-storage-mcap \
        xterm \
    && rm -rf /var/lib/apt/lists/*

# Third-party libraries built from source (pinned to the versions of the previous holohover image)
ARG QUILL_VERSION=v4.4.1
ARG CASADI_VERSION=582be5d9d0e8a9854951e9a11e2b033fa924e1ee
ARG PIQP_VERSION=a9e2cd91bb378223723cecf4ac8a628ef252fe04
COPY docker/scripts/install_deps.sh /tmp/install_deps.sh
RUN QUILL_VERSION=${QUILL_VERSION} CASADI_VERSION=${CASADI_VERSION} PIQP_VERSION=${PIQP_VERSION} \
        bash /tmp/install_deps.sh \
    && rm /tmp/install_deps.sh

COPY docker/scripts/entrypoint.sh /usr/local/bin/holohover-entrypoint
COPY docker/scripts/setup.bash /root/setup.bash
COPY docker/scripts/check_optitrack.py /usr/local/bin/check-optitrack
RUN chmod +x /usr/local/bin/holohover-entrypoint /usr/local/bin/check-optitrack \
    && echo "source /root/setup.bash" >> /root/.bashrc

# Compile the ROS 2 packages of this repository
WORKDIR /root/ros2_ws
COPY . src/holohover
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

ENTRYPOINT ["/usr/local/bin/holohover-entrypoint"]
CMD ["bash"]
