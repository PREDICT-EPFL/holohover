# syntax=docker/dockerfile:1
# Holohover ROS 2 workspace image (amd64 / PC).
# Build context is the repository root, see docker/docker-compose.yml.
# Build args are set in docker/.env and passed by docker-compose.yml (no defaults here: single source of truth;
# compose checks they are set). Each ARG is declared just before its first use so changing it only rebuilds
# from there: ROS_DISTRO (base image), QUILL/CASADI/PIQP_VERSION (third-party libs), MAKE_JOBS (third-party libs + colcon), COLCON_WORKERS (colcon).
ARG ROS_DISTRO
FROM osrf/ros:${ROS_DISTRO}-desktop
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
# Ubuntu 26.04 enforces PEP 668; allow pip (and rosdep's pip keys) system-wide.
# Fine in a single-purpose container: pip installs to /usr/local, apt's files are untouched
ENV PIP_BREAK_SYSTEM_PACKAGES=1
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
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
        xterm \
    && rm -rf /var/lib/apt/lists/*

# Third-party libraries built from source (pinned to the versions of the previous holohover image)
ARG QUILL_VERSION
ARG CASADI_VERSION
# Empty = skipped (see docker/.env); holohover_common needs PIQP
ARG PIQP_VERSION
# Parallel compilers per build, also caps the colcon build below (each compiler can take ~2 GB of RAM)
ARG MAKE_JOBS
COPY docker/scripts/install_deps.sh /tmp/install_deps.sh
RUN QUILL_VERSION=${QUILL_VERSION} CASADI_VERSION=${CASADI_VERSION} PIQP_VERSION=${PIQP_VERSION} \
        MAKE_JOBS=${MAKE_JOBS} bash /tmp/install_deps.sh \
    && rm /tmp/install_deps.sh

COPY docker/scripts/entrypoint.sh /usr/local/bin/holohover-entrypoint
COPY docker/scripts/setup.bash /root/setup.bash
COPY docker/scripts/check_optitrack.py /usr/local/bin/check-optitrack
RUN chmod +x /usr/local/bin/holohover-entrypoint /usr/local/bin/check-optitrack \
    && echo "source /root/setup.bash" >> /root/.bashrc

# Python packages from pip (numpy stays the apt one)
COPY src/holohover_planner/requirements.txt /tmp/planner_requirements.txt
RUN apt-get update && apt-get install --no-install-recommends -y python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir -r /tmp/planner_requirements.txt \
    && rm /tmp/planner_requirements.txt

# Compile the ROS 2 packages of this repository
# Each C++ compiler process can take ~2 GB, so cap the parallelism (at most
# COLCON_WORKERS x MAKE_JOBS compilers at once); unbounded it exhausts RAM
# and freezes the host. Override with --build-arg on machines with more RAM.
ARG COLCON_WORKERS
WORKDIR /root/ros2_ws
COPY src src/holohover
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && MAKEFLAGS="-j${MAKE_JOBS}" colcon build --symlink-install \
        --parallel-workers ${COLCON_WORKERS}

ENTRYPOINT ["/usr/local/bin/holohover-entrypoint"]
CMD ["bash"]
