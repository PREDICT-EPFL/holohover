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

# Python packages from pip (numpy stays the apt one, < 2, as Humble's message packages need)
COPY src/holohover_planner/requirements.txt /tmp/planner_requirements.txt
RUN apt-get update && apt-get install --no-install-recommends -y python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir -r /tmp/planner_requirements.txt \
    && rm /tmp/planner_requirements.txt

# Puck identification (holohover_simulator/identification): its own Python (>= 3.11, Humble has 3.10)
# and the locked packages of its uv.lock, in /opt so the mounted source never gets a root-owned .venv.
# Run with `identify <command> BAG`, see docker/scripts/identify.
ARG UV_VERSION=0.8.22
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python \
    UV_PROJECT_ENVIRONMENT=/opt/identification-venv
COPY src/holohover_simulator/identification/pyproject.toml src/holohover_simulator/identification/uv.lock /tmp/identification/
RUN pip install --no-cache-dir uv==${UV_VERSION} \
    && cd /tmp/identification && uv sync --frozen --no-cache \
    && rm -rf /tmp/identification
COPY docker/scripts/identify /usr/local/bin/identify
RUN chmod +x /usr/local/bin/identify

# Compile the ROS 2 packages of this repository
# Each C++ compiler process can take ~2 GB, so cap the parallelism (at most
# COLCON_WORKERS x MAKE_JOBS compilers at once); unbounded it exhausts RAM
# and freezes the host. Override with --build-arg on machines with more RAM.
ARG COLCON_WORKERS=2
ARG MAKE_JOBS=2
WORKDIR /root/ros2_ws
COPY src src/holohover
RUN source /opt/ros/humble/setup.bash \
    && MAKEFLAGS="-j${MAKE_JOBS}" colcon build --symlink-install \
        --parallel-workers ${COLCON_WORKERS}

ENTRYPOINT ["/usr/local/bin/holohover-entrypoint"]
CMD ["bash"]
