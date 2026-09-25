FROM ubuntu:24.04

COPY --from=ghcr.io/astral-sh/uv:0.12.14 /uv /uvx /bin/

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
	&& apt-get install -y --no-install-recommends \
		ffmpeg \
		python3 \
		python3-pip \
		python3-tk \
		python3-venv \
	&& rm -rf /var/lib/apt/lists/*

COPY holohover_simulator/python-sim/pyproject.toml holohover_simulator/python-sim/uv.lock /tmp/python-sim/
RUN uv venv /opt/venv --python python3 \
	&& uv export --project /tmp/python-sim --locked --format requirements.txt --output-file /tmp/python-sim-requirements.txt \
	&& uv pip install --python /opt/venv --no-cache --requirement /tmp/python-sim-requirements.txt \
	&& rm -rf /tmp/python-sim

ENV VIRTUAL_ENV=/opt/venv \
	PATH="/opt/venv/bin:${PATH}"

WORKDIR /root/ros2_ws/holohover_simulator/python-sim
