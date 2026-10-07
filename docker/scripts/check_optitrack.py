#!/usr/bin/env python3
"""Checks that OptiTrack Motive is streaming NatNet data on the multicast group of the mocap config."""
import argparse
import socket
import struct
import sys
import time

import yaml

DEFAULT_CONFIG = "/root/ros2_ws/src/holohover/holohover_utils/config/optitrack/mocap_config.yaml"
NATNET_FRAME_OF_DATA = 7


def read_config(path):
    with open(path) as f:
        params = yaml.safe_load(f)["mocap_node"]["ros__parameters"]["optitrack_config"]
    return params["multicast_address"], int(params["data_port"])


def listen(group, port, timeout):
    """Returns (motive_ip, natnet_message_id) of the first packet received, or None on timeout."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    mreq = struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton("0.0.0.0"))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(timeout)
    try:
        data, (ip, _) = sock.recvfrom(65535)
    except socket.timeout:
        return None
    finally:
        sock.close()
    msg_id = struct.unpack("<H", data[:2])[0] if len(data) >= 2 else None
    return ip, msg_id


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="mocap_node parameter file")
    parser.add_argument("--timeout", type=float, default=5.0, help="Seconds to wait for data")
    parser.add_argument("--wait", action="store_true", help="Keep retrying until Motive is streaming")
    args = parser.parse_args()

    group, port = read_config(args.config)
    while True:
        print(f"Listening for OptiTrack NatNet data on {group}:{port} ...", flush=True)
        result = listen(group, port, args.timeout)
        if result is not None:
            ip, msg_id = result
            kind = "frame data" if msg_id == NATNET_FRAME_OF_DATA else f"message id {msg_id}"
            print(f"OptiTrack Motive is streaming from {ip} ({kind}).", flush=True)
            return 0
        print(f"No OptiTrack data received within {args.timeout:g}s. Check that Motive is running and that\n"
              f"  Streaming > NatNet is enabled, with multicast interface {group} and data port {port},\n"
              f"  and that this PC is on the OptiTrack network.", flush=True)
        if not args.wait:
            return 1
        time.sleep(1)


if __name__ == "__main__":
    sys.exit(main())
