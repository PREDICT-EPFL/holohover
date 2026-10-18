#!/usr/bin/env python3
"""Print the messages of a rosbag / python-sim MCAP around an elapsed time, as compact tables.

Elapsed time is counted from the first message, like the Foxglove timeline. Runs on the host with
the python-sim venv (it has mcap and mcap-ros2-support):

  PY=src/holohover_simulator/python-sim/.venv/bin/python
  $PY src/holohover_utils/scripts/bag_window.py docker/data/rosbags/air_hockey_20261018_073253 --list
  $PY src/holohover_utils/scripts/bag_window.py <bag> --t 29.2 --dt 0.2 --topics /puck/state,/h0/plan
  $PY src/holohover_utils/scripts/bag_window.py <bag> --episode 38 --out before_ep38.mcap   # one evaluation episode
"""
import argparse
import glob
import json
import os
from collections import Counter, defaultdict

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_ros2.decoder import DecoderFactory


def flatten(obj, prefix="", out=None, depth=0):
    """Message -> {dotted field: value}; arrays longer than 4 are shown as their length."""
    out = {} if out is None else out
    if isinstance(obj, dict):
        items = obj.items()
    elif hasattr(obj, "__slots__") or hasattr(obj, "__dict__"):
        items = ((a, getattr(obj, a)) for a in dir(obj) if not a.startswith("_") and not callable(getattr(obj, a)))
    else:
        out[prefix] = obj
        return out
    for key, value in items:
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(value, (list, tuple)) and (len(value) > 4 or any(not isinstance(v, (int, float)) for v in value)):
            out[name] = f"[{len(value)}]"
        elif depth < 5 and not isinstance(value, (int, float, str, bytes, list, tuple)):
            flatten(value, name, out, depth + 1)
        else:
            out[name] = value
    return out


def fmt(value):
    if isinstance(value, float):
        return f"{value:.4g}"
    if isinstance(value, (list, tuple)):
        return "[" + ",".join(fmt(v) for v in value) + "]"
    return str(value)


def messages(path):
    """Yield (channel, log_time_ns, decoded message) over every .mcap file of a bag."""
    files = bag_files(path)
    factory, decoders = DecoderFactory(), {}
    for file in files:
        with open(file, "rb") as f:
            for schema, channel, message in make_reader(f).iter_messages(log_time_order=True):
                if channel.message_encoding == "json":
                    yield channel, message.log_time, json.loads(message.data)
                    continue
                if channel.id not in decoders:
                    decoders[channel.id] = factory.decoder_for(channel.message_encoding, schema)
                decoder = decoders[channel.id]
                yield channel, message.log_time, decoder(message.data) if decoder else message.data


def bag_files(path):
    return sorted(glob.glob(os.path.join(path, "*.mcap"))) if os.path.isdir(path) else [path]


def episode_window(path, episode, name="h0"):
    """(start, end) log times (ns) of an evaluation episode: from 0.5 s before its first /<name>/episode
    message (the evaluator's resets) to the first message of the next episode, or the bag's end."""
    start = end = None
    for file in bag_files(path):
        with open(file, "rb") as f:
            for _, _, message in make_reader(f).iter_messages(topics=[f"/{name}/episode"], log_time_order=True):
                e = int.from_bytes(message.data[4:8], "little", signed=True)  # CDR: 4-byte header, then the int32
                if e == episode and start is None:
                    start = message.log_time - int(0.5e9)
                elif start is not None and e >= 0 and e != episode:
                    return start, message.log_time
    if start is None:
        raise SystemExit(f"no episode {episode} in {path}")
    return start, end


def write_window(path, out, start, end):
    """Copy the messages logged in [start, end] (ns, end None: to the bag's end) to the MCAP file out."""
    with open(out, "wb") as f_out:
        writer, schemas, channels = Writer(f_out), {}, {}
        writer.start()
        for file in bag_files(path):
            with open(file, "rb") as f:
                for schema, channel, message in make_reader(f).iter_messages(
                        start_time=start, end_time=None if end is None else end + 1, log_time_order=True):
                    if schema is not None and schema.id not in schemas:
                        schemas[schema.id] = writer.register_schema(schema.name, schema.encoding, schema.data)
                    if channel.id not in channels:
                        channels[channel.id] = writer.register_channel(
                            channel.topic, channel.message_encoding, schemas.get(schema.id, 0) if schema else 0,
                            channel.metadata)
                    writer.add_message(channels[channel.id], message.log_time, message.data, message.publish_time,
                                       message.sequence)
        writer.finish()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag", help="rosbag directory or .mcap file")
    parser.add_argument("--list", action="store_true", help="list topics, message counts and duration")
    parser.add_argument("--t", type=float, help="centre of the window, elapsed seconds")
    parser.add_argument("--dt", type=float, default=0.3, help="half width of the window, seconds (default 0.3)")
    parser.add_argument("--topics", help="comma-separated topics (default: all)")
    parser.add_argument("--fields", help="comma-separated substrings; keep only matching fields")
    parser.add_argument("--episode", type=int, help="with --out: the window of that evaluation episode")
    parser.add_argument("--out", help="write the window (--t/--dt or --episode) to this MCAP file, all topics")
    args = parser.parse_args()
    if args.out:
        if args.episode is not None:
            start, end = episode_window(args.bag, args.episode)
        elif args.t is not None:
            first = min(make_reader(open(f, "rb")).get_summary().statistics.message_start_time for f in bag_files(args.bag))
            start, end = first + int((args.t - args.dt) * 1e9), first + int((args.t + args.dt) * 1e9)
        else:
            parser.error("--out needs --episode or --t")
        write_window(args.bag, args.out, start, end)
        return
    if not args.list and args.t is None:
        parser.error("give --list or --t")
    topics = set(args.topics.split(",")) if args.topics else None
    fields = args.fields.split(",") if args.fields else None

    start, counts, last, rows = None, Counter(), 0.0, defaultdict(list)
    for channel, log_time, msg in messages(args.bag):
        start = log_time if start is None else start
        t = (log_time - start) * 1e-9
        if args.list:
            counts[channel.topic] += 1
            last = t
            continue
        if t > args.t + args.dt:
            break
        if t >= args.t - args.dt and (topics is None or channel.topic in topics):
            flat = flatten(msg)
            if fields:
                flat = {k: v for k, v in flat.items() if any(s in k for s in fields)}
            rows[channel.topic].append((t, flat))

    if args.list:
        print(f"duration {last:.2f} s")
        for topic, n in sorted(counts.items()):
            print(f"  {n:7d}  {topic}")
        return
    for topic, entries in rows.items():
        keys = list(dict.fromkeys(k for _, flat in entries for k in flat))
        print(f"== {topic} ({len(entries)} msgs)")
        print("t\t" + "\t".join(keys))
        for t, flat in entries:
            print(f"{t:.3f}\t" + "\t".join(fmt(flat.get(k, "")) for k in keys))


if __name__ == "__main__":
    main()
