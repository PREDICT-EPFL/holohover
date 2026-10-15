#!/usr/bin/env python3
"""Puck model identification from a rosbag2 MCAP recording of the puck EKF state / OptiTrack pose.

    uv run identify.py segment BAG    automatic segmentation -> segments.yaml
    uv run identify.py review  BAG    check and correct the segments in a window
    uv run identify.py fit     BAG    fit the model -> results.yaml
    uv run identify.py plot    BAG    measured vs predicted, per segment, plus a summary
    uv run identify.py synth   OUT    write a synthetic bag with known parameters (to test the pipeline)

BAG is a bag directory or its .mcap file. See README.md.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# the fit works on many small arrays: BLAS threads only add overhead
for _var in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_var, "1")

import data  # noqa: E402


def _paths(args):
    _, out_dir = data.bag_paths(args.bag)
    return os.path.join(out_dir, "segments.yaml"), os.path.join(out_dir, "results.yaml")


def cmd_segment(args, cfg):
    import segment
    seg_file, _ = _paths(args)
    if os.path.exists(seg_file) and not args.force:
        sys.exit(f"{seg_file} exists (it may hold your reviewed segments). Use --force to overwrite.")
    rec = data.load_for_segmentation(args.bag, cfg)
    print(f"segmenting the {rec.kind} data")
    walls, events, segments = segment.auto_segment(rec, cfg)
    data.save_yaml(seg_file, segment.to_yaml(rec, walls, events, segments),
                   header="Puck identification segments, times in s from t_ref (first puck stamp).\n"
                          "Edit with `identify.py review`. Labels: glide, wall_impact, hit (not fitted).")
    kinds = {}
    for e in events:
        kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
    n = {k: sum(g["label"] == k for g in segments) for k in segment.LABELS}
    print(f"events: {kinds}")
    print(f"walls: {[w['name'] for w in walls]}")
    print(f"segments: {n['glide']} glides, {n['wall_impact']} wall impacts, {n['hit']} hits, "
          f"{sum(g['t1'] - g['t0'] for g in segments):.1f} s of {rec.t[-1]:.1f} s")
    print(f"wrote {seg_file}")


def cmd_review(args, cfg):
    import review
    seg_file, _ = _paths(args)
    if not os.path.exists(seg_file):
        sys.exit(f"{seg_file} not found, run `identify.py segment` first")
    review.run(data.load_for_segmentation(args.bag, cfg), seg_file, cfg)


def cmd_fit(args, cfg):
    import fit
    seg_file, res_file = _paths(args)
    if not os.path.exists(seg_file):
        sys.exit(f"{seg_file} not found, run `identify.py segment` first")
    rec = data.load(args.bag, cfg)
    pr, res = fit.run(rec, data.load_segments(seg_file, rec), cfg, verbose=args.verbose)
    out = fit.report(pr, res)
    out = {"bag": rec.source, "segments_file": seg_file, "config": cfg, **out}
    data.save_yaml(res_file, out, header="Puck identification results (identify.py fit).")
    print(fit.summary_text(out))
    print(f"wrote {res_file}")


def cmd_plot(args, cfg):
    import plot
    seg_file, res_file = _paths(args)
    for f in (seg_file, res_file):
        if not os.path.exists(f):
            sys.exit(f"{f} not found, run `identify.py segment` and `identify.py fit` first")
    rec = data.load(args.bag, cfg)
    save = args.save
    if save == "":
        save = os.path.join(os.path.dirname(res_file), "plots")
    res = data.load_yaml(res_file)
    if res.get("fit", {}).get("data", cfg["source"]) != cfg["source"]:
        sys.exit(f"{res_file} was fitted on {res['fit']['data']} data, the config says {cfg['source']}: fit again")
    plot.run(rec, data.load_segments(seg_file, rec), res, cfg, save_dir=save,
             segment_id=args.segment, horizon=args.horizon, every=args.every)


def cmd_synth(args, cfg):
    import synthetic
    synthetic.write(args.out, cfg, duration=args.duration, seed=args.seed)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="yaml file merged over config.yaml")
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("segment", help="automatic segmentation -> segments.yaml")
    p.add_argument("bag")
    p.add_argument("--force", action="store_true", help="overwrite an existing segments.yaml")
    p.set_defaults(func=cmd_segment)

    p = sub.add_parser("review", help="check and correct the segments in a window")
    p.add_argument("bag")
    p.set_defaults(func=cmd_review)

    p = sub.add_parser("fit", help="fit the model to the segments -> results.yaml")
    p.add_argument("bag")
    p.add_argument("-v", "--verbose", type=int, default=1, choices=[0, 1, 2], help="least_squares verbosity")
    p.set_defaults(func=cmd_fit)

    p = sub.add_parser("plot", help="measured vs model prediction")
    p.add_argument("bag")
    p.add_argument("--segment", type=int, help="start at this segment id")
    p.add_argument("--horizon", type=float, help="prediction horizon [s] (default from config)")
    p.add_argument("--every", type=float, help="time between predictions [s] (default from config)")
    p.add_argument("--save", nargs="?", const="", help="write PNGs instead of opening a window "
                   "(default directory: plots/ next to results.yaml)")
    p.set_defaults(func=cmd_plot)

    p = sub.add_parser("synth", help="write a synthetic bag with known parameters")
    p.add_argument("out", help="output bag directory")
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--seed", type=int, default=0)
    p.set_defaults(func=cmd_synth)

    args = ap.parse_args()
    args.func(args, data.load_config(args.config))


if __name__ == "__main__":
    main()
