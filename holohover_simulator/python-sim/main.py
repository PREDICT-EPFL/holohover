"""
Holohover python simulation: robot 1 (right, attacks the left goal) against robot 2 on the
air-hockey table, robot 1 driven by the DIAL planner, the shot MPC or the forward example
(config "controller", or --controller). Run from python-sim/:

    python main.py simulate                                   # config scenario "default", opened in Foxglove
    python main.py simulate --controller dial --scenario incoming --no-view
    python main.py evaluate --episodes 50                     # the stored random starts: conversion rate
    python main.py evaluate --mcap --outcomes miss            # the misses, in one MCAP file
    python main.py evaluate --first 37 --episodes 1 --mcap    # rerun start 37 only
    python main.py evaluate --input-delay 3                   # robot 1's inputs applied 3 steps (30 ms) late
    python main.py evaluate --input-delay 3 --compensate      # shot_mpc plans from the state 3 steps ahead
    python main.py view [simulation_data/<run>.mcap]          # open a saved run (default: the newest)
    python main.py starts --count 1000 --seed 0 --force       # redraw the stored starts

A run is saved to simulation_data/<controller>_<scenario>_<stamp>.mcap, with its config and log
attached. simulate then serves it to the Foxglove web app until Ctrl-C (viz/foxglove_open.py).
evaluate --mcap saves its episodes one after the other to simulation_data/<controller>_eval_<stamp>.mcap.

evaluate always runs the same starts, read from the file config "evaluation.starts": episode i
starts from its start i. starts draws that file (config "evaluation.sampling").
"""

import argparse
from datetime import datetime
from pathlib import Path

import controllers
from simulation.episode import episode_log, run_episode
from simulation.setup import (generate_starts, initial_state, load_config, load_starts, physics_from_config,
                              start_state, write_starts)
from viz.foxglove_open import open_in_foxglove
from viz.foxglove_viz import McapWriter, write_mcap

DATA_DIR = Path("simulation_data")


def simulate(config, factory, physics, scenario):
    """One episode of config["simulation"]["duration"] from a config scenario, saved to an MCAP file
    whose path is returned."""
    init = initial_state(config, scenario)
    ctrl = factory.new(physics, init, verbose=True)
    result = run_episode(config, physics, ctrl, init, config["simulation"]["duration"])

    print(f"\nOutcome: {result['outcome']}"
          + (f" at t={result['t_goal']:.2f}s" if result["t_goal"] else "")
          + f" | contacts at {[round(float(c), 2) for c in result['contacts']]}")
    summary = factory.summary([result["log"]])
    if summary:
        print(summary)

    now = datetime.now()
    path = DATA_DIR / f"{config['controller']}_{scenario or 'default'}_{now:%Y%m%d_%H%M%S}.mcap"
    write_mcap(result["data"], path, log=episode_log(result),
               metadata={"scenario": scenario or "default", "outcome": result["outcome"]},
               start_ns=int(now.timestamp() * 1e9))
    print(f"Saved to {path}")
    return path


def evaluate(config, factory, physics, starts, first, mcap=False, outcomes=None):
    """Episodes from `starts` (evaluation starts, numbered from `first`), each stopped shortly
    after the first goal. With mcap, every episode (or only those whose outcome is in `outcomes`)
    is saved to simulation_data/<controller>_eval_<stamp>.mcap, one after the other."""
    duration = config["evaluation"]["duration"]
    episodes = len(starts)
    counts, logs, first_shot = {}, [], 0
    writer = McapWriter(DATA_DIR / f"{config['controller']}_eval_{datetime.now():%Y%m%d_%H%M%S}.mcap",
                        metadata={"starts": config["evaluation"]["starts"]})  # file created at its first episode
    try:
        for e, start in enumerate(starts, first):
            init = start_state(config, start)
            res = run_episode(config, physics, factory.new(physics, init), init, duration, stop_on_goal=True,
                              record=mcap, tail=0.5 if mcap else 0.0)
            outcome = res["outcome"]
            counts[outcome] = counts.get(outcome, 0) + 1
            first_shot += outcome == "goal" and res["contacts_at_goal"] == 1
            logs.append(res["log"])
            print(f"episode {e:3d}: {outcome:<10} contacts={len(res['contacts'])}")
            if mcap and (not outcomes or outcome in outcomes):
                writer.add(res["data"], log=episode_log(res), episode=e)
    finally:
        writer.close()
        if writer.episodes:
            print(f"{len(writer.episodes)} episodes saved to {writer.path}")

    goals = counts.get("goal", 0)
    print(f"\nConversion: {goals}/{episodes} = {100 * goals / episodes:.0f}% "
          f"(first-shot goals: {first_shot}) | outcomes: {counts}")
    summary = factory.summary(logs)
    if summary:
        print(summary)


def newest_run():
    runs = list(DATA_DIR.glob("*.mcap"))
    if not runs:
        raise FileNotFoundError(f"no .mcap file in {DATA_DIR}/")
    return max(runs, key=lambda f: f.stat().st_mtime)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)

    def run_parser(name, help):
        p = commands.add_parser(name, help=help)
        p.add_argument("--config", default="config/config.json")
        p.add_argument("--controller", choices=controllers.CONTROLLERS, help="default: the config's")
        p.add_argument("--opponent", choices=["none", "static"],
                       help="robot 2 parked in a corner, or holding its home (default: the config's)")
        p.add_argument("--input-delay", type=int, metavar="STEPS",
                       help="control steps between robot 1's input and its application (default: the config's)")
        shot = p.add_argument_group("shot_mpc")
        shot.add_argument("--workers", type=int, help="solver processes (default shot_mpc.n_workers; 0: serial)")
        shot.add_argument("--no-tracker", action="store_true", help="apply the plan's inputs open loop")
        shot.add_argument("--compensate", nargs="?", type=int, const=True, metavar="STEPS",
                          help="plan from the state predicted STEPS control steps ahead "
                               "(shot_mpc.delay_compensation; without STEPS: the input delay)")
        return p

    p = run_parser("simulate", "run one episode from a config scenario")
    p.add_argument("--scenario", help="a config scenario (default: default)")
    p.add_argument("--no-view", action="store_true", help="don't open the run in Foxglove")

    p = run_parser("evaluate", "run episodes from the stored random starts and report the conversion rate")
    p.add_argument("--episodes", type=int, default=50)
    p.add_argument("--first", type=int, default=0, help="index of the first start")
    p.add_argument("--mcap", action="store_true", help="save the episodes to one MCAP file")
    p.add_argument("--outcomes", nargs="+", help="with --mcap: only save these outcomes, e.g. miss no_contact")

    p = commands.add_parser("view", help="open a saved run in Foxglove")
    p.add_argument("mcap", nargs="?", help="default: the newest simulation_data/*.mcap")
    p.add_argument("--no-browser", action="store_true", help="only print the link")

    p = commands.add_parser("starts", help="draw the random starts evaluate runs (config evaluation.sampling)")
    p.add_argument("--config", default="config/config.json")
    p.add_argument("--count", type=int, default=1000)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--force", action="store_true", help="replace the existing file")

    args = parser.parse_args(argv)

    if args.command == "view":
        open_in_foxglove(args.mcap or newest_run(), browser=not args.no_browser)
        return

    config = load_config(args.config)
    if args.command == "starts":
        path = Path(config["evaluation"]["starts"])
        if path.exists() and not args.force:
            parser.error(f"{path} exists: evaluation results are only comparable on the same starts (--force replaces it)")
        write_starts(path, generate_starts(config, args.count, args.seed))
        print(f"{args.count} starts (seed {args.seed}) written to {path}")
        return
    if args.controller:
        config["controller"] = args.controller
    if args.opponent:
        config["simulation"]["opponent"] = args.opponent
    if args.input_delay is not None:
        if args.input_delay < 0:
            parser.error("--input-delay: 0 or more steps")
        config["simulation"]["input_delay"] = args.input_delay
    if args.no_tracker:
        config["shot_mpc"]["tracker"]["enabled"] = False
    if args.compensate is not None:
        if args.compensate is True:  # no STEPS (argparse would pass a str const through int)
            args.compensate = config["simulation"]["input_delay"]
        if args.compensate < 0:
            parser.error("--compensate: 0 or more steps")
        config["shot_mpc"]["delay_compensation"] = args.compensate
    if args.command == "simulate" and args.scenario and args.scenario not in config["scenarios"]:
        parser.error(f"--scenario: one of {', '.join(config['scenarios'])}")
    if args.command == "evaluate":
        starts = load_starts(config["evaluation"]["starts"])
        if not 0 <= args.first < args.first + args.episodes <= len(starts):
            parser.error(f"--first/--episodes: {len(starts)} starts in {config['evaluation']['starts']}")
        starts = starts[args.first:args.first + args.episodes]

    factory = controllers.load(config, args.workers)  # before JAX starts: see controllers.load
    physics = physics_from_config(config)
    try:
        if args.command == "simulate":
            path = simulate(config, factory, physics, args.scenario)
        else:
            evaluate(config, factory, physics, starts, args.first, mcap=args.mcap, outcomes=args.outcomes)
    finally:
        factory.close()
    if args.command == "simulate":
        if args.no_view:
            print(f"Open it in Foxglove: python main.py view {path}")
        else:
            open_in_foxglove(path)


if __name__ == "__main__":
    main()
