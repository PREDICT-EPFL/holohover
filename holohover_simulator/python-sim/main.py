"""
Holohover python simulation: robot 1 (right, attacks the left goal) against robot 2 on the
air-hockey table, robot 1 driven by the DIAL planner or the shot MPC (config "controller", or
--controller). Run from python-sim/:

    python main.py simulate                                   # config scenario "default", animated
    python main.py simulate --controller dial --scenario incoming --mcap --no-animate
    python main.py evaluate --episodes 50 --seed 0            # random starts: conversion rate
    python main.py animate [simulation_data/<run>.pkl] [--mode dial] [--gif NAME]
    python main.py mcap simulation_data/<run>.pkl             # Foxglove file of a saved run

A run is saved to simulation_data/<controller>_<scenario>_<stamp>.pkl (the data the viewers read,
simulation.episode.episode_data) with its log <...>_log.json, and with --mcap <...>.mcap.
GIFs go to gifs/<NAME>.gif.
"""

import argparse
import json
import pickle
from datetime import datetime
from pathlib import Path

import numpy as np

import controllers
from simulation.episode import episode_log, run_episode
from simulation.setup import initial_state, load_config, physics_from_config, sample_initial_state

DATA_DIR = Path("simulation_data")


def simulate(config, factory, physics, scenario, animate=True, gif=None, mcap=False, save=True):
    """One episode of config["simulation"]["duration"] from a config scenario."""
    init = initial_state(config, scenario)
    ctrl = factory.new(physics, init, verbose=True)
    result = run_episode(config, physics, ctrl, init, config["simulation"]["duration"])

    print(f"\nOutcome: {result['outcome']}"
          + (f" at t={result['t_goal']:.2f}s" if result["t_goal"] else "")
          + f" | contacts at {[round(float(c), 2) for c in result['contacts']]}")
    summary = factory.summary([result["log"]])
    if summary:
        print(summary)

    data, log = result["data"], episode_log(result)
    now = datetime.now()
    base = DATA_DIR / f"{config['controller']}_{scenario or 'default'}_{now:%Y%m%d_%H%M%S}"
    if save:
        save_run(data, log, base.with_suffix(".pkl"))
    if mcap:
        from viz.foxglove_viz import write_mcap

        write_mcap(data, base.with_suffix(".mcap"), log=log,
                   metadata={"scenario": scenario or "default", "outcome": result["outcome"]},
                   start_ns=int(now.timestamp() * 1e9))
        print(f"Saved Foxglove MCAP to {base.with_suffix('.mcap')}")
    if gif:
        from viz.animation import export_gif

        export_gif(data, gif)
    elif animate:
        from viz.animation import play

        play(data)


def evaluate(config, factory, physics, episodes, seed, gif=None, mcap=False, outcomes=None):
    """Episodes from random starts (config "evaluation"), each stopped shortly after the first
    goal. With gif or mcap, every episode (or only those whose outcome is in `outcomes`) is
    recorded to gifs/<gif>_ep<i>_<outcome>.gif or simulation_data/<controller>_eval_<stamp>_ep<i>_<outcome>.mcap."""
    rng = np.random.default_rng(seed)
    duration = config["evaluation"]["duration"]
    record = gif is not None or mcap
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    counts, logs, first_shot = {}, [], 0
    for e in range(episodes):
        init = sample_initial_state(rng, config)
        res = run_episode(config, physics, factory.new(physics, init), init, duration, stop_on_goal=True,
                          record=record, tail=0.5 if record else 0.0)
        outcome = res["outcome"]
        counts[outcome] = counts.get(outcome, 0) + 1
        first_shot += outcome == "goal" and res["contacts_at_goal"] == 1
        logs.append(res["log"])
        print(f"episode {e:3d}: {outcome:<10} contacts={len(res['contacts'])}")
        if record and (not outcomes or outcome in outcomes):
            if gif:
                from viz.animation import export_gif, gif_name

                export_gif(res["data"], f"{gif_name(gif)}_ep{e:03d}_{outcome}")
            if mcap:
                from viz.foxglove_viz import write_mcap

                write_mcap(res["data"], DATA_DIR / f"{config['controller']}_eval_{stamp}_ep{e:03d}_{outcome}.mcap",
                           log=episode_log(res), metadata={"seed": seed, "episode": e, "outcome": outcome})

    goals = counts.get("goal", 0)
    print(f"\nConversion: {goals}/{episodes} = {100 * goals / episodes:.0f}% "
          f"(first-shot goals: {first_shot}) | outcomes: {counts}")
    summary = factory.summary(logs)
    if summary:
        print(summary)


def save_run(data, log, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        pickle.dump(data, f)
    with open(log_path(path), "w") as f:
        json.dump(log, f, indent=1, default=float)
    print(f"Saved to {path}")


def log_path(pkl):
    return pkl.with_name(pkl.stem + "_log.json")


def load_run(path=None):
    """A saved run (default: the newest) and its log, or None if it has none."""
    if path is None:
        runs = list(DATA_DIR.glob("*.pkl"))
        if not runs:
            raise FileNotFoundError(f"no .pkl file in {DATA_DIR}/")
        path = max(runs, key=lambda f: f.stat().st_mtime)
    path = Path(path)
    with open(path, "rb") as f:
        data = pickle.load(f)
    log = json.loads(log_path(path).read_text()) if log_path(path).exists() else None
    return path, data, log


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)

    def run_parser(name, help):
        p = commands.add_parser(name, help=help)
        p.add_argument("--config", default="config/config.json")
        p.add_argument("--controller", choices=controllers.CONTROLLERS, help="default: the config's")
        p.add_argument("--opponent", choices=["none", "static"],
                       help="robot 2 parked in a corner, or holding its home (default: the config's)")
        p.add_argument("--mcap", action="store_true", help="also write a Foxglove MCAP file")
        p.add_argument("--gif", metavar="NAME", help="write gifs/NAME.gif instead of opening a window")
        shot = p.add_argument_group("shot_mpc")
        shot.add_argument("--workers", type=int, help="solver processes (default shot_mpc.n_workers; 0: serial)")
        shot.add_argument("--no-jit", action="store_true", help="interpreted NLP (no compile time)")
        shot.add_argument("--no-tracker", action="store_true", help="apply the plan's inputs open loop")
        return p

    p = run_parser("simulate", "run one episode from a config scenario")
    p.add_argument("--scenario", help="a config scenario (default: default)")
    p.add_argument("--no-animate", action="store_true")
    p.add_argument("--no-save", action="store_true")

    p = run_parser("evaluate", "run episodes from random starts and report the conversion rate")
    p.add_argument("--episodes", type=int, default=50)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--outcomes", nargs="+", help="with --gif/--mcap: only record these outcomes, e.g. miss no_contact")

    p = commands.add_parser("animate", help="animate a saved run")
    p.add_argument("pkl", nargs="?", help="default: the newest simulation_data/*.pkl")
    p.add_argument("--mode", choices=("standard", "dial"), default="standard",
                   help="dial: step through the DIAL planner's samples")
    p.add_argument("--gif", metavar="NAME", help="write gifs/NAME.gif instead of opening a window")

    p = commands.add_parser("mcap", help="write the Foxglove MCAP file of a saved run")
    p.add_argument("pkl")
    p.add_argument("--output", help="default: the pickle path with .mcap")

    args = parser.parse_args(argv)

    if args.command == "animate":
        from viz.animation import export_gif, play

        path, data, _ = load_run(args.pkl)
        print(f"Animating {path}")
        if args.gif:
            export_gif(data, args.gif)
        else:
            play(data, mode=args.mode)
        return
    if args.command == "mcap":
        from viz.foxglove_viz import write_mcap

        path, data, log = load_run(args.pkl)
        output = write_mcap(data, args.output or path.with_suffix(".mcap"), log=log,
                            metadata={"source_file": path.name}, start_ns=int(path.stat().st_mtime * 1e9))
        print(f"Saved MCAP to {output}")
        return

    config = load_config(args.config)
    if args.controller:
        config["controller"] = args.controller
    if args.opponent:
        config["simulation"]["opponent"] = args.opponent
    if args.no_jit:
        config["shot_mpc"]["jit"] = False
    if args.no_tracker:
        config["shot_mpc"]["tracker"]["enabled"] = False
    if args.command == "simulate" and args.scenario and args.scenario not in config["scenarios"]:
        parser.error(f"--scenario: one of {', '.join(config['scenarios'])}")

    factory = controllers.load(config, args.workers)  # before JAX starts: see controllers.load
    physics = physics_from_config(config)
    try:
        if args.command == "simulate":
            simulate(config, factory, physics, args.scenario, animate=not args.no_animate, gif=args.gif,
                     mcap=args.mcap, save=not args.no_save)
        else:
            evaluate(config, factory, physics, args.episodes, args.seed, gif=args.gif, mcap=args.mcap,
                     outcomes=args.outcomes)
    finally:
        factory.close()


if __name__ == "__main__":
    main()
