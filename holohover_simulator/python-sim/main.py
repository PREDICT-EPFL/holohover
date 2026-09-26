import argparse
import json
import pickle
from pathlib import Path

from simulation import run_sim
from animation import TrajecoryAnimation
from mcap_export import default_output_path, write_mcap


def _load_config(path="config/config0.json"):
    with open(path, "r") as f:
        return json.load(f)


def simulate(config_path="config/config0.json", export_format=None, output=None):
    config = _load_config(config_path)
    simulation_data = run_sim(config, save_data=False)

    if export_format is None:
        return

    output = Path(output) if output else default_output_path().with_suffix(f".{export_format}")
    output.parent.mkdir(parents=True, exist_ok=True)

    if export_format == "mcap":
        write_mcap(simulation_data, output)
    elif export_format == "pkl":
        with output.open("wb") as f:
            pickle.dump(simulation_data, f)
    else:
        raise ValueError(f"Unsupported export format: {export_format}")

    print(f"Saved {export_format.upper()} to {output}")
    return output


def _get_latest_pkl():
    p = Path("simulation_data")
    pkl_files = list(p.glob("*.pkl"))

    if not pkl_files:
        raise FileNotFoundError(f"No .pkl files found")

    latest_file = max(pkl_files, key=lambda f: f.stat().st_mtime)
    return latest_file


def animate(path=None, export=None, mode="standard"):
    if path is None:
        path = _get_latest_pkl()

    with open(path, "rb") as f:
        print(f"Animating from {path}")
        simulation_data = pickle.load(f)

    animation = TrajecoryAnimation(**simulation_data)

    if not export:
        animation.play(mode=mode)
    else:
        animation.export_animation(filename=export)


def simulate_and_animate(mode="standard", config_path="config/config0.json"):
    config = _load_config(config_path)
    simulation_data = run_sim(config, save_data=False)
    animation = TrajecoryAnimation(**simulation_data)
    animation.play(mode=mode)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run or visualize a Holohover simulation.")
    commands = parser.add_subparsers(dest="command", required=True)

    simulate_parser = commands.add_parser("simulate", help="Run a simulation, optionally exporting its data.")
    simulate_parser.add_argument("--config", default="config/config0.json", help="Simulation config JSON.")
    simulate_parser.add_argument("--export", choices=("mcap", "pkl"), help="Export simulation data in this format.")
    simulate_parser.add_argument("--output", help="Export path; defaults to a timestamped path.")

    animate_parser = commands.add_parser("animate", help="Animate saved simulation data.")
    animate_parser.add_argument("--path", help="Simulation pickle; defaults to the newest file.")
    animate_parser.add_argument("--export", help="Export the animation to this filename.")
    animate_parser.add_argument("--mode", choices=("standard", "dial"), default="standard")

    combined_parser = commands.add_parser(
        "simulate-and-animate", help="Run a simulation and animate the result."
    )
    combined_parser.add_argument("--config", default="config/config0.json", help="Simulation config JSON.")
    combined_parser.add_argument("--mode", choices=("standard", "dial"), default="standard")

    args = parser.parse_args(argv)
    if args.command == "simulate":
        if args.output and not args.export:
            parser.error("--output requires --export")
        simulate(config_path=args.config, export_format=args.export, output=args.output)
    elif args.command == "animate":
        animate(path=args.path, export=args.export, mode=args.mode)
    elif args.command == "simulate-and-animate":
        simulate_and_animate(mode=args.mode, config_path=args.config)

if __name__ == "__main__":
    main()
