import json
import pickle
from pathlib import Path

from simulation import run_sim
from animation import TrajecoryAnimation
from mcap_export import default_output_path, write_mcap


def _load_config(path="config/config0.json"):
    with open(path, "r") as f:
        return json.load(f)


def simulate():
    config = _load_config()
    run_sim(config, save_data=True)


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


def simulate_and_animate(mode="standard"):
    config = _load_config()
    simulation_data = run_sim(config, save_data=False)
    animation = TrajecoryAnimation(**simulation_data)
    animation.play(mode=mode)


def simulate_and_export(output=None):
    config = _load_config()
    simulation_data = run_sim(config, save_data=False)
    output = output or default_output_path()
    write_mcap(simulation_data, output)
    print(f"Saved MCAP to {output}")
    return output


if __name__ == "__main__":
    simulate_and_animate("standard")
    # simulate_and_export()
    #simulate_and_animate("dial")
    # simulate()
    #animate("simulation_data/tuned_going_off_top.pkl", mode="standard")
    #animate(mode="dial")
    # animate(export="first_success_with_r2_avoid")
    # animate("simulation_data/enemy_puck_first.pkl", mode="dial")
